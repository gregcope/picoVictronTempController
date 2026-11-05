# main.py for picoVictronTempController
# DS18B20 Temp Monitoring, Dual Relay Fan Control, Immutable Failsafe, MQTT Hysteresis & ugit OTA Updates

import asyncio
import gc
import json
import os
import sys
import time
import machine
import network
import secrets
import ugit
import onewire
import ds18x20
from ubinascii import hexlify

try:
    import logging
except ImportError:
    import ulogging as logging

try:
    from simple import MQTTClient
except ImportError:
    from umqtt.simple import MQTTException, MQTTClient

app_version = "1.6.1"

# System Logging Configuration
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("victron_controller")

# ==========================================
# 1. CONFIGURATION
# ==========================================
client_id = "pico_victron_controller"
sensor_names = ["InverterBottom", "InverterTop", "BatteryCables", "VictronLynx"]

# Time and Threshold Variables
sensor_interval_secs = 10
heartbeat_interval_secs = 300
sensor_change_threshold_c = 0.3

# --- Operational & Safety Thresholds ---
# 1. CRITICAL_OVERRIDE_TEMP_C: Immutable local hardware safety floor (NOT changeable via MQTT).
# 2. FAN_ON_THRESHOLD_C / FAN_OFF_THRESHOLD_C: Operational hysteresis setpoints (Configurable via MQTT).
CRITICAL_OVERRIDE_TEMP_C = 50.0  
FAN_ON_THRESHOLD_C = 35.0
FAN_OFF_THRESHOLD_C = 30.0

# Hardware Pin Assignments
ONEWIRE_PIN_NUM = 15
RELAY_FAN_1_PIN_NUM = 14
RELAY_FAN_2_PIN_NUM = 13

# Hardware and Network Timing
watchdog_timeout_ms = 8000
mqtt_keepalive_grace_secs = 20
initial_backoff_secs = 1.0
max_backoff_secs = 60.0
max_failed_attempts = 20

# --- System Internals ---
system_start_time_secs = time.time()
watchdog = None
onboard_led = machine.Pin("LED", machine.Pin.OUT, value=1)
internal_temp_sensor = machine.ADC(4)
device_serial = hexlify(machine.unique_id()).decode()

system_status = "nominal - starting"
last_published_status = ""
force_sensor_publish = False


class SensorReadError(Exception):
    """Raised when an individual one-wire sensor read fails."""
    pass


def check_fatal_crash_log() -> str:
    """Read fatal crash log without deleting until published."""
    try:
        with open("/fatal_crash.log", "r") as crash_file:
            log_content = crash_file.read().strip()
        return log_content[:250] if log_content else "Empty Crash Log"
    except OSError:
        return ""


last_crash_log = check_fatal_crash_log()


def get_reset_cause() -> str:
    """Determine reset cause prioritizing crash dumps and OTA updates[cite: 3, 4]."""
    if last_crash_log:
        return f"Fatal Crash: {last_crash_log}"

    try:
        with open("/ugit_log.txt", "r") as log_file:
            log_data = log_file.read()
        if "OTA Interrupted" in log_data:
            os.remove("/ugit_log.txt")
            return "OTA Interrupted - Rolled Back"
        if "/main.py updated" in log_data:
            os.remove("/ugit_log.txt")
            return "Firmware Upgrade"
    except OSError:
        pass

    cause = machine.reset_cause()
    if cause == machine.PWRON_RESET:
        return "Power On"
    if cause == machine.WDT_RESET:
        return "Watchdog Timer"
    if cause == machine.SOFT_RESET:
        return "Software Reset"
    return "Unknown"


last_reset_reason = get_reset_cause()

for _ in range(10):
    onboard_led.toggle()
    time.sleep(0.2)


# ==========================================
# 2. DUAL FAN RELAY CONTROLLER
# ==========================================
class DualFanController:
    """Controls two standard non-latching relay switches for dual 12V fan banks."""

    def __init__(self, pin_fan_1: int, pin_fan_2: int) -> None:
        self.fan_1_pin = machine.Pin(pin_fan_1, machine.Pin.OUT, value=0)
        self.fan_2_pin = machine.Pin(pin_fan_2, machine.Pin.OUT, value=0)
        self.current_state = "OFF"

    def set_fans(self, state: str) -> None:
        """Set both fans to ON or OFF."""
        if state.upper() == "ON":
            self.fan_1_pin.value(1)
            self.fan_2_pin.value(1)
            self.current_state = "ON"
            logger.info("Dual Relays: Fans ON")
        else:
            self.fan_1_pin.value(0)
            self.fan_2_pin.value(0)
            self.current_state = "OFF"
            logger.info("Dual Relays: Fans OFF")


# ==========================================
# 3. SENSOR MANAGER (DS18B20 & Median Filtering)
# ==========================================
class SensorManager:
    """Manages DS18B20 temperature sensors on a one-wire bus."""

    def __init__(self, data_pin_num: int, sensor_list: list) -> None:
        self.data_pin = machine.Pin(data_pin_num)
        self.ds_bus = onewire.OneWire(self.data_pin)
        self.ds_sensor = ds18x20.DS18X20(self.ds_bus)
        self.roms_cache = []
        self.sensors = []
        for name in sensor_list:
            self.sensors.append({
                'sensor_name': name,
                'sensor_id': name.lower(),
                'last_temp_c': -999.0,
                'last_avail': 'unknown',
                'history': [],
                'fail_count': 0
            })

    def start_conversion(self) -> bool:
        """Starts a temperature conversion for all devices on the bus[cite: 3, 4]."""
        global system_status
        try:
            system_status = "nominal - scanning bus"
            self.roms_cache = self.ds_sensor.scan()
            if not self.roms_cache:
                system_status = "error - 1-wire bus 0 roms"
                return False
            system_status = "nominal - converting temp"
            self.ds_sensor.convert_temp()
            return True
        except (onewire.OneWireError, OSError) as err:
            system_status = f"error - 1-wire bus error: {err}"
            return False

    def read_and_evaluate(self, force_publish: bool = False) -> tuple[list, float]:
        """Reads temperatures, applies median filtering, and evaluates thresholds."""
        global system_status
        updates = []
        bus_fault_detected = False
        max_temp_c = -999.0

        try:
            system_status = "nominal - reading sensors"
            for idx, s in enumerate(self.sensors):
                temp_c = None
                if idx < len(self.roms_cache):
                    try:
                        t = self.ds_sensor.read_temp(self.roms_cache[idx])
                        if t != 85.0 and t != -127.0:
                            temp_c = t
                    except (onewire.OneWireError, OSError):
                        bus_fault_detected = True

                if temp_c is not None:
                    s['fail_count'] = 0
                    s['history'].append(temp_c)
                    if len(s['history']) > 3:
                        s['history'].pop(0)
                    sorted_history = sorted(s['history'])
                    filtered_temp_c = sorted_history[1] if len(sorted_history) == 3 else sorted_history[-1]
                    filtered_temp_c = round(filtered_temp_c, 1)
                    current_avail = "online"
                    final_temp_to_evaluate_c = filtered_temp_c
                else:
                    s['fail_count'] += 1
                    final_temp_to_evaluate_c = None
                    current_avail = "offline" if s['fail_count'] >= 3 else s['last_avail']

                if current_avail == "online" and final_temp_to_evaluate_c is not None:
                    if final_temp_to_evaluate_c > max_temp_c:
                        max_temp_c = final_temp_to_evaluate_c

                if force_publish or current_avail != s['last_avail']:
                    updates.append({
                        'topic': f"homeassistant/sensor/{client_id}_{s['sensor_id']}/availability",
                        'payload': current_avail
                    })
                    s['last_avail'] = current_avail

                if current_avail == "online" and final_temp_to_evaluate_c is not None:
                    if force_publish or abs(final_temp_to_evaluate_c - s['last_temp_c']) >= sensor_change_threshold_c:
                        updates.append({
                            'topic': f"homeassistant/sensor/{client_id}_{s['sensor_id']}/state",
                            'payload': {"temperature": final_temp_to_evaluate_c}
                        })
                        s['last_temp_c'] = final_temp_to_evaluate_c

            if system_status.startswith("error - 1-wire") and not bus_fault_detected:
                system_status = "nominal - mainloop"

        except (onewire.OneWireError, OSError) as err:
            system_status = f"error - sensor bus exception: {err}"

        return updates, max_temp_c


# ==========================================
# 4. NETWORK MANAGER
# ==========================================
class NetworkManager:
    """Manages WiFi connectivity, MQTT communication, exponential backoff, and discovery payloads."""

    def __init__(self, client_id: str, broker: str, user: str, password: str) -> None:
        self.client_id = client_id
        self.broker = broker
        self.user = user
        self.password = password
        self.client = None
        self.failed_attempts = 0
        self.reconnects = 0
        self.current_backoff = initial_backoff_secs

        self.master_avail_topic = f"homeassistant/sensor/{self.client_id}/availability"
        self.relay_state_topic = f"homeassistant/switch/{self.client_id}_fans/state"
        self.relay_cmd_topic = f"homeassistant/switch/{self.client_id}_fans/set"
        
        # Operational Hysteresis Setpoints
        self.fan_on_state_topic = f"homeassistant/number/{self.client_id}_fan_on/state"
        self.fan_on_cmd_topic = f"homeassistant/number/{self.client_id}_fan_on/set"
        self.fan_off_state_topic = f"homeassistant/number/{self.client_id}_fan_off/state"
        self.fan_off_cmd_topic = f"homeassistant/number/{self.client_id}_fan_off/set"

        self.ota_cmd_topic = f"homeassistant/button/{self.client_id}_ota/set"

    async def maintain_connection(self) -> bool:
        """Maintain robust network and MQTT connectivity using exponential backoff."""
        global system_status
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)

        if wlan.isconnected() and self.client is not None:
            return True

        system_status = "nominal - connecting wifi"
        if not wlan.isconnected():
            wlan.connect(secrets.wifiSsid, secrets.wifiPassword)
            for _ in range(10):
                if wlan.isconnected():
                    break
                if watchdog is not None:
                    watchdog.feed()
                await asyncio.sleep(1)

        if wlan.isconnected():
            try:
                system_status = "nominal - connecting mqtt"
                unique_id = self.client_id + "_" + device_serial
                self.client = MQTTClient(
                    unique_id,
                    self.broker,
                    user=self.user,
                    password=self.password,
                    keepalive=heartbeat_interval_secs + mqtt_keepalive_grace_secs,
                )

                lwt_topic_bytes = self.master_avail_topic.encode("utf-8")
                self.client.set_last_will(lwt_topic_bytes, b"offline", retain=True)

                self.client.connect()

                # Enforce non-blocking socket timeout on reads and writes[cite: 3, 4]
                if hasattr(self.client, "sock") and self.client.sock is not None:
                    self.client.sock.settimeout(5.0)

                self.publish(self.master_avail_topic, "online", retain=True)

                # --- OTA Validation Handshake ---
                try:
                    os.remove("/ota_pending.flag")
                except OSError:
                    pass

                self.reconnects += 1
                self.failed_attempts = 0
                self.current_backoff = initial_backoff_secs
                system_status = "nominal - mainloop"
                return True
            except (OSError, MQTTException) as err:
                logger.error("MQTT connection failed: %s", err)
                self.client = None

        self.failed_attempts += 1
        system_status = f"error - network ({self.failed_attempts}/{max_failed_attempts})"

        sleep_steps = int(self.current_backoff)
        for _ in range(max(1, sleep_steps)):
            if watchdog is not None:
                watchdog.feed()
            await asyncio.sleep(1)

        self.current_backoff = min(self.current_backoff * 2.0, max_backoff_secs)

        if self.failed_attempts >= max_failed_attempts:
            system_status = "error - max network retries, rebooting"
            logger.error(system_status)
            time.sleep(1)
            machine.reset()

        return False

    def publish(self, topic: str, payload_data, retain: bool = True) -> bool:
        """Publish data payload to specified MQTT topic[cite: 3, 4]."""
        if self.client is None:
            return False
        try:
            t_bytes = topic.encode("utf-8")
            p_bytes = (
                json.dumps(payload_data).encode("utf-8")
                if isinstance(payload_data, dict)
                else str(payload_data).encode("utf-8")
            )
            self.client.publish(t_bytes, p_bytes, retain=retain)
            return True
        except (OSError, MQTTException) as err:
            logger.error("MQTT publish error to %s: %s", topic, err)
            self.client = None
            return False

    def check_messages(self) -> None:
        """Check for incoming MQTT messages[cite: 3, 4]."""
        if self.client:
            try:
                self.client.check_msg()
            except (OSError, MQTTException) as err:
                logger.error("MQTT check_msg error: %s", err)
                self.client = None

    async def send_discovery(self, sensors_list: list) -> bool:
        """Send Home Assistant MQTT discovery configuration non-blocking."""
        try:
            system_status = "nominal - sending discovery"
            device_info = {
                "identifiers": [self.client_id, device_serial],
                "name": "Victron Temp and Fan Controller",
                "model": "Raspberry Pi Pico 2 W",
                "sw_version": app_version,
                "serial_number": device_serial,
            }

            # Register Dual Fan Relay Switch
            switch_config_topic = f"homeassistant/switch/{self.client_id}_fans/config"
            switch_payload = {
                "name": "Fans",
                "unique_id": f"{self.client_id}_fans",
                "command_topic": self.relay_cmd_topic,
                "state_topic": self.relay_state_topic,
                "availability_topic": self.master_avail_topic,
                "device": device_info,
            }
            self.publish(switch_config_topic, switch_payload)
            await asyncio.sleep(0.1)

            # Register Fan ON Threshold Number Entity
            fan_on_config_topic = f"homeassistant/number/{self.client_id}_fan_on/config"
            fan_on_payload = {
                "name": "Fan ON Threshold",
                "unique_id": f"{self.client_id}_fan_on",
                "command_topic": self.fan_on_cmd_topic,
                "state_topic": self.fan_on_state_topic,
                "unit_of_measurement": "°C",
                "min": 20.0,
                "max": 60.0,
                "step": 0.5,
                "mode": "slider",
                "availability_topic": self.master_avail_topic,
                "device": device_info,
            }
            self.publish(fan_on_config_topic, fan_on_payload)
            await asyncio.sleep(0.1)

            # Register Fan OFF Threshold Number Entity
            fan_off_config_topic = f"homeassistant/number/{self.client_id}_fan_off/config"
            fan_off_payload = {
                "name": "Fan OFF Threshold",
                "unique_id": f"{self.client_id}_fan_off",
                "command_topic": self.fan_off_cmd_topic,
                "state_topic": self.fan_off_state_topic,
                "unit_of_measurement": "°C",
                "min": 15.0,
                "max": 55.0,
                "step": 0.5,
                "mode": "slider",
                "availability_topic": self.master_avail_topic,
                "device": device_info,
            }
            self.publish(fan_off_config_topic, fan_off_payload)
            await asyncio.sleep(0.1)

            # Register OTA Update Button
            ota_config_topic = f"homeassistant/button/{self.client_id}_ota/config"
            ota_payload = {
                "name": "Update Firmware",
                "unique_id": f"{self.client_id}_ota",
                "command_topic": self.ota_cmd_topic,
                "device_class": "update",
                "availability_topic": self.master_avail_topic,
                "device": device_info,
            }
            self.publish(ota_config_topic, ota_payload)
            await asyncio.sleep(0.1)

            # Register DS18B20 Temperature Sensors
            for s in sensors_list:
                topic = f"homeassistant/sensor/{self.client_id}_{s['sensor_id']}_T/config"
                payload = {
                    "name": f"{s['sensor_name']} Temp",
                    "unique_id": f"{self.client_id}_{s['sensor_id']}_T",
                    "state_topic": f"homeassistant/sensor/{s['sensor_id']}/state",
                    "availability": [
                        {"topic": self.master_avail_topic},
                        {"topic": f"homeassistant/sensor/{s['sensor_id']}/availability"}
                    ],
                    "availability_mode": "all",
                    "unit_of_measurement": "°C",
                    "device_class": "temperature",
                    "state_class": "measurement",
                    "value_template": "{{ value_json.temperature }}",
                    "device": device_info
                }
                self.publish(topic, payload)
                await asyncio.sleep(0.1)

            # Register Diagnostic Sensors
            sys_sensors = [
                ("pico_temp", "Internal CPU Temp", "temperature", "°C", "measurement"),
                ("rssi", "Signal Strength", "signal_strength", "dBm", "measurement"),
                ("uptime", "Uptime", "duration", "s", None),
                ("version", "Firmware Version", None, None, None),
                ("reconnects", "Reconnect Count", None, None, None),
                ("last_reset", "Last Reset Reason", None, None, None),
                ("crash_log", "Crash Log", None, None, None),
                ("status", "System Status", None, None, None)
            ]

            for key, name, d_class, unit, s_class in sys_sensors:
                topic = f"homeassistant/sensor/{self.client_id}_{key}/config"
                payload = {
                    "name": name,
                    "unique_id": f"{self.client_id}_{key}",
                    "state_topic": f"homeassistant/sensor/{self.client_id}_sys/state",
                    "availability_topic": self.master_avail_topic,
                    "value_template": f"{{{{ value_json.{key} }}}}",
                    "entity_category": "diagnostic",
                    "device": device_info
                }
                if d_class:
                    payload["device_class"] = d_class
                if unit:
                    payload["unit_of_measurement"] = unit
                if s_class:
                    payload["state_class"] = s_class
                self.publish(topic, payload)
                await asyncio.sleep(0.1)

            return True
        except (OSError, ValueError) as err:
            logger.error("Discovery error: %s", err)
            return False


# ==========================================
# 5. MAIN ASYNC LOOP
# ==========================================
async def main() -> None:
    global system_status, last_published_status, force_sensor_publish, watchdog, FAN_ON_THRESHOLD_C, FAN_OFF_THRESHOLD_C, last_crash_log

    watchdog = machine.WDT(timeout=watchdog_timeout_ms)

    sensors_manager = SensorManager(data_pin_num=ONEWIRE_PIN_NUM, sensor_list=sensor_names)
    fans = DualFanController(pin_fan_1=RELAY_FAN_1_PIN_NUM, pin_fan_2=RELAY_FAN_2_PIN_NUM)
    network_controller = NetworkManager(
        client_id, secrets.mqttBroker, secrets.mqttUser, secrets.mqttPassword
    )

    # Track manual override state from Home Assistant
    manual_override_active = False

    # MQTT message callback handler
    def sub_cb(topic: bytes, msg: bytes) -> None:
        nonlocal manual_override_active
        global FAN_ON_THRESHOLD_C, FAN_OFF_THRESHOLD_C, system_status
        topic_str = topic.decode("utf-8")
        msg_str = msg.decode("utf-8")

        # Configurable via MQTT: Manual Fan Switch State
        if topic_str == network_controller.relay_cmd_topic:
            if msg_str.upper() == "ON":
                manual_override_active = True
                fans.set_fans("ON")
                network_controller.publish(network_controller.relay_state_topic, "ON", retain=True)
            elif msg_str.upper() == "OFF":
                manual_override_active = False
                fans.set_fans("OFF")
                network_controller.publish(network_controller.relay_state_topic, "OFF", retain=True)

        # Configurable via MQTT: Fan ON Threshold Slider
        elif topic_str == network_controller.fan_on_cmd_topic:
            try:
                new_temp = float(msg_str)
                FAN_ON_THRESHOLD_C = new_temp
                logger.info("Fan ON threshold updated via MQTT to: %s°C", FAN_ON_THRESHOLD_C)
                network_controller.publish(network_controller.fan_on_state_topic, str(FAN_ON_THRESHOLD_C), retain=True)
            except ValueError:
                logger.error("Invalid Fan ON temperature received: %s", msg_str)

        # Configurable via MQTT: Fan OFF Threshold Slider
        elif topic_str == network_controller.fan_off_cmd_topic:
            try:
                new_temp = float(msg_str)
                FAN_OFF_THRESHOLD_C = new_temp
                logger.info("Fan OFF threshold updated via MQTT to: %s°C", FAN_OFF_THRESHOLD_C)
                network_controller.publish(network_controller.fan_off_state_topic, str(FAN_OFF_THRESHOLD_C), retain=True)
            except ValueError:
                logger.error("Invalid Fan OFF temperature received: %s", msg_str)

        elif topic_str == network_controller.ota_cmd_topic:
            if msg_str.upper() == "PRESS":
                try:
                    system_status = "nominal - ota initiating"
                    internal_volts = internal_temp_sensor.read_u16() * (3.3 / 65535)
                    pico_temp_c = round(27 - (internal_volts - 0.706) / 0.001721, 1)
                    sys_data = {
                        "pico_temp": pico_temp_c,
                        "rssi": network.WLAN(network.STA_IF).status("rssi"),
                        "uptime": time.time() - system_start_time_secs,
                        "version": app_version,
                        "reconnects": network_controller.reconnects,
                        "last_reset": last_reset_reason,
                        "crash_log": last_crash_log,
                        "status": system_status,
                    }

                    network_controller.publish(f"homeassistant/sensor/{client_id}_sys/state", sys_data)

                    with open("/ota_pending.flag", "w") as f:
                        f.write("pending")

                    try:
                        os.remove("/main_backup.py")
                    except OSError:
                        pass

                    os.rename("/main.py", "/main_backup.py")

                    system_status = "nominal - ota downloading"
                    sys_data["status"] = system_status
                    network_controller.publish(f"homeassistant/sensor/{client_id}_sys/state", sys_data)

                    ugit.pull_all(
                        isconnected=True,
                        ignore=[
                            "/README.md",
                            "/secrets.py",
                            "/secrets_example.py",
                            "/config.json",
                            "/main_backup.py",
                            "/LICENSE",
                        ],
                    )

                    try:
                        with open("/ugit_log.txt", "r") as log_file:
                            log_lines = log_file.readlines()

                        updated_files = []
                        for line in log_lines:
                            clean_line = line.strip()
                            if "updated" in clean_line or "downloaded" in clean_line:
                                parts = clean_line.split(" ")
                                if parts:
                                    file_name = parts[0].lstrip("/")
                                    if file_name not in updated_files:
                                        updated_files.append(file_name)

                        if updated_files:
                            system_status = f"nominal - ota files: {', '.join(updated_files)}"
                        else:
                            system_status = "nominal - ota no files changed"

                        sys_data["status"] = system_status
                        network_controller.publish(f"homeassistant/sensor/{client_id}_sys/state", sys_data)
                        time.sleep(2)

                    except OSError:
                        pass

                    try:
                        os.remove("/ota_pending.flag")
                    except OSError:
                        pass

                    system_status = "nominal - ota complete, rebooting"
                    sys_data["status"] = system_status
                    network_controller.publish(f"homeassistant/sensor/{client_id}_sys/state", sys_data)
                    time.sleep(1)
                    machine.reset()

                except Exception as err:
                    system_status = f"error - ota failed: {err}"
                    sys_data["status"] = system_status
                    network_controller.publish(f"homeassistant/sensor/{client_id}_sys/state", sys_data)
                    try:
                        os.rename("/main_backup.py", "/main.py")
                        os.remove("/ota_pending.flag")
                    except OSError:
                        pass
                    time.sleep(2)
                    machine.reset()

    discovery_sent = False
    last_heartbeat_time_secs = 0
    last_read_time_secs = 0
    last_loop_success_secs = time.time()

    while True:
        if watchdog is not None:
            watchdog.feed()
        now_secs = time.time()

        # Software Deadlock Guard: Force reboot ONLY if connected to MQTT but loop processing stalled > 120s[cite: 3, 4]
        if network_controller.client is not None and (now_secs - last_loop_success_secs > 120):
            system_status = "error - main loop deadlock, forcing reset"
            logger.error(system_status)
            time.sleep(1)
            machine.reset()

        # 1. Maintain Network Connection
        if not await network_controller.maintain_connection():
            discovery_sent = False
            last_loop_success_secs = now_secs  # Reset timer during network backoff[cite: 3, 4]
            await asyncio.sleep(2)
            continue

        if network_controller.client and not discovery_sent:
            network_controller.client.set_callback(sub_cb)
            network_controller.client.subscribe(network_controller.relay_cmd_topic.encode("utf-8"))
            network_controller.client.subscribe(network_controller.fan_on_cmd_topic.encode("utf-8"))
            network_controller.client.subscribe(network_controller.fan_off_cmd_topic.encode("utf-8"))
            network_controller.client.subscribe(network_controller.ota_cmd_topic.encode("utf-8"))

            if await network_controller.send_discovery(sensors_manager.sensors):
                discovery_sent = True
                network_controller.publish(network_controller.relay_state_topic, fans.current_state, retain=True)
                network_controller.publish(network_controller.fan_on_state_topic, str(FAN_ON_THRESHOLD_C), retain=True)
                network_controller.publish(network_controller.fan_off_state_topic, str(FAN_OFF_THRESHOLD_C), retain=True)
                last_heartbeat_time_secs = now_secs - heartbeat_interval_secs
                last_read_time_secs = now_secs - sensor_interval_secs

        network_controller.check_messages()

        # 2. Dynamic System Telemetry Broadcast
        force_heartbeat = (now_secs - last_heartbeat_time_secs >= heartbeat_interval_secs)
        status_changed = system_status != last_published_status

        if (force_heartbeat or status_changed) and network_controller.client is not None:
            internal_volts = internal_temp_sensor.read_u16() * (3.3 / 65535)
            pico_temp_c = round(27 - (internal_volts - 0.706) / 0.001721, 1)

            sys_data = {
                "pico_temp": pico_temp_c,
                "rssi": network.WLAN(network.STA_IF).status("rssi"),
                "uptime": time.time() - system_start_time_secs,
                "version": app_version,
                "reconnects": network_controller.reconnects,
                "last_reset": last_reset_reason,
                "crash_log": last_crash_log,
                "status": system_status,
            }
            if network_controller.publish(f"homeassistant/sensor/{client_id}_sys/state", sys_data):
                # Safely delete fatal crash log only after successful MQTT transmission[cite: 3, 4]
                if last_crash_log:
                    try:
                        os.remove("/fatal_crash.log")
                        last_crash_log = ""
                    except OSError:
                        pass

            last_published_status = system_status
            if force_heartbeat:
                force_sensor_publish = True
                last_heartbeat_time_secs = now_secs

        # 3. Poll Sensors & Apply Fallback / Hysteresis Logic
        if now_secs - last_read_time_secs >= sensor_interval_secs:
            system_status = "nominal - sensor conversion"
            bus_active = sensors_manager.start_conversion()
            if not bus_active:
                for s in sensors_manager.sensors:
                    if s['last_avail'] != 'offline':
                        network_controller.publish(f"homeassistant/sensor/{client_id}_{s['sensor_id']}/availability", "offline")
                        s['last_avail'] = 'offline'
                last_read_time_secs = now_secs
            else:
                await asyncio.sleep_ms(750)

                ready_payloads, max_temp_c = sensors_manager.read_and_evaluate(force_publish=force_sensor_publish)
                force_sensor_publish = False

                for item in ready_payloads:
                    network_controller.publish(item['topic'], item['payload'])

                # --- Fan Control Logic (Safety Fallback vs Hysteresis vs Manual) ---
                if max_temp_c > -900.0:
                    if max_temp_c >= CRITICAL_OVERRIDE_TEMP_C:
                        # Safety Critical Override: Force fans ON locally regardless of network or manual states
                        if fans.current_state != "ON":
                            fans.set_fans("ON")
                            if network_controller.client:
                                network_controller.publish(network_controller.relay_state_topic, "ON", retain=True)
                    elif not manual_override_active:
                        # Normal Automatic Hysteresis (Configurable via Home Assistant sliders)
                        if fans.current_state != "ON" and max_temp_c >= FAN_ON_THRESHOLD_C:
                            fans.set_fans("ON")
                            if network_controller.client:
                                network_controller.publish(network_controller.relay_state_topic, "ON", retain=True)
                        elif fans.current_state != "OFF" and max_temp_c <= FAN_OFF_THRESHOLD_C:
                            fans.set_fans("OFF")
                            if network_controller.client:
                                network_controller.publish(network_controller.relay_state_topic, "OFF", retain=True)
                else:
                    # COMPLETE SENSOR FAILURE BLIND FAILSAFE: Force fans ON
                    if not manual_override_active and fans.current_state != "ON":
                        fans.set_fans("ON")
                        logger.warning("Failsafe activated: All sensors offline, forcing fans ON")
                        if network_controller.client:
                            network_controller.publish(network_controller.relay_state_topic, "ON", retain=True)

                last_read_time_secs = time.time()
                system_status = "nominal - mainloop"
                gc.collect()

        # Mark loop cycle success timestamp
        last_loop_success_secs = now_secs

        await asyncio.sleep(0.1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        # Catch fatal unhandled top-level crashes to flash before rebooting[cite: 3, 4]
        try:
            with open("/fatal_crash.log", "w") as crash_file:
                sys.print_exception(exc, crash_file)
        except OSError:
            pass
        time.sleep(1)
        machine.reset()