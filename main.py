import asyncio
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
    from simple import MQTTClient
except ImportError:
    from umqtt.simple import MQTTException, MQTTClient

app_version = "1.6.6"

# ==========================================
# 1. CONFIGURATION & HARDWARE SETUP
# ==========================================
client_id = "pico_victron_controller"
sensor_names = ["InverterBottom", "InverterTop", "BatteryCables", "VictronLynx"]

# Pin Definitions
ONE_WIRE_PIN = 15
FAN_1_PIN = 14
FAN_2_PIN = 13

# Fan Control & Hysteresis Defaults (Configurable via MQTT)
fan_on_threshold_c = 35.0
fan_off_threshold_c = 28.0
CRITICAL_OVERRIDE_TEMP_C = 50.0  # Immutable hardware failsafe

# Timing Variables
sensor_interval_secs = 5 
heartbeat_interval_secs = 300 
sensor_change_threshold_c = 0.3 

# Hardware and Network Timing
watchdog_timeout_ms = 8000 
mqtt_keepalive_grace_secs = 20
initial_backoff_secs = 1.0
max_backoff_secs = 60.0
max_failed_attempts = 20

# --- System Internals ---
system_start_time_secs = time.time()
watchdog = None
onboard_led = machine.Pin("LED", machine.Pin.OUT, value=0)
internal_temp_sensor = machine.ADC(4)
device_serial = hexlify(machine.unique_id()).decode()

# Fan Relay Outputs
fan_1_relay = machine.Pin(FAN_1_PIN, machine.Pin.OUT, value=0)
fan_2_relay = machine.Pin(FAN_2_PIN, machine.Pin.OUT, value=0)
fans_currently_active = False

system_status = "nominal - starting"
last_published_status = ""
force_sensor_publish = False 


def flash_led(duration_ms: int = 30) -> None:
    """Briefly flash the onboard LED to indicate activity."""
    try:
        onboard_led.value(1)
        time.sleep_ms(duration_ms)
        onboard_led.value(0)
    except Exception:
        pass


def set_fans(state: bool) -> None:
    """Set physical relay state for dual fan banks."""
    global fans_currently_active
    fans_currently_active = state
    val = 1 if state else 0
    fan_1_relay.value(val)
    fan_2_relay.value(val)
    if state:
        flash_led(50)


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
    """Determine reset cause prioritizing crash dumps and OTA updates."""
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

# Brief startup double-blink to confirm clean boot
for _ in range(2):
    onboard_led.value(1)
    time.sleep(0.15)
    onboard_led.value(0)
    time.sleep(0.15)


# ==========================================
# 2. THE SENSOR LIBRARY (Median Filtering & Fan Logic)
# ==========================================
class SensorManager:
    def __init__(self, data_pin_num: int, sensor_list: list) -> None:
        self.data_pin = machine.Pin(data_pin_num)
        self.ds_bus = onewire.OneWire(self.data_pin)
        self.ds_sensor = ds18x20.DS18X20(self.ds_bus)
        self.roms_cache = []
        self.sensors = []
        for name in sensor_list:
            self.sensors.append({
                'sensor_name': name, 'sensor_id': name.lower(), 
                'last_temp_c': -999.0, 'last_avail': 'unknown', 
                'history': [], 'fail_count': 0 
            })

    def start_conversion(self) -> bool:
        """Starts a temperature conversion and logs bus status for debugging."""
        global system_status
        try:
            system_status = "nominal - scanning bus"
            self.roms_cache = self.ds_sensor.scan()
            if not self.roms_cache: 
                system_status = "error - 1-wire bus 0 roms"
                return False
            system_status = "nominal - converting temp"
            self.ds_sensor.convert_temp()
            flash_led(20)
            return True
        except (onewire.OneWireError, OSError) as err:
            system_status = f"error - 1-wire bus error: {err}"
            return False

    def read_and_evaluate(self, force_publish: bool = False) -> list:
        global system_status, fans_currently_active
        updates = []
        bus_fault_detected = False
        max_active_temp_c = -999.0

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
                    
                    if final_temp_to_evaluate_c > max_active_temp_c:
                        max_active_temp_c = final_temp_to_evaluate_c
                else:
                    s['fail_count'] += 1
                    final_temp_to_evaluate_c = None
                    current_avail = "offline" if s['fail_count'] >= 3 else s['last_avail'] 
                
                if force_publish or current_avail != s['last_avail']:
                    updates.append({'topic': f"homeassistant/sensor/{s['sensor_id']}/availability", 'payload': current_avail})
                    s['last_avail'] = current_avail
                    
                if current_avail == "online" and final_temp_to_evaluate_c is not None:
                    if force_publish or abs(final_temp_to_evaluate_c - s['last_temp_c']) >= sensor_change_threshold_c:
                        updates.append({'topic': f"homeassistant/sensor/{s['sensor_id']}/state", 'payload': {"temperature": final_temp_to_evaluate_c}})
                        s['last_temp_c'] = final_temp_to_evaluate_c
                        
            # --- Fan Control & Failsafe Evaluation ---
            if max_active_temp_c >= CRITICAL_OVERRIDE_TEMP_C:
                if not fans_currently_active:
                    set_fans(True)
                system_status = f"error - CRITICAL OVERRIDE ACTIVE ({max_active_temp_c}°C)"
            else:
                if fans_currently_active and max_active_temp_c <= fan_off_threshold_c:
                    set_fans(False)
                elif not fans_currently_active and max_active_temp_c >= fan_on_threshold_c:
                    set_fans(True)

            if system_status.startswith("error - 1-wire") and not bus_fault_detected:
                system_status = "nominal - mainloop"
                
        except (onewire.OneWireError, OSError) as err: 
            system_status = f"error - sensor bus exception: {err}"
        
        # Include fan state in updates
        updates.append({'topic': f"homeassistant/switch/{client_id}_relay/state", 'payload': "ON" if fans_currently_active else "OFF"})
        return updates


# ==========================================
# 3. THE NETWORK MANAGER
# ==========================================
class NetworkManager:
    """Manages WiFi connectivity, MQTT communication, exponential backoff and discovery payloads."""

    def __init__(
        self, client_id: str, broker: str, user: str, password: str
    ) -> None:
        self.client_id = client_id
        self.broker = broker
        self.user = user
        self.password = password
        self.client = None
        self.failed_attempts = 0
        self.reconnects = 0
        self.current_backoff = initial_backoff_secs

        self.master_avail_topic = f"homeassistant/sensor/{self.client_id}/availability"
        self.ota_cmd_topic = f"homeassistant/button/{self.client_id}_ota/set"
        self.fan_cmd_topic = f"homeassistant/switch/{self.client_id}_relay/set"
        self.fan_on_thresh_topic = f"homeassistant/number/{self.client_id}_fan_on/set"
        self.fan_off_thresh_topic = f"homeassistant/number/{self.client_id}_fan_off/set"

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
                
                if hasattr(self.client, "sock") and self.client.sock is not None:
                    self.client.sock.settimeout(5.0)

                self.publish(self.master_avail_topic, "online", retain=True)

                self.reconnects += 1
                self.failed_attempts = 0
                self.current_backoff = initial_backoff_secs
                system_status = "nominal - mainloop"
                flash_led(40)
                return True
            except (OSError, MQTTException) as err:
                print(f"MQTT connection failed: {err}")
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
            print(system_status)
            time.sleep(1)
            machine.reset()

        return False

    def publish(self, topic: str, payload_data, retain: bool = True) -> bool:
        """Publish data payload to specified MQTT topic."""
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
            flash_led(15)
            return True
        except (OSError, MQTTException) as err:
            print(f"MQTT publish error to {topic}: {err}")
            self.client = None
            return False

    def check_messages(self) -> None:
        """Check for incoming MQTT messages."""
        if self.client:
            try:
                self.client.check_msg()
            except (OSError, MQTTException) as err:
                print(f"MQTT check_msg error: {err}")
                self.client = None

    async def send_discovery(self, sensors_list: list) -> bool:
        """Send Home Assistant MQTT discovery configuration."""
        try:
            system_status = "nominal - sending discovery"
            device_info = {
                "identifiers": [self.client_id, device_serial],
                "name": "Victron Enclosure Controller",
                "model": "Raspberry Pi Pico 2 W",
                "sw_version": app_version,
                "serial_number": device_serial,
            }

            # 1. Fan Relay Switch Discovery
            relay_config_topic = f"homeassistant/switch/{self.client_id}_relay/config"
            relay_payload = {
                "name": "Cooling Fans",
                "unique_id": f"{self.client_id}_relay",
                "command_topic": self.fan_cmd_topic,
                "state_topic": f"homeassistant/switch/{self.client_id}_relay/state",
                "availability_topic": self.master_avail_topic,
                "device": device_info,
            }
            self.publish(relay_config_topic, relay_payload)
            await asyncio.sleep(0.1)

            # 2. Hysteresis Threshold Sliders
            for thresh_key, thresh_name, val in [("fan_on", "Fan ON Threshold", fan_on_threshold_c), ("fan_off", "Fan OFF Threshold", fan_off_threshold_c)]:
                t_config = f"homeassistant/number/{self.client_id}_{thresh_key}/config"
                t_payload = {
                    "name": thresh_name,
                    "unique_id": f"{self.client_id}_{thresh_key}",
                    "command_topic": f"homeassistant/number/{self.client_id}_{thresh_key}/set",
                    "state_topic": f"homeassistant/number/{self.client_id}_{thresh_key}/state",
                    "availability_topic": self.master_avail_topic,
                    "unit_of_measurement": "°C",
                    "min": 10.0,
                    "max": 45.0,
                    "step": 0.5,
                    "device": device_info,
                }
                self.publish(t_config, t_payload)
                self.publish(f"homeassistant/number/{self.client_id}_{thresh_key}/state", val)
                await asyncio.sleep(0.1)

            # 3. OTA Update Button Discovery
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

            # 4. Temperature Sensors Discovery
            for s in sensors_list:
                topic = f"homeassistant/sensor/{s['sensor_id']}_T/config"
                payload = {
                    "name": f"{s['sensor_name']} Temp", 
                    "unique_id": f"{s['sensor_id']}_T", 
                    "state_topic": f"homeassistant/sensor/{s['sensor_id']}/state",
                    "availability": [
                        {"topic": self.master_avail_topic},
                        {"topic": f"homeassistant/sensor/{s['sensor_id']}/availability"}
                    ],
                    "availability_mode": "all",
                    "unit_of_measurement": "°C", "device_class": "temperature",
                    "state_class": "measurement", 
                    "value_template": "{{ value_json.temperature }}",
                    "device": device_info
                }
                self.publish(topic, payload)
                await asyncio.sleep(0.1)
            
            # 5. Diagnostic Sensors Discovery
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
                if d_class: payload["device_class"] = d_class
                if unit: payload["unit_of_measurement"] = unit
                if s_class: payload["state_class"] = s_class 
                self.publish(topic, payload)
                await asyncio.sleep(0.1)
            return True
        except (OSError, ValueError) as err:
            print(f"Discovery error: {err}")
            return False


# ==========================================
# 4. THE MAIN LOOP & ASYNC TASKS
# ==========================================
async def main() -> None:
    global system_status, last_published_status, force_sensor_publish, watchdog, last_crash_log, fan_on_threshold_c, fan_off_threshold_c

    watchdog = machine.WDT(timeout=watchdog_timeout_ms)

    sensors_manager = SensorManager(data_pin_num=ONE_WIRE_PIN, sensor_list=sensor_names)
    network_controller = NetworkManager(
        client_id, secrets.mqttBroker, secrets.mqttUser, secrets.mqttPassword
    )

    # MQTT message callback handler
    def sub_cb(topic: bytes, msg: bytes) -> None:
        global system_status, fan_on_threshold_c, fan_off_threshold_c
        topic_str = topic.decode("utf-8")
        msg_str = msg.decode("utf-8")

        if topic_str == network_controller.fan_cmd_topic:
            if msg_str.upper() == "ON":
                set_fans(True)
            elif msg_str.upper() == "OFF":
                set_fans(False)
            network_controller.publish(f"homeassistant/switch/{client_id}_relay/state", "ON" if fans_currently_active else "OFF")

        elif topic_str == network_controller.fan_on_thresh_topic:
            try:
                fan_on_threshold_c = float(msg_str)
                network_controller.publish(topic_str.replace("/set", "/state"), fan_on_threshold_c)
            except ValueError:
                pass

        elif topic_str == network_controller.fan_off_thresh_topic:
            try:
                fan_off_threshold_c = float(msg_str)
                network_controller.publish(topic_str.replace("/set", "/state"), fan_off_threshold_c)
            except ValueError:
                pass

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

        if network_controller.client is not None and (now_secs - last_loop_success_secs > 120):
            system_status = "error - main loop deadlock, forcing reset"
            print(system_status)
            time.sleep(1)
            machine.reset()

        if not await network_controller.maintain_connection():
            discovery_sent = False
            last_loop_success_secs = now_secs
            await asyncio.sleep(2)
            continue

        if network_controller.client and not discovery_sent:
            network_controller.client.set_callback(sub_cb)
            network_controller.client.subscribe(network_controller.ota_cmd_topic.encode("utf-8"))
            network_controller.client.subscribe(network_controller.fan_cmd_topic.encode("utf-8"))
            network_controller.client.subscribe(network_controller.fan_on_thresh_topic.encode("utf-8"))
            network_controller.client.subscribe(network_controller.fan_off_thresh_topic.encode("utf-8"))

            if await network_controller.send_discovery(sensors_manager.sensors):
                discovery_sent = True
                last_heartbeat_time_secs = now_secs - heartbeat_interval_secs
                last_read_time_secs = now_secs - sensor_interval_secs

        network_controller.check_messages()

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

        # Poll Sensors & Fans
        if now_secs - last_read_time_secs >= sensor_interval_secs:
            system_status = "nominal - sensor conversion"
            bus_active = sensors_manager.start_conversion()
            if not bus_active:
                for s in sensors_manager.sensors:
                    if s['last_avail'] != 'offline':
                        network_controller.publish(f"homeassistant/sensor/{s['sensor_id']}/availability", "offline")
                        s['last_avail'] = 'offline'
                last_read_time_secs = now_secs 
            else:
                await asyncio.sleep_ms(750)
                
                ready_payloads = sensors_manager.read_and_evaluate(force_publish=force_sensor_publish)
                force_sensor_publish = False 
                
                for item in ready_payloads:
                    network_controller.publish(item['topic'], item['payload'])
                    
                last_read_time_secs = time.time()
                system_status = "nominal - mainloop"

        last_loop_success_secs = now_secs
        await asyncio.sleep(0.1)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        try:
            with open("/fatal_crash.log", "w") as crash_file:
                sys.print_exception(exc, crash_file)
        except OSError:
            pass
        time.sleep(1)
        machine.reset()