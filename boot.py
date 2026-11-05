# boot.py for picoVictronTempController
import gc
import os
import sys
import machine
import time
import asyncio

try:
    import logging
except ImportError:
    import ulogging as logging

# System Logging Configuration
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("boot")

boot_version = "1.6.1"


def init_environment() -> None:
    """Configure baseline garbage collection and system thresholds."""
    gc.enable()
    gc.threshold(gc.mem_free() // 4)
    logger.info("boot.py v%s: Environment initialised", boot_version)


def perform_rollback(reason: str) -> None:
    """Roll back main.py to main_backup.py upon startup failure."""
    logger.warning("boot.py v%s: Initiating rollback due to: %s", boot_version, reason)
    try:
        files = os.listdir()
        if "main_backup.py" in files or "/main_backup.py" in files:
            try:
                os.remove("/main.py")
            except OSError:
                pass
            os.rename("/main_backup.py", "/main.py")
            
            try:
                os.remove("/ota_pending.flag")
            except OSError:
                pass
                
            with open("/ugit_log.txt", "w") as log_file:
                log_file.write(f"OTA Interrupted - Rolled Back ({reason})")
                
            logger.info("boot.py v%s: Rollback complete. Resetting machine...", boot_version)
            machine.reset()
        else:
            logger.error("boot.py v%s: Rollback failed - main_backup.py missing", boot_version)
    except Exception as rb_err:
        logger.error("boot.py v%s: Critical failure during rollback: %s", boot_version, rb_err)


def boot_sequence() -> None:
    """Execute boot sequence with OTA monitoring and main import protection."""
    ota_pending = False
    try:
        with open("/ota_pending.flag", "r") as flag_file:
            _ = flag_file.read()
        ota_pending = True
        logger.info("boot.py v%s: OTA pending flag detected. Monitoring boot...", boot_version)
    except OSError:
        pass

    init_environment()

    try:
        # Import main module and explicitly start the asynchronous main event loop[cite: 2, 5]
        import main
        asyncio.run(main.main())
    except Exception as err:
        logger.error("boot.py v%s: Boot failed with error: %s", boot_version, err)
        if ota_pending:
            perform_rollback(str(err))
        else:
            # If no OTA was pending but main crashed, save crash log[cite: 2, 5]
            try:
                with open("/fatal_crash.log", "w") as crash_file:
                    sys.print_exception(err, crash_file)
            except OSError:
                pass
            # 2-second buffer allows USB interruption via Thonny if caught in an import crash-loop
            time.sleep(2)
            machine.reset()


if __name__ == "__main__":
    logger.info("boot.py v%s: Starting boot sequence", boot_version)
    boot_sequence()