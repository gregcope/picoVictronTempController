# boot.py for picoVictronTempController
import gc
import os
import sys
import machine
import time
import asyncio

boot_version = "1.6.6"
print(f"BOOT: boot.py v{boot_version} starting")


def init_environment() -> None:
    """Configure baseline garbage collection and system thresholds."""
    gc.enable()
    gc.threshold(gc.mem_free() // 4)
    print("BOOT: Environment initialised")


def perform_rollback(reason: str) -> None:
    """Roll back main.py to main_backup.py upon startup failure."""
    print(f"BOOT: Initiating rollback due to: {reason}")
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
                
            print("BOOT: Rollback complete. Resetting machine...")
            machine.reset()
        else:
            print("BOOT: Rollback failed - main_backup.py missing")
    except Exception as rb_err:
        print(f"BOOT: Critical failure during rollback: {rb_err}")


def boot_sequence() -> None:
    """Execute boot sequence with OTA monitoring and main import protection."""
    ota_pending = False
    try:
        with open("/ota_pending.flag", "r") as flag_file:
            _ = flag_file.read()
        ota_pending = True
        print("BOOT: OTA pending flag detected. Monitoring boot...")
    except OSError:
        pass

    init_environment()

    try:
        import main
        asyncio.run(main.main())
    except Exception as err:
        print(f"BOOT: Boot failed with error: {err}")
        if ota_pending:
            perform_rollback(str(err))
        else:
            try:
                with open("/fatal_crash.log", "w") as crash_file:
                    sys.print_exception(err, crash_file)
            except OSError:
                pass
            # 2-second buffer allows USB interruption via Thonny if caught in an import crash-loop
            time.sleep(2)
            machine.reset()


if __name__ == "__main__":
    boot_sequence()
