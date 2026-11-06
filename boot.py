import gc
import os
import sys
import time
import machine
import asyncio

boot_version = "1.6.6"

STARTUP_GRACE_PERIOD_SECS = 5


def init_environment() -> None:
    """Configure baseline garbage collection and system thresholds."""
    gc.enable()
    gc.threshold(gc.mem_free() // 4)
    print(f"boot.py v{boot_version}: Environment initialised")


def startup_grace_period() -> bool:
    """Provide a brief window on boot to actively catch KeyboardInterrupt (Ctrl+C) and enter REPL."""
    print(f"boot.py v{boot_version}: Boot... graceperiod {STARTUP_GRACE_PERIOD_SECS}s sleeping awaiting console interrupt")
    start_time = time.time()
    try:
        while time.time() - start_time < STARTUP_GRACE_PERIOD_SECS:
            time.sleep_ms(100)
        print(f"boot.py v{boot_version}: Boot ... graceperiod complete carrying on with boot")
        return True
    except KeyboardInterrupt:
        print("\n*** BOOT ABORTED BY USER *** Entering interactive REPL.")
        return False


def perform_rollback(reason: str) -> None:
    """Roll back main.py to main_backup.py upon startup failure."""
    print(f"boot.py v{boot_version}: Initiating rollback due to: {reason}")
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
                
            print(f"boot.py v{boot_version}: Rollback complete. Resetting machine...")
            machine.reset()
        else:
            print(f"boot.py v{boot_version}: Rollback failed - main_backup.py missing")
    except Exception as rb_err:
        print(f"boot.py v{boot_version}: Critical failure during rollback: {rb_err}")


def boot_sequence() -> None:
    """Execute boot sequence with graceful interrupt protection and OTA monitoring."""
    if not startup_grace_period():
        return

    ota_pending = False
    try:
        with open("/ota_pending.flag", "r") as flag_file:
            _ = flag_file.read()
        ota_pending = True
        print(f"boot.py v{boot_version}: OTA pending flag detected. Monitoring boot...")
    except OSError:
        pass

    init_environment()

    try:
        import main
        asyncio.run(main.main())
    except Exception as err:
        print(f"boot.py v{boot_version}: Boot failed with error: {err}")
        if ota_pending:
            perform_rollback(str(err))
        else:
            try:
                with open("/fatal_crash.log", "w") as crash_file:
                    sys.print_exception(err, crash_file)
            except OSError:
                pass
            time.sleep(2)
            machine.reset()


if __name__ == "__main__":
    print(f"boot.py v{boot_version}: Starting boot sequence")
    boot_sequence()