import os
import machine

# Check if an OTA update was pending
try:
    with open("ota_pending.flag", "r") as f:
        pending = f.read()
    
    # If a flag exists and main.py fails or has syntax errors, roll back
    print("OTA pending flag detected. Monitoring boot...")
except OSError:
    pass

try:
    # Attempt to import or execute main
    import main
except Exception as e:
    print(f"Boot failed with error: {e}")
    try:
        if "main_backup.py" in os.listdir():
            print("Rolling back to main_backup.py...")
            try:
                os.remove("main.py")
            except OSError:
                pass
            os.rename("main_backup.py", "main.py")
            os.remove("ota_pending.flag")
            print("Rollback complete. Rebooting...")
            machine.reset()
    except Exception as rb_err:
        print(f"Rollback failed: {rb_err}")