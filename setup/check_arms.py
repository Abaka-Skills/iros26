"""Connect to each arm and read its state. Sends no motion commands.

Run from ALOHA/:  uv run python setup/check_arms.py
"""
import trossen_arm

ARMS = ["192.168.10.2", "192.168.10.3", "192.168.10.4", "192.168.10.5"]

for ip in ARMS:
    driver = trossen_arm.TrossenArmDriver()
    try:
        driver.configure(
            trossen_arm.Model.wxai_v0,
            trossen_arm.StandardEndEffector.wxai_v0_base,
            ip,
            False,  # don't clear errors; just report them
            timeout=5.0,
        )
        print(f"{ip}  controller {driver.get_controller_version()}")
        print(f"    modes     {[m.name for m in driver.get_modes()]}")
        print(f"    positions {[round(p, 3) for p in driver.get_all_positions()]}")
    except Exception as e:
        print(f"{ip}  FAILED: {e}")
    finally:
        driver.cleanup()
