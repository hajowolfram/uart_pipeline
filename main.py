import subprocess
import sys
import os

def run_step(description, command, shell=False):
    print(f"\n[INFO] Running: {description}...")
    try:
        subprocess.run(command, check=True, shell=shell)
        print(f"[SUCCESS] {description} completed.\n")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] {description} failed with exit code {e.returncode}.")
        sys.exit(e.returncode)

if __name__ == "__main__":
    run_step("Flashing configuration", ["python3", "flash_cfg.py"])
    run_step("Running port test", ["python3", "port_test.py"])
    run_step("Compiling C project", ["make"])
    binary_path = os.path.join("bin", "parser")
    if not os.path.exists(binary_path):
        print(f"[ERROR] Binary not found at {binary_path}")
        sys.exit(1)

    run_step("Running compiled binary", [binary_path])
