import subprocess
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent


def check_and_install(package, module_name=None):
    """
    Check for python import and install via pip if missing.
    package: pip package name/spec
    module_name: python import name (optional)
    """
    mod = module_name or str(package).split()[0]
    try:
        if module_available(mod):
            print(f"{mod} is already installed.")
            return True
    except Exception:
        pass
    return pip_install(package)


def module_available(module_name):
    try:
        __import__(module_name)
        return True
    except ImportError:
        return False


def pip_install(package):
    commands = [
        f"pip3 install --user {package}",
        f"pip3 install --break-system-packages {package}",
    ]
    for command in commands:
        try:
            subprocess.run(command, shell=True, check=True)
            print(f"Successfully installed {package} via pip.")
            return True
        except subprocess.CalledProcessError:
            continue
    print(f"Failed to install {package} via pip.")
    return False


def ensure_python_package(module_name, pip_package):
    if module_available(module_name):
        print(f"{module_name} is already installed.")
        return True
    return pip_install(pip_package)


def ensure_import_only(module_name):
    if module_available(module_name):
        print(f"{module_name} is available.")
        return True
    print(f"Missing required module: {module_name}")
    return False


def apt_install(package):
    install_command = f"sudo apt-get install -y {package}"
    try:
        subprocess.run(install_command, shell=True, check=True)
        print(f"Successfully installed {package} via apt-get.")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Failed to install {package} via apt-get: {e}")
        return False


def custom_install(command):
    try:
        subprocess.run(command, shell=True, check=True)
        print(f"Successfully executed custom command: {command}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Failed to execute custom command: {command}: {e}")
        return False


def get_raspberry_pi_version():
    print("Getting Raspberry Pi version...")
    try:
        result = subprocess.run(
            ["cat", "/sys/firmware/devicetree/base/model"],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode == 0:
            model = result.stdout.strip()
            if "Raspberry Pi 5" in model:
                print("Detected Raspberry Pi 5")
                return 3
            elif "Raspberry Pi 3" in model:
                print("Detected Raspberry Pi 3")
                return 2
            else:
                print(f"Detected Raspberry Pi {model}")
                return 1
        else:
            print("Failed to get Raspberry Pi model information.")
            return 0
    except Exception as e:
        print(f"Error getting Raspberry Pi version: {e}")
        return 0


def update_config_file(file_path, command, value):
    new_content = []
    command_found = False
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    for line in lines:
        stripped_line = line.strip()
        if stripped_line.startswith(command) or stripped_line.startswith(f"#{command}"):
            command_found = True
            new_content.append(f"{command}={value}\n")
        else:
            new_content.append(line)
    if not command_found:
        new_content.append(f"\n{command}={value}\n")
    with open(file_path, "w", encoding="utf-8") as f:
        f.writelines(new_content)
    print(f"Updated {file_path} with '{command}={value}'")


def config_camera_to_config_txt(file_path, command, value=None):
    new_content = []
    command_found = False
    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    for line in lines:
        stripped_line = line.strip()
        if "ov5647" in stripped_line or "imx219" in stripped_line:
            continue
        if stripped_line.startswith(f"dtoverlay={command}") or stripped_line.startswith(
            f"#dtoverlay={command}"
        ):
            command_found = True
            if value:
                new_content.append(f"dtoverlay={command},{value}\n")
            else:
                new_content.append(f"dtoverlay={command}\n")
        else:
            new_content.append(line)
    if not command_found:
        if value:
            new_content.append(f"\ndtoverlay={command},{value}\n")
        else:
            new_content.append(f"\ndtoverlay={command}\n")
    with open(file_path, "w", encoding="utf-8") as f:
        f.writelines(new_content)
    value_str = f",{value}" if value else ""
    print(f"Updated {file_path} with 'dtoverlay={command}{value_str}'")


def backup_file(file_path):
    config_path = file_path
    backup_path = config_path + ".bak"
    print("Backing up ", backup_path)
    try:
        with open(config_path, "rb") as src_file:
            with open(backup_path, "wb") as dst_file:
                dst_file.write(src_file.read())
        print(f"Backup of {config_path} created at {backup_path}")
    except Exception as e:
        print(f"Error backing up {config_path}: {e}")


def config_file():
    pi_version = get_raspberry_pi_version()
    file_path = "/boot/firmware/config.txt"
    backup_file(file_path)
    update_config_file(file_path, "dtparam=spi", "on")
    update_config_file(file_path, "camera_auto_detect", "0")
    while True:
        camera_model = (
            input("\nEnter the camera model (e.g., ov5647 or imx219): ").strip().lower()
        )
        if camera_model not in ["ov5647", "imx219"]:
            print("Invalid input. Please enter either ov5647 or imx219.")
        else:
            break
    if pi_version == 3:
        print("Setting up for Raspberry Pi 5")
        while True:
            camera_port = (
                input(
                    "You have a Raspberry Pi 5. Which camera port is the camera connected to? cam0 or cam1: "
                )
                .strip()
                .lower()
            )
            if camera_port not in ["cam0", "cam1"]:
                print("Invalid input. Please enter either cam0 or cam1.")
            else:
                break
        config_camera_to_config_txt(file_path, camera_model, camera_port)
    elif pi_version == 2:
        print("Setting up for Raspberry Pi 3")
        update_config_file(file_path, "dtparam=audio", "off")
        config_camera_to_config_txt(file_path, camera_model)
    else:
        config_camera_to_config_txt(file_path, camera_model)


def main():
    pi_version = get_raspberry_pi_version()
    install_status = {
        "apt core libs": False,
        "apt vision libs": False,
        "apt lgpio (pi5 only)": True,
        "python gpiozero": False,
        "python numpy": False,
        "python cv2": False,
        "python picamera2": False,
        "python libcamera": False,
        "python rpi_hardware_pwm": False,
        "python lgpio (pi5 only)": True,
        "custom pi-hardware-pwm overlay": False,
        "custom rpi-ws281x": False,
    }

    print("Updating package lists...")
    subprocess.run("sudo apt-get update", shell=True, check=True)

    print("Installing APT packages...")
    install_status["apt core libs"] = apt_install(
        "python3-dev python3-pyqt5 python3-pigpio python3-gpiozero"
    )
    install_status["apt vision libs"] = apt_install(
        "python3-numpy python3-opencv python3-picamera2 python3-libcamera"
    )
    if pi_version == 3:
        install_status["apt lgpio (pi5 only)"] = apt_install("python3-lgpio")

    print("Checking Python imports used by robot runtime...")
    install_status["python gpiozero"] = ensure_import_only("gpiozero")
    install_status["python numpy"] = ensure_python_package("numpy", "numpy")
    install_status["python cv2"] = ensure_python_package("cv2", "opencv-python")
    install_status["python picamera2"] = ensure_import_only("picamera2")
    install_status["python libcamera"] = ensure_import_only("libcamera")
    install_status["python rpi_hardware_pwm"] = ensure_python_package(
        "rpi_hardware_pwm", "rpi-hardware-pwm"
    )
    if pi_version == 3:
        install_status["python lgpio (pi5 only)"] = ensure_import_only("lgpio")

    print("Running custom installations...")
    install_status["custom pi-hardware-pwm overlay"] = custom_install(
        f"cd {SCRIPT_DIR / 'Libs/pi-hardware-pwm'} && sh ./cleanup_pwm_overlay.sh && sh ./setup_pwm_overlay.sh"
    )
    install_status["custom rpi-ws281x"] = custom_install(
        f"cd {SCRIPT_DIR / 'Libs/rpi-ws281x-python/library'} && sudo python3 setup.py install"
    )

    if all(install_status.values()):
        print("\nAll libraries have been installed successfully.")
        config_file()
        print("Please reboot your Raspberry Pi to complete the installation.")
    else:
        missing_libraries = [
            lib for lib, status in install_status.items() if not status
        ]
        print(
            f"\nSome libraries have not been installed yet: {', '.join(missing_libraries)}. Please run the script again."
        )


if __name__ == "__main__":
    main()
