import tempfile
import time
import zipfile
from pathlib import Path

import click
import esptool
import requests
from gpiozero import DigitalOutputDevice

BOOT_PIN = 5  # BCM pin connected to ESP BOOT
RESET_PIN = 22  # BCM pin connected to ESP RESET

# Set up each pin as an active-high digital output, initially HIGH
# so that both lines are released (high) by default.
boot_pin = DigitalOutputDevice(BOOT_PIN, active_high=True, initial_value=True)
reset_pin = DigitalOutputDevice(RESET_PIN, active_high=True, initial_value=True)

FIRMWARE_INDEX = "https://raw.githubusercontent.com/Every-Flavor-Robotics/motorgo-python/refs/heads/main/firmware_releases/firmware_index.json"


def enter_program_mode():
    """
    Drive the ESP32-S3 into its serial bootloader mode:
      1. Pull BOOT low.
      2. Toggle RESET low momentarily.
      3. Release RESET (go high).
      4. Wait briefly.
      5. Release BOOT (go high).
    """
    print("Pulling BOOT low...")
    boot_pin.off()
    time.sleep(0.1)

    print("Resetting ESP (pulling RESET low)...")
    reset_pin.off()
    time.sleep(0.1)

    print("Releasing RESET (going high)...")
    reset_pin.on()
    time.sleep(0.5)

    print("Releasing BOOT (going high)...")
    boot_pin.on()


def reset():
    """ """

    print("Resetting ESP (pulling RESET low)...")
    reset_pin.off()
    time.sleep(0.1)

    print("Releasing RESET (going high)...")
    reset_pin.on()
    time.sleep(0.5)


def flash_firmware(
    firmware_dir,
    port="/dev/ttyS0",
    baud="460800",
):
    """
    Flash an ESP32-S3 with multiple binary files (bootloader, partition table,
    boot_app0, and main firmware) using esptool, similar to PlatformIO's command:

      esptool.py
          --chip esp32s3
          --port /dev/ttyACM1
          --baud 460800
          --before default_reset
          --after hard_reset
          write_flash -z --flash_mode dio
          --flash_freq 80m
          --flash_size detect
          0x0000 bootloader.bin
          0x8000 partitions.bin
          0xe000 boot_app0.bin
          0x10000 firmware.bin

    :param port: Serial port to use (e.g. "/dev/ttyACM1", "/dev/ttyUSB0", etc.)
    :param baud: Baud rate for flashing
    :param bootloader: Path to the bootloader.bin
    :param partitions: Path to the partitions.bin
    :param boot_app0: Path to the boot_app0.bin
    :param firmware: Path to the main firmware .bin
    """

    # Paths to the binary files
    # Check that fimrware_dir exists
    firmware_dir = Path(firmware_dir)
    assert firmware_dir.exists(), f"Directory {firmware_dir} does not exist."

    bootloader = firmware_dir / "bootloader.bin"
    partitions = firmware_dir / "partitions.bin"
    boot_app0 = firmware_dir / "boot_app0.bin"
    firmware = firmware_dir / "firmware.bin"

    args = [
        "--chip",
        "esp32s3",
        "--port",
        port,
        "--baud",
        baud,
        "--before",
        "default_reset",
        "--after",
        "hard_reset",
        "write_flash",
        "-z",
        "--flash_mode",
        "dio",
        "--flash_freq",
        "80m",
        "--flash_size",
        "detect",
        # Offsets + file paths, as strings
        "0x0000",
        str(bootloader),
        "0x8000",
        str(partitions),
        "0xe000",
        str(boot_app0),
        "0x10000",
        str(firmware),
    ]
    esptool.main(args)


def download_firmware(firmware_name, firmware_url, download_dir=None):
    """Download the firmware from the server"""

    if download_dir is not None:
        # Check that the directory exists
        assert Path(download_dir).exists(), f"Directory {download_dir} does not exist."
    else:
        # Create temp directory to download the firmware
        download_dir = tempfile.mkdtemp()
        print(f"Downloading firmware to {download_dir}")

    # Download the firmware
    firmware_path = Path(download_dir) / "firmware.zip"
    with requests.get(firmware_url, stream=True) as response:
        response.raise_for_status()
        with open(firmware_path, "wb") as file:
            for chunk in response.iter_content(chunk_size=8192):
                file.write(chunk)

    # Unzip the firmware
    with zipfile.ZipFile(firmware_path, "r") as zip_ref:
        zip_ref.extractall(download_dir)

    # Confirm that the firmware directory exists
    firmware_dir = Path(download_dir) / firmware_name
    assert firmware_dir.exists(), f"Directory {firmware_dir} does not exist."

    # Return path to the firmware directory
    return firmware_dir


def download_and_flash_firmware(firmware_name, firmware_url):
    """Download the firmware and flash it to the ESP32-S3"""

    # Create a temp directory to download the firmware
    with tempfile.TemporaryDirectory() as download_dir:

        # Download the firmware
        firmware_dir = download_firmware(firmware_name, firmware_url, download_dir)

        enter_program_mode()

        # Flash the firmware
        flash_firmware(firmware_dir)

        reset()


if __name__ == "__main__":
    download_and_flash_firmware()
