import click
import requests

from .board_comms_utils import get_board_id
from .flash_utils import FIRMWARE_INDEX, download_and_flash_firmware


@click.group()
def cli():
    pass


@cli.command()
def flash():
    """
    Flash an ESP32-S3 with the latest firmware.
    """
    board_id_map = {
        0x01: "MotorGo Mini",
        0x02: "MotorGo Core",
        0x03: "MotorGo Plink",
        0x04: "MotorGo Axis",
    }

    click.secho("Configuring MotorGo for PiHat mode", fg="green")

    click.secho("Step 1: Identifying MotorGo board", fg="blue")
    # Retrieve the firmware index
    firmware_index = requests.get(FIRMWARE_INDEX).json()
    # Retrieve get_board_id_firmware
    get_board_id_name = firmware_index["get_board_id_firmware"]["name"]
    get_board_id_url = firmware_index["get_board_id_firmware"]["url"]

    # Download and flash the get_board_id_firmware
    download_and_flash_firmware(get_board_id_name, get_board_id_url)

    board_id = get_board_id()
    if board_id not in board_id_map:
        click.secho("Error: Unknown board ID", fg="red")
        return

    click.secho(f"Detected board: {board_id_map[board_id]}", fg="green")

    click.secho("Step 2: Downloading and flashing latest firmware", fg="blue")
    # Retrieve url for the latest firmware
    try:
        firmware_name = firmware_index["motorgo_python_firmware"][f"0x{board_id:02x}"][
            "latest"
        ]["name"]
        firmware_url = firmware_index["motorgo_python_firmware"][f"0x{board_id:02x}"][
            "latest"
        ]["url"]
    except KeyError:
        click.secho("Error: Firmware not available for this board", fg="red")
        return

    download_and_flash_firmware(firmware_name, firmware_url)

    click.secho("MotorGo successfully configured for PiHat mode!", fg="green")


if __name__ == "__main__":
    cli()
