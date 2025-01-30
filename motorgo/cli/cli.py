import click
import requests

from motorgo.messages import InitFromPeri, InitToPeri, InvalidFromPeri, InvalidToPeri

from .flash_utils import FIRMWARE_INDEX, download_and_flash_firmware


@click.group()
def cli():
    pass


VALID_BOARD_IDS = [0x01, 0x02, 0x03, 0x04]


def get_board_id(self):
    """Initializes the Plink communication sequence.

    This method sends initial messages to verify version and board ID,
    and ensures the Plink is ready for normal operation.

    Returns:
        bool: True if the Plink is successfully recognized and initialized,
        False otherwise.
    """
    print("Connecting to Plink...")

    # Send an invalid message first to reset SPI state
    data = InvalidToPeri()

    message = None
    while not message:
        message = self.transfer(data)

    # Prepare the actual init message
    data = InitToPeri(0, 0, 0, 0, 0, 0)

    message = None
    board_id = 0x00
    while not board_id in VALID_BOARD_IDS:
        message = self.transfer(data)

        if isinstance(message, InitFromPeri):
            board_id = message.board_id

    return board_id


@cli.command()
def flash():
    """
    Flash an ESP32-S3 with the latest firmware.
    """
    click.secho("Configuring MotorGo for PiHat mode", fg="green")

    click.secho("Step 1: Identify MotorGo board", fg="blue")
    # Retrieve the firmware index
    firmware_index = requests.get(FIRMWARE_INDEX).json()
    # Retrieve get_board_id_firmware
    get_board_id_firmware = firmware_index["get_board_id_firmware"]["url"]

    # Download and flash the get_board_id_firmware
    download_and_flash_firmware(get_board_id_firmware)

    print(get_board_id())

    # Now retri

    click.secho("Step 2: Downloading and flashing latest firmware", fg="blue")
    download_and_flash_firmware()

    click.secho("MotorGo successfully configured for PiHat mode!", fg="green")


if __name__ == "__main__":
    cli()
