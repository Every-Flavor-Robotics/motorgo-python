import click
from .flash import download_and_flash_firmware


@click.group()
def cli():
    pass


@cli.command()
def flash():
    """
    Flash an ESP32-S3 with the latest firmware.
    """
    click.secho("Setting up MotorGo for PiHat Mode operation", fg="green")

    click.secho("Step 1: Identify MotorGo board", fg="blue")
    click.secho("Step 2: Downloading latest firmware", fg="blue")
    download_and_flash_firmware()

    click.secho("MotorGo successfully configured!", fg="green")


if __name__ == "__main__":
    cli()
