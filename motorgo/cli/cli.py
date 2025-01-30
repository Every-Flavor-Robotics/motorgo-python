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
    download_and_flash_firmware()


if __name__ == "__main__":
    cli()
