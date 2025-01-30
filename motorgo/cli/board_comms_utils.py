import spidev
from gpiozero import DigitalInputDevice

from motorgo.message_parser import MessageParser
from motorgo.messages import (
    InitFromPeri,
    InitToPeri,
    InvalidFromPeri,
    InvalidToPeri,
    MessageFromPeri,
    MessageToPeri,
)

VALID_BOARD_IDS = [0x01, 0x02, 0x03, 0x04]


def transfer(message: MessageToPeri, timeout: float = 1.0) -> MessageFromPeri:
    """Sends and receives a single message via SPI.

    Waits for the data-ready pin, transfers the given message, and parses
    the response.

    Args:
        message (MessageToPeri): The message object to send to the Plink.
        timeout (float, optional): Maximum wait time for the data-ready pin
            to become active/inactive. Defaults to 1.0.

    Returns:
        MessageFromPeri: Parsed response message, or None if transfer fails.
    """
    data_ready_pin = DigitalInputDevice(25)

    spi = spidev.SpiDev()
    spi.open(0, 0)  # Open bus 0, device (CS) 0
    spi.mode = 3
    spi.max_speed_hz = 7_200_000  # Set SPI speed

    transfer_size = 76

    if not data_ready_pin.wait_for_active(timeout=timeout):
        # Timed out waiting for data-ready pin
        return None

    raw_response = spi.xfer2(message.get_packed_struct(transfer_size))
    response = MessageParser().parse(raw_response)

    if not data_ready_pin.wait_for_inactive(timeout=timeout):
        # Timed out waiting for data-ready pin to go low
        return None

    return response


def get_board_id():
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
        message = transfer(data)

    # Prepare the actual init message
    data = InitToPeri(0, 0, 0, 0, 0, 0)

    message = None
    board_id = 0x00
    while not board_id in VALID_BOARD_IDS:
        message = transfer(data)

        if isinstance(message, InitFromPeri):
            board_id = message.board_id

    return board_id
