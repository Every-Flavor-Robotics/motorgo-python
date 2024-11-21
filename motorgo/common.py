# Collection of common tools used across all motor controller classes

import struct
import time


class InitOutputStruct:
    """A class representing the data that is sent to the motor controller to initialize the motor controller

    The struct contains:
    - valid (bool)
    - Target Frequency (float)
    """

    SIZE = 5

    def __init__(self, target_frequency: float):
        self.target_frequency = target_frequency

    def get_packed_struct(self, output_size=None) -> bytes:
        """
        Pack the structure data into bytes for transmission.
        """
        packed = struct.pack(
            "<?f",
            True,
            self.target_frequency,
        )

        assert len(packed) == self.SIZE

        if output_size is not None and output_size > len(packed):
            return packed + b"\x00" * (output_size - len(packed))

        return packed


class InitInputStruct:
    """A class representing the data that is received from the motor controller to initialize the motor controller

    The struct contains:
    - valid (bool)
    - board id (int)
    - firmware version (int)
    """

    SIZE = 3

    def __init__(self, data: bytes):
        self.data = data

        self.valid = False
        self.board_id = None
        self.firmware_version = None

    def decode(self):
        data = bytearray(self.data)[: self.SIZE]

        # Unpack two ints from the data
        self.valid, board_id, firmware_version = struct.unpack("<?BB", data)

        if self.valid:
            self.board_id = board_id
            self.firmware_version = firmware_version

    def __str__(self):
        return f"Board ID: {self.board_id}, Firmware Version: {self.firmware_version}"
