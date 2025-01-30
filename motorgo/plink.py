import atexit
import struct
import threading
import time
from collections import deque

import spidev
from gpiozero import DigitalInputDevice, DigitalOutputDevice

from .brushed_motor_channel import BrakeMode, ControlMode, MotorChannel
from .common import crc16
from .imu import IMU
from .message_parser import MessageParser
from .messages import (
    DataFromPeri,
    DataToPeri,
    InitFromPeri,
    InitToPeri,
    InvalidFromPeri,
    InvalidToPeri,
    MessageFromPeri,
    MessageToPeri,
    PIDToPeri,
)
from .version import __version__


class Plink:
    """Interface for controller MotorGo Plink in PiHat mode.

    Plink automatically handles communication with the Plink when connected as
    a PiHat. The class provides interfaces for controlling the motor channels,
    ready the encoder data, and reading the IMU sensor data.

    Attributes:
        channel1 (MotorChannel): Interface for controlling motor channel 1.
        channel2 (MotorChannel): Interface for controlling motor channel 2.
        channel3 (MotorChannel): Interface for controlling motor channel 3.
        channel4 (MotorChannel): Interface for controlling motor channel 4.
        imu (IMU): Interface for reading IMU sensor data.
    """

    SUPPORTED_BOARD_IDS = [0x03]

    def __init__(self, frequency: int = 200, timeout: float = 1.0):
        """Initializes the Plink object.

        Args:
            frequency (int, optional): Communication frequency with the Plink.
                The update frequency tends to become unstable past 200 Hz.
                Defaults to 200.
            timeout (float, optional): Communication timeout (seconds).
                If no data is received beyond this time, the Plink is reset.
                Defaults to 1.0.
        """
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # Open bus 0, device (CS) 0
        self.spi.mode = 3
        self.spi.max_speed_hz = 7_200_000  # Set SPI speed

        self.last_message_time = None

        self.channel1 = MotorChannel()
        self.channel2 = MotorChannel()
        self.channel3 = MotorChannel()
        self.channel4 = MotorChannel()

        self.frequency = frequency
        self.timeout = timeout

        self.running = False
        self.connected = False

        # Add an extra 4 bytes at the end for padding
        # (esp32 slave has a known bug requiring this)
        self.transfer_size = 76

        self.imu = IMU(self.frequency)

        self.data_ready_pin = DigitalInputDevice(25)
        self.reset_pin = DigitalOutputDevice(22, active_high=False, initial_value=False)

        self.config_message_queue = deque()

        self.power_supply_voltage = None

    def reset(self):
        """Resets the Plink device.

        This toggles the reset pin on the ESP32, causing its program to restart.
        Useful if the Plink is unresponsive or misbehaving.
        """
        self.reset_pin.blink(on_time=0.05, off_time=0.05, n=1)

    def connect(self):
        """Begins communication with the Plink.

        Initializes the communication thread, confirms the connected device
        is a MotorGo Plink, and starts periodic data transfer. The power supply
        voltage limit must be set prior to calling this.

        Raises:
            ValueError: If the voltage limit is not set before connecting.
        """
        atexit.register(self.shutdown)  # Register cleanup function

        if self.power_supply_voltage is None:
            raise ValueError("Voltage limit must be set before connecting")

        # Reset the Plink first
        self.reset()

        self.connected = self.initialize_comms()

        if self.connected:
            print("Connected to Plink")

            self.running = True
            self.thread = threading.Thread(target=self.comms_thread)
            self.thread.daemon = True
            self.thread.start()
        else:
            print("Not starting connection to protect MotorGo")

    def update_motor_states(self, response: DataFromPeri):
        """Updates the motor state and IMU data from a response message.

        Args:
            response (DataFromPeri): Message containing motor positions/velocities
                and IMU sensor readings.
        """
        self.channel1.update_position(response.channel_1_pos)
        self.channel1.update_velocity(response.channel_1_vel)

        self.channel2.update_position(response.channel_2_pos)
        self.channel2.update_velocity(response.channel_2_vel)

        self.channel3.update_position(response.channel_3_pos)
        self.channel3.update_velocity(response.channel_3_vel)

        self.channel4.update_position(response.channel_4_pos)
        self.channel4.update_velocity(response.channel_4_vel)

        self.imu.update(
            response.gyro_x,
            response.gyro_y,
            response.gyro_z,
            response.accel_x,
            response.accel_y,
            response.accel_z,
            response.mag_x,
            response.mag_y,
            response.mag_z,
        )

    def initialize_comms(self):
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
        data = InitToPeri(
            self.frequency,
            self.power_supply_voltage,
            self.channel1.motor_voltage_limit,
            self.channel2.motor_voltage_limit,
            self.channel3.motor_voltage_limit,
            self.channel4.motor_voltage_limit,
        )

        initialized = False
        while not initialized:
            response = self.transfer(data)
            if isinstance(response, InitFromPeri):
                initialized = True

        valid = True
        expected_version = crc16(__version__.encode())
        if response.version_hash != expected_version:
            print("Invalid version received")
            valid = False

        if response.board_id not in self.SUPPORTED_BOARD_IDS:
            print("Unsupported board ID received")
            valid = False

        return valid

    def transfer(self, message: MessageToPeri, timeout: float = 1.0) -> MessageFromPeri:
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
        if not self.data_ready_pin.wait_for_active(timeout=timeout):
            # Timed out waiting for data-ready pin
            return None

        raw_response = self.spi.xfer2(message.get_packed_struct(self.transfer_size))
        response = MessageParser().parse(raw_response)

        if not self.data_ready_pin.wait_for_inactive(timeout=timeout):
            # Timed out waiting for data-ready pin to go low
            return None

        if not isinstance(response, InvalidFromPeri):
            self.last_message_time = time.time()

        return response

    def update_motorgo(self):
        """Handles periodic updates to the MotorGo device.

        Prepares and sends data (either config updates or normal command data),
        then parses the response and updates local state.
        """
        self.prepare_config_update_messages()

        if self.config_message_queue:
            message = self.config_message_queue.popleft()
        else:
            message = DataToPeri(
                self.channel1, self.channel2, self.channel3, self.channel4
            )

        response = self.transfer(message)
        if isinstance(response, DataFromPeri):
            self.update_motor_states(response)
            self.last_message_time = time.time()
        elif isinstance(response, InitFromPeri):
            print("Received initialization response, re-initializing...")
            self.initialize_comms()
        elif isinstance(response, InvalidFromPeri):
            print("Invalid response received")

    def prepare_config_update_messages(self):
        """Checks each motor channel for pending config updates and queues them.

        If a channel’s PID gains were marked for update, create a PIDToPeri
        message and place it in the `config_message_queue`.
        """
        for i, channel in enumerate(
            [self.channel1, self.channel2, self.channel3, self.channel4]
        ):
            if channel._pid_update_ready:
                channel_number = i + 1
                new_params = channel._get_velocity_gain_update()
                self.config_message_queue.append(PIDToPeri(channel_number, *new_params))

    def comms_thread(self):
        """Communication thread to handle periodic data transfer.

        This continuously updates the MotorGo device at the specified frequency,
        detects timeouts, and attempts reconnection if needed.
        """
        try:
            while self.running:
                if self.connected:
                    self.update_motorgo()

                    # Check for response timeout
                    if self.last_message_time is not None and (
                        time.time() - self.last_message_time > self.timeout
                    ):
                        print("MotorGo response timeout")
                        self.connected = False
                else:
                    # Attempt to re-initialize if disconnected
                    print("Attempting to re-initialize connection")
                    self.connected = self.initialize_comms()

        except KeyboardInterrupt:
            self.shutdown()
        finally:
            self.shutdown()

    def shutdown(self):
        """Cleans up resources and performs shutdown tasks.

        Stops the communication loop, toggles reset, and closes SPI as needed.
        """
        self.running = False
        self.reset()
        print("Disconnecting from Plink ...")
        # self.spi.close()  # Uncomment if you want to close SPI on shutdown
