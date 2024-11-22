import atexit
import struct
import threading
import time

import spidev
from gpiozero import DigitalInputDevice, DigitalOutputDevice

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
)
from .motor_channel import BrakeMode, ControlMode, MotorChannel


class Plink:

    def __init__(self, frequency: int = 200, timeout: float = 1.0):
        """
        Initialize the Plink communication object with motor channels and communication settings.
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
        # esp32 slave has a bug that requires this
        self.transfer_size = 76

        self.imu = IMU(self.frequency)

        self.data_ready_pin = DigitalInputDevice(25)
        self.reset_pin = DigitalOutputDevice(22, active_high=False, initial_value=False)

    def reset(self):
        """
        Reset the Plink by toggling the reset pin.
        """
        self.reset_pin.blink(on_time=0.05, off_time=0.05, n=1)

    def connect(self):
        """
        Start the communication thread.
        """
        atexit.register(self.shutdown)  # Register cleanup function

        # Reset the Plink
        self.reset()

        self.initialize_comms()

        # Start the communication thread
        self.running = True
        self.thread = threading.Thread(target=self.comms_thread)
        self.thread.daemon = True  # Set the thread as a daemon thread
        self.thread.start()

    def update_motor_states(self, response: DataFromPeri):
        """
        Update the motor states based on the input structure data.

        Args:
            response (InputStruct): The input structure containing the motor states.
        """
        # Update the motor states
        self.channel1.update_position(response.channel_1_pos)
        self.channel1.update_velocity(response.channel_1_vel)

        self.channel2.update_position(response.channel_2_pos)
        self.channel2.update_velocity(response.channel_2_vel)

        self.channel3.update_position(response.channel_3_pos)
        self.channel3.update_velocity(response.channel_3_vel)

        self.channel4.update_position(response.channel_4_pos)
        self.channel4.update_velocity(response.channel_4_vel)

        # Update the IMU data
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
        """Prepare and send the initialization data."""

        # First, send an invalid message to reset the SPI state
        data = InvalidToPeri()

        self.transfer(data)

        # Prepare data actual initialization data
        data = InitToPeri(self.frequency)

        initialized = False

        while not initialized:
            response = self.transfer(data)

            # Check that the response is of the correct type
            if isinstance(response, InitFromPeri):
                print("Received initialization response")
                initialized = True

        # TODO: Confirm that the response is correct

        return response

    def transfer(self, message: MessageToPeri) -> MessageFromPeri:
        """
        Prepare and send data, then receive and process the response.
        """
        # Prepare data from current state
        # data = DataToPeri(self.channel1, self.channel2, self.channel3, self.channel4)
        # data.valid = True

        self.data_ready_pin.wait_for_active()

        # Send data and receive response (Mock response for now)
        response = MessageParser().parse(
            self.spi.xfer2(message.get_packed_struct(self.transfer_size))
        )

        # Update the motor states
        if isinstance(response, DataFromPeri):
            self.update_motor_states(response)
            self.last_message_time = time.time()

        # wait for data ready pin to go low
        self.data_ready_pin.wait_for_inactive()

        return response

    def update_motorgo(
        self,
    ):
        """
        Prepare and send data, then receive and process the response.
        """
        # Prepare data from current state
        data = DataToPeri(self.channel1, self.channel2, self.channel3, self.channel4)

        response = self.transfer(data)

        # Update the motor states
        if isinstance(response, DataFromPeri):
            # self.update_motor_states(response)
            self.last_message_time = time.time()

        elif isinstance(response, InitFromPeri):
            print("Received initialization response, re-doing initialization")
            self.initialize_comms()

        elif isinstance(response, InvalidFromPeri):
            print("Invalid response received")

    def comms_thread(self):
        """
        Communication thread to handle periodic data transfer.
        """
        try:
            while self.running:
                self.update_motorgo()

                # Check for response timeout
                if self.last_message_time is not None:
                    if time.time() - self.last_message_time > self.timeout:
                        print("No response from SPI device")
                        self.connected = False
                        break

        except KeyboardInterrupt:
            # Handle keyboard interrupt (Ctrl+C)
            self.shutdown()

        finally:
            # Ensure cleanup code runs when the thread is stopped
            self.shutdown()

    def shutdown(self):
        """
        Clean up resources and perform shutdown tasks.
        """
        self.running = False

        self.reset()

        print("Disconnecting from Plink ...")
        # Close the SPI connection
        # self.spi.close()
