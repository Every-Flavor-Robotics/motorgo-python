import atexit
import struct
import threading
import time

import spidev
from gpiozero import DigitalInputDevice, DigitalOutputDevice

from .common import InitInputStruct, InitOutputStruct
from .imu import IMU
from .motor_channel import BrakeMode, ControlMode, MotorChannel


class OutputStruct:
    BUFFER_OUT_SIZE = 25

    def __init__(
        self,
        channel1: MotorChannel,
        channel2: MotorChannel,
        channel3: MotorChannel,
        channel4: MotorChannel,
    ):
        """
        Initialize the output structure with motor channels' commands and modes.
        """
        self.valid = False

        self.channel_1_command = channel1.command
        self.channel_2_command = channel2.command
        self.channel_3_command = channel3.command
        self.channel_4_command = channel4.command

        self.channel_1_control_mode = channel1.control_mode
        self.channel_2_control_mode = channel2.control_mode
        self.channel_3_control_mode = channel3.control_mode
        self.channel_4_control_mode = channel4.control_mode

        self.channel_1_brake_mode = channel1.brake_mode
        self.channel_2_brake_mode = channel2.brake_mode
        self.channel_3_brake_mode = channel3.brake_mode
        self.channel_4_brake_mode = channel4.brake_mode

    def get_packed_struct(self, output_size=None) -> bytes:
        """
        Pack the structure data into bytes for transmission.
        """
        packed = struct.pack(
            "<?4f8B",
            True,
            self.channel_1_command,
            self.channel_2_command,
            self.channel_3_command,
            self.channel_4_command,
            self.channel_1_control_mode,
            self.channel_2_control_mode,
            self.channel_3_control_mode,
            self.channel_4_control_mode,
            self.channel_1_brake_mode,
            self.channel_2_brake_mode,
            self.channel_3_brake_mode,
            self.channel_4_brake_mode,
        )

        assert len(packed) == self.BUFFER_OUT_SIZE

        if output_size is not None and output_size > len(packed):
            return packed + b"\x00" * (output_size - len(packed))

        return packed

    def __str__(self) -> str:
        """
        Return a string representation of the output structure.
        """
        return (
            f"OutputStruct:\n"
            f"Valid: {self.valid}\n"
            f"Channel 1 Command: {self.channel_1_command}\n"
            f"Channel 2 Command: {self.channel_2_command}\n"
            f"Channel 3 Command: {self.channel_3_command}\n"
            f"Channel 4 Command: {self.channel_4_command}\n"
            f"Channel 1 Control Mode: {self.channel_1_control_mode}\n"
            f"Channel 2 Control Mode: {self.channel_2_control_mode}\n"
            f"Channel 3 Control Mode: {self.channel_3_control_mode}\n"
            f"Channel 4 Control Mode: {self.channel_4_control_mode}\n"
            f"Channel 1 Brake Mode: {self.channel_1_brake_mode}\n"
            f"Channel 2 Brake Mode: {self.channel_2_brake_mode}\n"
            f"Channel 3 Brake Mode: {self.channel_3_brake_mode}\n"
            f"Channel 4 Brake Mode: {self.channel_4_brake_mode}\n"
        )


class InputStruct:
    BUFFER_IN_SIZE = 69

    def __init__(self, data: bytes = None):
        """
        Initialize the input structure and decode data if provided.
        """

        self.valid = False
        self.channel_1_pos = 0
        self.channel_1_vel = 0
        self.channel_2_pos = 0
        self.channel_2_vel = 0
        self.channel_3_pos = 0
        self.channel_3_vel = 0
        self.channel_4_pos = 0
        self.channel_4_vel = 0

        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0

        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0

        self.mag_x = 0
        self.mag_y = 0
        self.mag_z = 0

        if data is not None:
            self.decode(data)

    def decode(self, data: list):
        """
        Decode the input data into the structure fields.
        """
        data = bytearray(data)

        unpacked_data = struct.unpack_from("<?17f", data)
        self.valid = unpacked_data[0]
        self.channel_1_pos = unpacked_data[1]
        self.channel_1_vel = unpacked_data[2]
        self.channel_2_pos = unpacked_data[3]
        self.channel_2_vel = unpacked_data[4]
        self.channel_3_pos = unpacked_data[5]
        self.channel_3_vel = unpacked_data[6]
        self.channel_4_pos = unpacked_data[7]
        self.channel_4_vel = unpacked_data[8]

        self.gyro_x = unpacked_data[9]
        self.gyro_y = unpacked_data[10]
        self.gyro_z = unpacked_data[11]
        self.accel_x = unpacked_data[12]
        self.accel_y = unpacked_data[13]
        self.accel_z = unpacked_data[14]
        self.mag_x = unpacked_data[15]
        self.mag_y = unpacked_data[16]
        self.mag_z = unpacked_data[17]

    def __str__(self) -> str:
        """
        Return a string representation of the input structure.
        """
        return (
            f"InputStruct:\n"
            f"Valid: {self.valid}\n"
            f"Channel 1 Position: {self.channel_1_pos}\n"
            f"Channel 1 Velocity: {self.channel_1_vel}\n"
            f"Channel 2 Position: {self.channel_2_pos}\n"
            f"Channel 2 Velocity: {self.channel_2_vel}\n"
            f"Channel 3 Position: {self.channel_3_pos}\n"
            f"Channel 3 Velocity: {self.channel_3_vel}\n"
            f"Channel 4 Position: {self.channel_4_pos}\n"
            f"Channel 4 Velocity: {self.channel_4_vel}\n"
        )


class Plink:

    def __init__(self, frequency: int = 200, timeout: float = 1.0):
        """
        Initialize the Plink communication object with motor channels and communication settings.
        """
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # Open bus 0, device (CS) 0
        self.spi.mode = 3
        self.spi.max_speed_hz = 7_100_000  # Set SPI speed

        self.last_message_time = None

        self.channel1 = MotorChannel()
        self.channel2 = MotorChannel()
        self.channel3 = MotorChannel()
        self.channel4 = MotorChannel()

        self.imu = IMU()

        self.frequency = frequency
        self.timeout = timeout

        self.running = False
        self.connected = False

        # Add an extra 4 bytes at the end for padding
        # esp32 slave has a bug that requires this
        self.transfer_size = 76
        # (
        #     max(OutputStruct.BUFFER_OUT_SIZE, InputStruct.BUFFER_IN_SIZE) + 4
        # )

        self.data_ready_pin = DigitalInputDevice(25)
        self.reset_pin = DigitalOutputDevice(22, active_high=False, initial_value=False)

    def reset(self):
        """
        Reset the Plink by toggling the reset pin.
        """
        self.reset_pin.blink(on_time=0.1, off_time=0.1, n=1)

    def connect(self):
        """
        Start the communication thread.
        """
        atexit.register(self.shutdown)  # Register cleanup function

        # Reset the Plink
        self.reset()

        init_response = self.transfer_init()

        # Confirm that the response is correct
        if init_response.valid:
            # TODO: Validate that the response is correct
            print("Connection established!")

            self.last_message_time = time.time()
            self.connected = True

        # Start the communication thread
        self.running = True
        self.thread = threading.Thread(target=self.comms_thread)
        self.thread.daemon = True  # Set the thread as a daemon thread
        self.thread.start()

    def calibrate_imu(self):
        """
        Calibrate the IMU by finding the mean of the gyro data.
        """
        # Wait until Plink is connected
        while not self.connected:
            print("Waiting for connection before calibration ...")
            time.sleep(0.1)
        self.imu.calibrate()

    def update_motor_states(self, response: InputStruct):
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
            self.frequency,
        )

    def transfer_init(self):
        """Prepare and send the initialization data."""
        # Prepare data from current state
        data = InitOutputStruct(self.frequency)

        # Wait for the data ready pin to be active
        self.data_ready_pin.wait_for_active()
        time.sleep(0.0001)

        # Send data and receive response (Mock response for now)
        response = InitInputStruct(
            self.spi.xfer2(data.get_packed_struct(self.transfer_size))
        )

        response.decode()

        return response

    def transfer(self):
        """
        Prepare and send data, then receive and process the response.
        """
        # Prepare data from current state
        data = OutputStruct(self.channel1, self.channel2, self.channel3, self.channel4)
        data.valid = True

        self.data_ready_pin.wait_for_active()
        time.sleep(0.0001)

        # Send data and receive response (Mock response for now)
        response = InputStruct(
            self.spi.xfer2(data.get_packed_struct(self.transfer_size))
        )

        # Update the motor states
        if response.valid:
            self.update_motor_states(response)
            self.last_message_time = time.time()

    def comms_thread(self):
        """
        Communication thread to handle periodic data transfer.
        """
        try:
            while self.running:
                start = time.time()
                # Send the message
                self.transfer()

                # Check for response timeout
                if self.last_message_time is not None:
                    if time.time() - self.last_message_time > self.timeout:
                        print("No response from SPI device")
                        self.connected = False
                        break

                # Sleep for the remainder of the cycle
                delta = time.time() - start
                sleep_time = 1.0 / self.frequency - delta
                time.sleep(sleep_time if sleep_time > 0 else 0)

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
