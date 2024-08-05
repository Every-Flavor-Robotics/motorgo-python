# import spidev
import time
import struct
import threading
import atexit

# Enums to match C++ definitions
class ControlMode:
    VELOCITY = 0
    POWER = 1

class BrakeMode:
    BRAKE = 0
    COAST = 1

class MotorChannel:
    def __init__(self):
        """
        Initialize the MotorChannel with default control and brake modes.
        """
        self.lock = threading.Lock()

        self.control_mode = ControlMode.POWER
        self.brake_mode = BrakeMode.BRAKE

        # Variables to store current commands
        self._velocity_command = 0.0
        self._power_command = 0.0

        self._position = 0.0
        self._velocity = 0.0

    @property
    def velocity_command(self) -> float:
        """
        Get the velocity command with thread-safe access.
        """
        with self.lock:
            return self._velocity_command

    @velocity_command.setter
    def velocity_command(self, value: float):
        """
        Set the velocity command with thread-safe access and mode check.
        """
        with self.lock:
            if self._control_mode != ControlMode.VELOCITY:
                print("Warning: Setting velocity command with control mode set to power.")
            self._velocity_command = value

    @property
    def power_command(self) -> float:
        """
        Get the power command with thread-safe access.
        """
        with self.lock:
            return self._power_command

    @power_command.setter
    def power_command(self, value: float):
        """
        Set the power command with thread-safe access and mode check.
        """
        with self.lock:
            if self._control_mode != ControlMode.POWER:
                print("Warning: Setting power command with control mode set to velocity.")
            self._power_command = value

    @property
    def control_mode(self) -> int:
        """
        Get the control mode with thread-safe access.
        """
        with self.lock:
            return self._control_mode

    @control_mode.setter
    def control_mode(self, value: int):
        """
        Set the control mode with thread-safe access.
        """
        with self.lock:
            self._control_mode = value

    @property
    def brake_mode(self) -> int:
        """
        Get the brake mode with thread-safe access.
        """
        with self.lock:
            return self._brake_mode

    @brake_mode.setter
    def brake_mode(self, value: int):
        """
        Set the brake mode with thread-safe access.
        """
        with self.lock:
            self._brake_mode = value

    @property
    def command(self) -> float:
        """
        Get the command based on the current control mode.
        """
        return self.velocity_command if self.control_mode == ControlMode.VELOCITY else self.power_command

    @property
    def position(self) -> float:
        """
        Get the position with thread-safe access.
        """
        with self.lock:
            return self._position

    def update_position(self, value: float):
        """
        Update the position with thread-safe access.
        """
        with self.lock:
            self._position = value

    @property
    def velocity(self) -> float:
        """
        Get the velocity with thread-safe access.
        """
        with self.lock:
            return self._velocity

    def update_velocity(self, value: float):
        """
        Update the velocity with thread-safe access.
        """
        with self.lock:
            self._velocity = value

class OutputStruct:
    BUFFER_OUT_SIZE = 49

    def __init__(self, channel1: MotorChannel, channel2: MotorChannel, channel3: MotorChannel, channel4: MotorChannel):
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

    def get_packed_struct(self) -> bytes:
        """
        Pack the structure data into bytes for transmission.
        """
        return struct.pack('4f8B',
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
                           self.channel_4_brake_mode)

    def __str__(self) -> str:
        """
        Return a string representation of the output structure.
        """
        return (f"OutputStruct:\n"
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
                f"Channel 4 Brake Mode: {self.channel_4_brake_mode}\n")


class InputStruct:
    BUFFER_IN_SIZE = 33

    def __init__(self, data: bytes = None):
        """
        Initialize the input structure and decode data if provided.
        """
        if data is not None:
            self.decode(data)

    def decode(self, data: bytes):
        """
        Decode the input data into the structure fields.
        """
        unpacked_data = struct.unpack_from('<?8f', data)
        self.valid = unpacked_data[0]
        self.channel_1_pos = unpacked_data[1]
        self.channel_1_vel = unpacked_data[2]
        self.channel_2_pos = unpacked_data[3]
        self.channel_2_vel = unpacked_data[4]
        self.channel_3_pos = unpacked_data[5]
        self.channel_3_vel = unpacked_data[6]
        self.channel_4_pos = unpacked_data[7]
        self.channel_4_vel = unpacked_data[8]

    def __str__(self) -> str:
        """
        Return a string representation of the input structure.
        """
        return (f"InputStruct:\n"
                f"Valid: {self.valid}\n"
                f"Channel 1 Position: {self.channel_1_pos}\n"
                f"Channel 1 Velocity: {self.channel_1_vel}\n"
                f"Channel 2 Position: {self.channel_2_pos}\n"
                f"Channel 2 Velocity: {self.channel_2_vel}\n"
                f"Channel 3 Position: {self.channel_3_pos}\n"
                f"Channel 3 Velocity: {self.channel_3_vel}\n"
                f"Channel 4 Position: {self.channel_4_pos}\n"
                f"Channel 4 Velocity: {self.channel_4_vel}\n")

class Plink:

    def __init__(self, frequency: int = 100, timeout: float = 5.0):
        """
        Initialize the Plink communication object with motor channels and communication settings.
        """
        # self.spi = spidev.SpiDev()
        # self.spi.open(0, 0)  # Open bus 0, device (CS) 0
        # self.spi.mode = 0
        # self.spi.max_speed_hz = 50000  # Set SPI speed

        self.last_message_time = None

        self.channel1 = MotorChannel()
        self.channel2 = MotorChannel()
        self.channel3 = MotorChannel()
        self.channel4 = MotorChannel()

        self.frequency = frequency
        self.timeout = timeout

    def connect(self):
        """
        Start the communication thread.
        """
        atexit.register(self.shutdown)  # Register cleanup function

        self.running = True
        self.thread = threading.Thread(target=self.comms_thread)
        self.thread.daemon = True  # Set the thread as a daemon thread
        self.thread.start()


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

    def transfer(self):
        """
        Prepare and send data, then receive and process the response.
        """
        # Prepare data from current state
        data = OutputStruct(
            self.channel1,
            self.channel2,
            self.channel3,
            self.channel4)
        data.valid = True

        # Send data and receive response (Mock response for now)
        # response = InputStruct(self.spi.xfer2(data.get_packed_struct()))

        # Mock response for testing
        response = InputStruct()
        response.valid = True

        response.channel_1_pos = 1.0
        response.channel_1_vel = 2.0
        response.channel_2_pos = 3.0
        response.channel_2_vel = 4.0
        response.channel_3_pos = 5.0
        response.channel_3_vel = 6.0
        response.channel_4_pos = 7.0
        response.channel_4_vel = 8.0


        # Update the motor states
        if response.valid:
            self.update_motor_states(response)

            if self.last_message_time is None:
                print("Connection established!")

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

        print("Disconnecting from Plink ...")
        # Close the SPI connection
        # self.spi.close()
