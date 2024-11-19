import threading


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
                print(
                    "Warning: Setting velocity command with control mode set to power."
                )
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
                print(
                    "Warning: Setting power command with control mode set to velocity."
                )
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
        return (
            self.velocity_command
            if self.control_mode == ControlMode.VELOCITY
            else self.power_command
        )

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
