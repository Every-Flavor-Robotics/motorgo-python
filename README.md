<h1 align="center">🐍 | motorgo-python | 🐍</h1>

<p align="center">
  Python API for controlling PiHat-compatible MotorGo boards from the Raspberry Pi.
</p>

---

## Introduction

`motorgo-python` provides a Python API to control a MotorGo when it is mounted to a Raspberry Pi as a PiHat. It enables you to control the motor channels and stream the enocder and IMU data from the MotorGo. This package also includes a `motorgo` command line utility (CLI) that lets you configure settings on the MotorGo and perform basic operations without the need for code.

Please follow our setup guide at (our docs)[https://docs.motorgo.net].

## Supported Boards

Currently, `motorgo-python` only supports the Plink. Support for the Axis is on the way!

## Installation

You can install `motorgo-python` using pip.

```sh
pip install motorgo-python
```

If you are using Python 3.12 or greater, you may need to use the `--break-system-packages` argument. `motorgo-python` cannot be installed in a virtual env as it needs access to

Next, you need to configure the MotorGo to be used in PiHat mode, which can be done using the included CLI. Attach the MotorGo to the 40-pin Pi header and run the command:

```
motorgo flash
```

## Usage
Below is an example program that demonstrates how to use the pyplink package to control motor channels on the MotorGo Plink board.

This example, and others, can be found in the `examples/` directory of this repository.

```python
# spin_motors.py
# Before running this script, ensure that the MotorGo Plink is
# connected to the Raspberry Pi and that it has been flashed with the
# MotorGo firmware.

import time

from motorgo import BrakeMode, ControlMode, Plink


def main():
    # Create a Plink object, the main interface to the MotorGo board
    plink = Plink()

    # The first thing to set up for a Plink is the power supply voltage.
    # This is the voltage you are providing to the Plink on the barrel jack.
    # If this is the battery, make sure this is the charged voltage.
    plink.power_supply_voltage = 9.0

    # The Plink object has 4 MotorChannel objects, corresponding to the 4 motor channels
    # on the board
    # You can access them directly: plink.channel1
    # Or you can save references as local variables for convenience:
    left_motor = plink.channel1
    right_motor = plink.channel2

    # Next, you need to set the motor voltage limit.
    # This is the maximum voltage your motor channels will output to protect the motors.
    # The voltage limit is 0 by default, which means the motors will not move if this is not set.
    left_motor.motor_voltage_limit = 6.0
    right_motor.motor_voltage_limit = 6.0
    plink.channel3.motor_voltage_limit = 6.0
    plink.channel4.motor_voltage_limit = 6.0

    # Finally, connect to the MotorGo board and psuh the configuration
    plink.connect()

    # You can configure how you want to control the motor channels.
    # Power mode: Set the power of the motor in the range [-1, 1]
    #            This directly corresponds to setting a voltage to the motor
    #
    # Velocity mode: Set the velocity of the motor in rad/s
    #              This mode requires setting the velocity PID gains
    #              It also requires an encoder to be connected to the motor
    left_motor.control_mode = ControlMode.POWER
    right_motor.control_mode = ControlMode.VELOCITY
    plink.channel3.control_mode = ControlMode.POWER
    plink.channel4.control_mode = ControlMode.POWER

    # If you are using ControlMode.VELOCITY, you must set the velocity PID gains
    right_motor.set_velocity_pid_gains(4.5, 0.1, 0.0)

    while True:

        # Set motor powers
        # Set the power command in the range [-1, 1]
        left_motor.power_command = 0.5

        # Set the velocity command in rad/s
        right_motor.velocity_command = 1.2

        # Set the power command in the range [-1, 1]
        plink.channel3.power_command = 0.25
        plink.channel4.power_command = -0.25

        # You can read the position and velocity of the motor channels from the encoders
        print("----")
        print(
            f"Channel 1 position: {plink.channel1.position}, velocity: {plink.channel1.velocity}"
        )
        print(
            f"Channel 2 position: {plink.channel2.position}, velocity: {plink.channel2.velocity}"
        )
        print(
            f"Channel 3 position: {plink.channel3.position}, velocity: {plink.channel3.velocity}"
        )
        print(
            f"Channel 4 position: {plink.channel4.position}, velocity: {plink.channel4.velocity}"
        )
        print("----")

        time.sleep(0.01)


if __name__ == "__main__":
    main()

```



