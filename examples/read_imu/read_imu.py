# spin_motors.py
# Before running this script, ensure that the MotorGo Plink is
# connected to the Raspberry Pi and that it has been flashed with the
# MotorGo firmware.

from pyplink import Plink, BrakeMode, ControlMode
import time


def main():
    # Create a Plink object
    plink = Plink()

    # This command will initiate communications and confirm
    # that the Plink is connected/available
    plink.connect()

    # The Plink object has an IMU object that reads all of the IMU data
    # You can save references as local variables for convenience (as below) or
    # access them directly from the Plink object
    imu = plink.imu

    while True:

        # You can read the position and velocity of the motor channels
        print("----")
        print(f"Acceleration: {imu.accel}")
        print(f"Angular Velocity: {imu.gyro}")
        print("----")

        # Delay as long as you need, communications continue in the background
        time.sleep(0.1)


if __name__ == "__main__":
    main()
