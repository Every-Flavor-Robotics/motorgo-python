# read_imu.py
# Before running this script, ensure that the MotorGo Plink is
# connected to the Raspberry Pi and that it has been flashed with the
# PyPlink firmware.

from pyplink import Plink
import time


def main():
    # Create a Plink object
    plink = Plink()

    # This command will initiate communications and confirm
    # that the Plink is connected/available
    plink.connect()

    # The Plink object has an IMU object, corresponding to the 4 motor channels
    # You can save a reference as a local variable for convenience (as below) or
    # access them directly from the Plink object
    imu = plink.imu

    # The IMU object provides the raw IMU data:
    # - 3-axis accelerometer data in m/s^2
    # - 3-axis gyroscope data in rad/s
    # - 3-axis magnetometer data in uT

    # Additionally, it can povided fused 3-axis orientation data in the form of
    # of a gravity vector relative to the sensor frame
    # The orientation data will only be computed if the IMU object
    # is calibrated. Calibrating requires that the Plink be stationary
    imu.calibrate()

    while True:
        # Print out the IMU data
        print("----")
        print(f"Acceleration: {imu.accel}")
        print(f"Angular Velocity: {imu.gyro}")
        print(f"Magnetic Field: {imu.mag}")
        print(f"Gravity Vector: {imu.gravity_vector}")
        print("----")

        # Delay as long as you need, communications continue in the background
        time.sleep(0.1)


if __name__ == "__main__":
    main()
