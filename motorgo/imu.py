import threading
import time

import imufusion
import numpy as np


class IMU:
    def __init__(self):
        """
        Initialize the IMU with default values.
        """
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0

        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z = 0.0

        self.gyro_mean = [0.0, 0.0, 0.0]

        self.lock = threading.Lock()

        self.ahrs = imufusion.Ahrs()
        self.ahrs.settings = imufusion.Settings(
            imufusion.CONVENTION_NWU,  # convention
            0.9,  # gain
            2000,  # gyroscope range
            10,  # acceleration rejection
            10,  # magnetic rejection
            5 * 200,  # recovery trigger period = 5 seconds
        )

        self.calibration_complete = False

    def calibrate(self):
        """
        Calibrate the IMU, finding the mean of the gyro data.
        """

        gyro_x_sum = 0
        gyro_y_sum = 0
        gyro_z_sum = 0

        print("Calibrating IMU, do not move the Plink!")
        for _ in range(1000):
            with self.lock:
                gyro_x_sum += self.gyro_x
                gyro_y_sum += self.gyro_y
                gyro_z_sum += self.gyro_z

            time.sleep(0.01)

        self.gyro_mean = [
            gyro_x_sum / 1000,
            gyro_y_sum / 1000,
            gyro_z_sum / 1000,
        ]

        print("IMU calibration complete!")

        self.calibration_complete = True

    def update(
        self,
        gyro_x: float,
        gyro_y: float,
        gyro_z: float,
        accel_x: float,
        accel_y: float,
        accel_z: float,
        mag_x: float,
        mag_y: float,
        mag_z: float,
        frequency: float,
    ):
        """
        Update the IMU data with the provided list.
        """

        gyro_np = np.zeros(3)
        accel_np = np.zeros(3)

        with self.lock:
            self.gyro_x = gyro_x - self.gyro_mean[0]
            self.gyro_y = gyro_y - self.gyro_mean[1]
            self.gyro_z = gyro_z - self.gyro_mean[2]

            self.accel_x = accel_x
            self.accel_y = accel_y
            self.accel_z = accel_z

            self.mag_x = mag_x
            self.mag_y = mag_y
            self.mag_z = mag_z

            # Collect data for AHRS update
            gyro_np[0] = gyro_x
            gyro_np[1] = gyro_y
            gyro_np[2] = gyro_z

            accel_np[0] = accel_x
            accel_np[1] = accel_y
            accel_np[2] = accel_z

        if self.calibration_complete:
            self.ahrs.update_no_magnetometer(gyro_np, accel_np, 1 / frequency)

    @property
    def gyro(self) -> list:
        """
        Get the gyroscope data as a list.
        """
        with self.lock:
            return [self.gyro_x, self.gyro_y, self.gyro_z]

    @property
    def accel(self) -> list:
        """
        Get the accelerometer data as a list.
        """

        with self.lock:
            return [self.accel_x, self.accel_y, self.accel_z]

    @property
    def mag(self) -> list:
        """
        Get the magnetometer data as a list.
        """
        with self.lock:
            return [self.mag_x, self.mag_y, self.mag_z]

    @property
    def gravity_vector(self) -> np.ndarray:
        """
        Get the gravity vector calculated by the AHRS.
        """

        if not self.calibration_complete:
            # Print warning in red
            print(
                "\033[91mWarning: IMU not calibrated, gravity vector is not being computed!\033[0m"
            )
            return np.zeros(3)

        return self.ahrs.gravity

    def __str__(self) -> str:
        """
        Return a string representation of the IMU data.
        """
        return (
            f"IMU:\n"
            f"Gyro X: {self.gyro_x}\n"
            f"Gyro Y: {self.gyro_y}\n"
            f"Gyro Z: {self.gyro_z}\n"
            f"Accel X: {self.accel_x}\n"
            f"Accel Y: {self.accel_y}\n"
            f"Accel Z: {self.accel_z}\n"
            f"Mag X: {self.mag_x}\n"
            f"Mag Y: {self.mag_y}\n"
            f"Mag Z: {self.mag_z}\n"
        )
