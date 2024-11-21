import threading
import time

import imufusion
import numpy as np


class IMU:
    def __init__(self, frequency):
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

        self.frequency = frequency

        self.lock = threading.Lock()

        self.ahrs = imufusion.Ahrs()
        self.ahrs.settings = imufusion.Settings(
            imufusion.CONVENTION_NWU,  # convention
            2.0,  # gain
            2000,  # gyroscope range
            10,  # acceleration rejection
            0,  # magnetic rejection
            self.frequency * 5,  # recovery trigger period = 5 seconds
        )

        self.offset = imufusion.Offset(self.frequency)
        self.last_update_time = None

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
            self.gyro_x = gyro_x  # - self.gyro_mean[0]
            self.gyro_y = gyro_y  # - self.gyro_mean[1]
            self.gyro_z = gyro_z  # - self.gyro_mean[2]

            self.accel_x = accel_x
            self.accel_y = accel_y
            self.accel_z = accel_z

            self.mag_x = mag_x
            self.mag_y = mag_y
            self.mag_z = mag_z

            # Collect data for AHRS update
            gyro_np[0] = self.gyro_x
            gyro_np[1] = self.gyro_y
            gyro_np[2] = self.gyro_z

            accel_np[0] = self.accel_x
            accel_np[1] = self.accel_y
            accel_np[2] = self.accel_z

        if self.last_update_time is not None:
            dt = time.time() - self.last_update_time

        else:
            dt = 1 / frequency

        gyro_np = self.offset.update(gyro_np)
        self.ahrs.update_no_magnetometer(gyro_np, accel_np, dt)

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
