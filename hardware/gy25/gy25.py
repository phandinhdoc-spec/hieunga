import math
import time
from smbus2 import SMBus


class KalmanAngle:
    def __init__(self):
        self.angle = 0.0
        self.bias = 0.0
        self.P = [[0.0, 0.0], [0.0, 0.0]]

        self.Q_angle = 0.001
        self.Q_bias = 0.003
        self.R_measure = 0.03

    def update(self, measured_angle, gyro_rate, dt):
        rate = gyro_rate - self.bias
        self.angle += dt * rate

        self.P[0][0] += dt * (
            dt * self.P[1][1]
            - self.P[0][1]
            - self.P[1][0]
            + self.Q_angle
        )
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        innovation = measured_angle - self.angle
        s = self.P[0][0] + self.R_measure

        k0 = self.P[0][0] / s
        k1 = self.P[1][0] / s

        self.angle += k0 * innovation
        self.bias += k1 * innovation

        p00 = self.P[0][0]
        p01 = self.P[0][1]

        self.P[0][0] -= k0 * p00
        self.P[0][1] -= k0 * p01
        self.P[1][0] -= k1 * p00
        self.P[1][1] -= k1 * p01

        return self.angle


class GY25:
    ADDRESS = 0x68

    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    WHO_AM_I = 0x75

    ACCEL_SCALE = 16384.0
    GYRO_SCALE = 131.0

    def __init__(self, bus=3, address=ADDRESS):
        self.bus_number = bus
        self.address = address
        self.bus = SMBus(bus)

        self.kalman_roll = KalmanAngle()
        self.kalman_pitch = KalmanAngle()

        self.gyro_offset = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.last_time = time.monotonic()

        self._initialize()

    def _initialize(self):
        who = self.bus.read_byte_data(self.address, self.WHO_AM_I)

        if who != 0x68:
            raise RuntimeError(
                f"MPU6050 not detected correctly: WHO_AM_I=0x{who:02X}"
            )

        # Wake MPU6050 and use X gyro PLL as clock.
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x01)

        time.sleep(0.1)

    @staticmethod
    def _signed(value):
        return value - 65536 if value >= 32768 else value

    def _read_word(self, register):
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        return self._signed((high << 8) | low)

    def read_raw(self):
        data = self.bus.read_i2c_block_data(
            self.address,
            self.ACCEL_XOUT_H,
            14,
        )

        def word(i):
            return self._signed((data[i] << 8) | data[i + 1])

        return {
            "accel": (word(0), word(2), word(4)),
            "temperature": word(6),
            "gyro": (word(8), word(10), word(12)),
        }

    def calibrate(self, samples=1000, delay=0.002):
        print("Calibrating gyro - keep GY25 completely still...")

        gx_sum = 0.0
        gy_sum = 0.0
        gz_sum = 0.0

        for _ in range(samples):
            raw = self.read_raw()
            gx, gy, gz = raw["gyro"]

            gx_sum += gx / self.GYRO_SCALE
            gy_sum += gy / self.GYRO_SCALE
            gz_sum += gz / self.GYRO_SCALE

            time.sleep(delay)

        self.gyro_offset = [
            gx_sum / samples,
            gy_sum / samples,
            gz_sum / samples,
        ]

        # Initialize Kalman state from gravity.
        raw = self.read_raw()
        ax, ay, az = [
            value / self.ACCEL_SCALE for value in raw["accel"]
        ]

        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(
            math.atan2(-ax, math.sqrt(ay * ay + az * az))
        )

        self.kalman_roll.angle = roll
        self.kalman_pitch.angle = pitch

        self.yaw = 0.0
        self.last_time = time.monotonic()

        print(
            "Gyro offsets: "
            f"X={self.gyro_offset[0]:.4f}, "
            f"Y={self.gyro_offset[1]:.4f}, "
            f"Z={self.gyro_offset[2]:.4f} deg/s"
        )

    def get_orientation(self):
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        # Avoid a huge integration step after pauses/debugging.
        dt = min(max(dt, 0.0001), 0.1)

        raw = self.read_raw()

        ax, ay, az = [
            value / self.ACCEL_SCALE for value in raw["accel"]
        ]

        gx, gy, gz = [
            value / self.GYRO_SCALE
            for value in raw["gyro"]
        ]

        gx -= self.gyro_offset[0]
        gy -= self.gyro_offset[1]
        gz -= self.gyro_offset[2]

        accel_roll = math.degrees(math.atan2(ay, az))

        accel_pitch = math.degrees(
            math.atan2(
                -ax,
                math.sqrt(ay * ay + az * az),
            )
        )

        roll = self.kalman_roll.update(
            accel_roll,
            gx,
            dt,
        )

        pitch = self.kalman_pitch.update(
            accel_pitch,
            gy,
            dt,
        )

        # MPU6050 has no magnetometer.
        # Yaw is therefore relative to startup and is obtained from
        # bias-corrected Z gyro integration.
        self.yaw += gz * dt

        # Keep yaw convenient to consume.
        self.yaw = (self.yaw + 180.0) % 360.0 - 180.0

        return {
            "roll": roll,
            "pitch": pitch,
            "yaw": self.yaw,
            "gyro": {
                "x": gx,
                "y": gy,
                "z": gz,
            },
            "accel": {
                "x": ax,
                "y": ay,
                "z": az,
            },
            "dt": dt,
        }

    def reset_yaw(self):
        self.yaw = 0.0

    def close(self):
        self.bus.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
