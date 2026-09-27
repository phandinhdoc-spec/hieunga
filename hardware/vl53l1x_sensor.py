"""Driver wrapper for the GY53 / VL53L1X distance sensor."""

import VL53L1X


class VL53L1XSensor:
    """Simple interface for the VL53L1X time-of-flight distance sensor."""

    def __init__(self, bus: int = 1, address: int = 0x29):
        self.bus = bus
        self.address = address
        self._sensor = None
        self._running = False

    def open(self) -> None:
        """Initialize the sensor."""
        if self._sensor is not None:
            return

        self._sensor = VL53L1X.VL53L1X(
            i2c_bus=self.bus,
            i2c_address=self.address,
        )
        self._sensor.open()

    def start(self, ranging_mode: int = 2) -> None:
        """
        Start distance measurement.

        ranging_mode:
            1 = short
            2 = medium
            3 = long
        """
        if self._sensor is None:
            self.open()

        if not self._running:
            self._sensor.start_ranging(ranging_mode)
            self._running = True

    def get_distance_mm(self) -> int:
        """Return measured distance in millimetres."""
        if not self._running:
            raise RuntimeError("Sensor is not ranging. Call start() first.")

        return self._sensor.get_distance()

    def get_distance_cm(self) -> float:
        """Return measured distance in centimetres."""
        return self.get_distance_mm() / 10.0

    def stop(self) -> None:
        """Stop ranging."""
        if self._sensor is not None and self._running:
            self._sensor.stop_ranging()
            self._running = False

    def close(self) -> None:
        """Stop and release the sensor."""
        self.stop()
        self._sensor = None

    def __enter__(self):
        self.open()
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
