#!/usr/bin/env python3

import time

from hardware.vl53l1x_sensor import VL53L1XSensor


def main():
    print("GY53 / VL53L1X distance sensor test")
    print("I2C bus : 1")
    print("Address : 0x29")
    print("Press Ctrl+C to stop.")
    print()

    sensor = VL53L1XSensor(bus=1, address=0x29)

    try:
        sensor.open()
        sensor.start(ranging_mode=2)

        while True:
            distance_mm = sensor.get_distance_mm()

            print(
                f"Distance: {distance_mm:4d} mm "
                f"({distance_mm / 10:.1f} cm)"
            )

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nStopping sensor...")

    finally:
        sensor.close()

    print("Done.")


if __name__ == "__main__":
    main()
