import time

from gy25 import GY25


def main():
    with GY25(bus=3) as sensor:
        sensor.calibrate(samples=1000)

        print()
        print("GY25 running - Ctrl+C to stop")
        print("Yaw is relative to startup orientation.")
        print()

        try:
            while True:
                o = sensor.get_orientation()

                print(
                    f"\rROLL {o['roll']:8.2f}° | "
                    f"PITCH {o['pitch']:8.2f}° | "
                    f"YAW {o['yaw']:8.2f}°",
                    end="",
                    flush=True,
                )

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
