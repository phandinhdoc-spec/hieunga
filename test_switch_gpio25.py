import time
from switch_gpio25 import SwitchGPIO25


def main():
    switch = SwitchGPIO25(pin=25)

    print("=== TEST SWITCH GPIO25 ===")
    print("GPIO25 = pin vật lý 22")
    print("Nhấn công tắc để test.")
    print("Ctrl+C để thoát.\n")

    try:
        last_state = switch.is_pressed()

        while True:
            current_state = switch.is_pressed()

            if current_state != last_state:
                if current_state:
                    print("PRESSED")
                else:
                    print("RELEASED")

                last_state = current_state

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nDừng test.")

    finally:
        switch.close()


if __name__ == "__main__":
    main()
