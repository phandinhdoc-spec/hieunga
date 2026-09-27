from gpiozero import Device, Button
from gpiozero.pins.lgpio import LGPIOFactory

Device.pin_factory = LGPIOFactory()


class SwitchGPIO25:
    def __init__(self, pin=25):
        self.pin = pin
        self.button = Button(
            pin,
            pull_up=True,
            bounce_time=0.05
        )

    def is_pressed(self):
        return self.button.is_pressed

    def wait_for_press(self):
        self.button.wait_for_press()

    def wait_for_release(self):
        self.button.wait_for_release()

    def close(self):
        self.button.close()
