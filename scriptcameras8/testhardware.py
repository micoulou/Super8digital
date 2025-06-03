from hardware_controller import HardwareController
import time

ctrl = HardwareController()

try:
    ctrl.led_red.blink(0.2)
    ctrl.led_green.on()
    time.sleep(2)

    ctrl.buzzer.play_engine()
    time.sleep(2)

    ctrl.led_red.off()
    ctrl.led_green.blink(0.5)
    ctrl.buzzer.play_click()
    time.sleep(1)

finally:
    ctrl.cleanup()
