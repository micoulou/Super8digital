import RPi.GPIO as GPIO
import threading
import time

class LEDController:
    def __init__(self, pin):
        self.pin = pin
        self.blinking = False
        self.thread = None
        GPIO.setup(self.pin, GPIO.OUT)
        GPIO.output(self.pin, GPIO.LOW)

    def on(self):
        self.stop_blink()
        GPIO.output(self.pin, GPIO.HIGH)

    def off(self):
        self.stop_blink()
        GPIO.output(self.pin, GPIO.LOW)

    def blink(self, interval=0.5):
        self.stop_blink()
        self.blinking = True
        def _blink():
            while self.blinking:
                GPIO.output(self.pin, GPIO.HIGH)
                time.sleep(interval)
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(interval)
        self.thread = threading.Thread(target=_blink)
        self.thread.start()

    def stop_blink(self):
        self.blinking = False
        if self.thread:
            self.thread.join()
            self.thread = None

class BuzzerController:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 440)  # fréquence initiale
        self.playing = False

    def play_engine(self):
        self._play_pattern([100, 120, 140, 160, 180, 200, 220], 0.05)

    def play_click(self):
        self._play_pattern([1000], 0.1)

    def _play_pattern(self, freqs, duration):
        self.stop()
        self.playing = True
        def _play():
            for freq in freqs:
                if not self.playing:
                    break
                self.pwm.ChangeFrequency(freq)
                self.pwm.start(50)  # 50% duty cycle
                time.sleep(duration)
            self.pwm.stop()
        threading.Thread(target=_play).start()

    def stop(self):
        self.playing = False
        self.pwm.stop()

class HardwareController:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.led_red = LEDController(17)
        self.led_green = LEDController(27)
        self.buzzer = BuzzerController(18)

    def cleanup(self):
        self.led_red.off()
        self.led_green.off()
        self.buzzer.stop()
        GPIO.cleanup()

