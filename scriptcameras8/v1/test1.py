import cv2
import numpy as np
import time
import threading
import RPi.GPIO as GPIO
from datetime import datetime

# ============ CONFIGURATION ============
TRIGGER_GPIO = 17       # Gâchette de tournage
BUZZER_GPIO = 18        # Buzzer moteur
LED_RED_GPIO = 22       # LED Rouge
LED_GREEN_GPIO = 23     # LED Verte

WIDTH = 640
HEIGHT = 480
FRAMERATE = 18
MAX_RECORD_TIME = 180   # 3 minutes en secondes (fin de bobine)

# ============ INITIALISATION GPIO ============
GPIO.setmode(GPIO.BCM)

GPIO.setup(TRIGGER_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUZZER_GPIO, GPIO.OUT)
GPIO.setup(LED_RED_GPIO, GPIO.OUT)
GPIO.setup(LED_GREEN_GPIO, GPIO.OUT)

buzzer_pwm = GPIO.PWM(BUZZER_GPIO, 100)  # PWM pour le bruit moteur

# ============ FONCTIONS SUPER 8 ============
def add_super8_effect(frame):
    grain = np.random.normal(0, 10, frame.shape).astype(np.uint8)
    noisy = cv2.add(frame, grain)
    flicker = np.random.uniform(0.95, 1.05)
    flickered = cv2.convertScaleAbs(noisy, alpha=flicker, beta=0)
    return flickered

# ============ CLASSE CAMERA ============
class Super8Camera:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.out = None
        self.recording = False
        self.record_start_time = None
        self.clignote = False
        self.clignote_thread = None

    def start_recording(self):
        filename = f"super8_{datetime.now().strftime('%Y%m%d_%H%M%S')}.avi"
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter(filename, fourcc, FRAMERATE, (WIDTH, HEIGHT))
        self.recording = True
        self.record_start_time = time.time()
        print(f"Enregistrement commencé : {filename}")
        buzzer_pwm.start(30)  # Démarre le moteur
        set_led_recording(True)

    def stop_recording(self):
        if self.out:
            self.out.release()
            self.out = None
        self.recording = False
        print("Enregistrement arrêté.")
        buzzer_pwm.stop()
        set_led_recording(False)
        self.clignote = False

    def update_recording(self, frame):
        if self.recording and self.out:
            self.out.write(frame)

            # Gestion de la "fin de bobine"
            elapsed = time.time() - self.record_start_time
            if elapsed > MAX_RECORD_TIME and not self.clignote:
                self.clignote = True
                self.clignote_thread = threading.Thread(target=led_blink_red)
                self.clignote_thread.start()

    def release(self):
        self.cap.release()
        if self.out:
            self.out.release()
        GPIO.cleanup()
        cv2.destroyAllWindows()

# ============ GESTION LEDS ============
def set_led_recording(is_recording):
    if is_recording:
        GPIO.output(LED_GREEN_GPIO, GPIO.LOW)
        GPIO.output(LED_RED_GPIO, GPIO.HIGH)
    else:
        GPIO.output(LED_GREEN_GPIO, GPIO.HIGH)
        GPIO.output(LED_RED_GPIO, GPIO.LOW)

def led_blink_red():
    while True:
        if not camera.clignote:
            break
        GPIO.output(LED_RED_GPIO, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(LED_RED_GPIO, GPIO.LOW)
        time.sleep(0.5)

# ============ BOUCLE PRINCIPALE ============
camera = Super8Camera()

try:
    set_led_recording(False)
    while True:
        ret, frame = camera.cap.read()
        if not ret:
            continue

        frame = cv2.resize(frame, (WIDTH, HEIGHT))
        processed_frame = add_super8_effect(frame)

        trigger_pressed = (GPIO.input(TRIGGER_GPIO) == GPIO.LOW)

        if trigger_pressed and not camera.recording:
            camera.start_recording()

        elif not trigger_pressed and camera.recording:
            camera.stop_recording()

        camera.update_recording(processed_frame)

        # Affichage en live
        cv2.imshow('Super 8 Live View', processed_frame)

        if cv2.waitKey(int(1000 / FRAMERATE)) & 0xFF == ord('q'):
            break

finally:
    camera.release()

