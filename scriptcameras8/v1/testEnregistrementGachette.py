import cv2
import numpy as np
import time
import RPi.GPIO as GPIO
from datetime import datetime

# === CONFIG ===
TRIGGER_GPIO = 17  # Gâchette sur GPIO 17
WIDTH = 640
HEIGHT = 480
FRAMERATE = 18

# === GPIO SETUP ===
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGGER_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# === VIDÉO SETUP ===
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

out = None
recording = False

# === Fonction d'effet Super 8 (optionnelle) ===
def add_super8_effect(frame):
    grain = np.random.normal(0, 10, frame.shape).astype(np.uint8)
    noisy = cv2.add(frame, grain)
    flicker = np.random.uniform(0.95, 1.05)
    flickered = cv2.convertScaleAbs(noisy, alpha=flicker, beta=0)
    return flickered

# === Fonction pour commencer l'enregistrement ===
def start_recording():
    global out
    filename = f"super8_{datetime.now().strftime('%Y%m%d_%H%M%S')}.avi"
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(filename, fourcc, FRAMERATE, (WIDTH, HEIGHT))
    print(f"Enregistrement commencé : {filename}")

# === Fonction pour arrêter l'enregistrement ===
def stop_recording():
    global out
    if out:
        out.release()
        out = None
        print("Enregistrement arrêté.")

# === BOUCLE PRINCIPALE ===
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        frame = cv2.resize(frame, (WIDTH, HEIGHT))
        processed = add_super8_effect(frame)

        trigger_pressed = (GPIO.input(TRIGGER_GPIO) == GPIO.LOW)

        if trigger_pressed and not recording:
            start_recording()
            recording = True

        elif not trigger_pressed and recording:
            stop_recording()
            recording = False

        if recording and out:
            out.write(processed)

        # Affichage en temps réel
        cv2.imshow('Super 8 Camera', processed)

        if cv2.waitKey(int(1000 / FRAMERATE)) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    if out:
        out.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()

