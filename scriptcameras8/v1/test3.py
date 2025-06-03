# super8_camera_control.py

import RPi.GPIO as GPIO
import time
import threading
import os
from picamera2 import Picamera2, Preview
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
import pygame

# --- Configuration GPIO ---
TRIGGER_PIN = 17  # Gâchette sur GPIO17
KEY2_PIN = 6
KEY3_PIN = 13
KEY4_PIN = 19
KEY5_PIN = 26
BUZZER_PIN = 18   # Buzzer avec PWM sur GPIO18
LED_PIN_GREEN = 22
LED_PIN_RED = 23

# --- Durée max d'enregistrement ---
MAX_RECORD_TIME = 180  # 3 minutes max

# --- Initialisation OLED ---
serial = i2c(port=1, address=0x3C)
oled = ssd1306(serial, width=128, height=32)
font = ImageFont.load_default()

def update_oled(recording, elapsed_time):
    image = Image.new("1", (oled.width, oled.height))
    draw = ImageDraw.Draw(image)

    # Barre de progression en haut
    progress_ratio = min(elapsed_time / MAX_RECORD_TIME, 1.0)
    bar_width = int(progress_ratio * oled.width)
    draw.rectangle([0, 0, bar_width, 6], fill=255)

    # Symboles en bas
    if recording:
        draw.ellipse([4, 22, 12, 30], fill=255)  # Cercle REC
        draw.text((20, 20), "REC", font=font, fill=255)
    elif 0 < elapsed_time < MAX_RECORD_TIME:
        draw.rectangle([4, 22, 7, 30], fill=255)  # Pause ||
        draw.rectangle([9, 22, 12, 30], fill=255)
        draw.text((20, 20), "PAUSE", font=font, fill=255)
    else:
        draw.rectangle([4, 22, 12, 30], fill=255)  # Stop ⏹️
        draw.text((20, 20), "STOP", font=font, fill=255)

    oled.display(image)

# --- Initialisation GPIO ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGGER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY3_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY4_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY5_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(LED_PIN_GREEN, GPIO.OUT)
GPIO.setup(LED_PIN_RED, GPIO.OUT)

buzzer = GPIO.PWM(BUZZER_PIN, 80)

# --- Initialisation caméra ---
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration())
picam2.start()

# --- Contrôle d'enregistrement ---
recording = False
start_time = None
video_thread = None

# --- Buzzer moteur ---
def motor_buzzer_loop():
    buzzer.start(30)  # 30% duty cycle
    while recording:
        time.sleep(0.1)
    buzzer.stop()

# --- LED bicolore ---
def update_leds(elapsed):
    if elapsed >= 177:
        GPIO.output(LED_PIN_GREEN, elapsed % 2 < 1)  # clignote
        GPIO.output(LED_PIN_RED, elapsed % 2 >= 1)
    elif recording:
        GPIO.output(LED_PIN_RED, True)
        GPIO.output(LED_PIN_GREEN, False)
    else:
        GPIO.output(LED_PIN_RED, False)
        GPIO.output(LED_PIN_GREEN, True)

# --- Enregistrement vidéo ---
def record_video():
    global recording, start_time
    filename = f"video_{int(time.time())}.h264"
    picam2.start_recording(filename)
    buzzer_thread = threading.Thread(target=motor_buzzer_loop)
    buzzer_thread.start()

    start_time = time.time()
    while recording and (time.time() - start_time) < MAX_RECORD_TIME:
        elapsed = time.time() - start_time
        update_oled(True, elapsed)
        update_leds(elapsed)
        time.sleep(0.5)

    picam2.stop_recording()
    update_oled(False, time.time() - start_time)
    update_leds(MAX_RECORD_TIME)
    print("Recording stopped.")

# --- Boucle principale ---
try:
    print("Super 8 numérique prêt !")
    update_oled(False, 0)
    while True:
        if GPIO.input(TRIGGER_PIN) == GPIO.LOW:
            if not recording:
                recording = True
                video_thread = threading.Thread(target=record_video)
                video_thread.start()
        else:
            if recording:
                recording = False
                video_thread.join()
                update_oled(False, time.time() - start_time)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Arrêt manuel.")
finally:
    GPIO.cleanup()
    buzzer.stop()
    oled.clear()
    print("Nettoyage terminé.")

