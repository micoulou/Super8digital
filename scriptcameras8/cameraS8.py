#!/usr/bin/python3

import cv2
import numpy as np
import time
import threading
import RPi.GPIO as GPIO
from datetime import datetime
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
import pygame
import os
import shutil

# ============ CONFIGURATION ============
TRIGGER_GPIO = 17       # Gâchette de tournage
KEY2_PIN = 6
KEY3_PIN = 13
KEY4_PIN = 19
KEY5_PIN = 26
BUZZER_GPIO = 18        # Buzzer moteur
LED_RED_GPIO = 22       # LED Rouge
LED_GREEN_GPIO = 23     # LED Verte

WIDTH = 640
HEIGHT = 480
FRAMERATE = 18
MAX_RECORD_TIME = 180   # 3 minutes en secondes (fin de bobine)
FPS_CAMERA_MODULE = 25
MAX_FRAME_RECORDED_BY_FILE = MAX_RECORD_TIME*FPS_CAMERA_MODULE

# --- Initialisation OLED ---
serial = i2c(port=1, address=0x3C)
oled = ssd1306(serial, width=128, height=32)
font = ImageFont.load_default()

# ============ INITIALISATION GPIO ============
GPIO.setmode(GPIO.BCM)

GPIO.setup(TRIGGER_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY3_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY4_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY5_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.setup(BUZZER_GPIO, GPIO.OUT)
GPIO.setup(LED_RED_GPIO, GPIO.OUT)
GPIO.setup(LED_GREEN_GPIO, GPIO.OUT)

buzzer_pwm = GPIO.PWM(BUZZER_GPIO, 100)  # PWM pour le bruit moteur

recording = False
start_time = None
video_thread = None
frame_recorded = 0

# ============ FONCTIONS SUPER 8 ============
def add_super8_effect(frame):
    grain = np.random.normal(0, 10, frame.shape).astype(np.uint8)
    noisy = cv2.add(frame, grain)
    flicker = np.random.uniform(0.95, 1.05)
    flickered = cv2.convertScaleAbs(noisy, alpha=flicker, beta=0)
    return flickered

def update_oled(recording, elapsed_time):
    image = Image.new("1", (oled.width, oled.height))
    draw = ImageDraw.Draw(image)

    # Barre de progression en haut
    progress_ratio = min(MAX_FRAME_RECORDED_BY_FILE / frame_recorded, 1.0)
    bar_width = int(progress_ratio * oled.width)
    draw.rectangle([0, 0, bar_width, 6], fill=255)

    # Symboles en bas
    if recording:
        draw.ellipse([4, 22, 12, 30], fill=255)  # Cercle REC
        draw.text((20, 20), "REC", font=font, fill=255)
    elif 0 < frame_recorded < MAX_FRAME_RECORDED_BY_FILE:
        draw.rectangle([4, 22, 7, 30], fill=255)  # Pause ||
        draw.rectangle([9, 22, 12, 30], fill=255)
        draw.text((20, 20), "PAUSE", font=font, fill=255)
    else:
        draw.rectangle([4, 22, 12, 30], fill=255)  # Stop ⏹️
        draw.text((20, 20), "STOP", font=font, fill=255)

    oled.display(image)

def apply_lut_to_video(input_path, output_path, lut_path):
    # Appliquer le LUT avec ffmpeg
    subprocess.run([
        "ffmpeg",
        "-i", input_path,
        "-vf", f"lut3d={lut_path}",
        "-c:a", "copy",
        output_path
    ])

def on_save_lut(channel):
    print("🎞️ Application de l'effet Kodachrome périmé...")
    input_video = "/home/pi/video_output/latest_video.mp4"
    output_video = "/home/pi/video_output/kodachrome_video.mp4"
    lut_file = "/home/pi/luts/kodachrome_perime.cube"
    apply_lut_to_video(input_video, output_video, lut_file)
    print("✅ Vidéo sauvegardée avec LUT appliquée !")

def on_exit(channel):
    print("🚪 Sortie du script proprement...")
    cleanup()
    exit(0)

def on_save(channel):
    print("💾 Tentative de sauvegarde sur carte SD SPI...")

    mount_point = "/mnt/sdcard"
    device = "/dev/sda1"  # À ajuster si besoin (selon où est reconnue ta carte SPI)

    try:
        os.makedirs(mount_point, exist_ok=True)
        # Monter la carte si elle n'est pas déjà montée
        if not os.path.ismount(mount_point):
            os.system(f"sudo mount {device} {mount_point}")
            print("📂 Carte SD montée.")

        # Chemins à copier
        source_files = [
            "/home/mikey/video_output/*.avi"
        ]
        for file in source_files:
            if os.path.exists(file):
                shutil.copy(file, mount_point)
                print(f"✅ Copié : {os.path.basename(file)}")
            else:
                print(f"⚠️ Fichier introuvable : {file}")

        # Synchroniser et démonter
        os.system("sync")
        os.system(f"sudo umount {mount_point}")
        print("Carte SD démontée proprement.")

    except Exception as e:
        print(f"❌ Erreur lors de la sauvegarde sur SD : {e}")


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
        start_time = time.time()


    def start_recording(self):
        filename = f"super8_{datetime.now().strftime('%Y%m%d_%H%M%S')}.avi"
        #fourcc = cv2.VideoWriter_fourcc(*'XVID')
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
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
            frame_recorded = frame_recorded + 1
            # Gestion de la "fin de bobine"
            elapsed = time.time() - self.record_start_time
            if frame_recorded > MAX_FRAME_RECORDED_BY_FILE and not self.clignote:
                self.clignote = True
                self.clignote_thread = threading.Thread(target=led_blink_red)
                self.clignote_thread.start()

    def release(self):
        self.cap.release()
        if self.out:
            self.out.release()
        GPIO.cleanup()
        cv2.destroyAllWindows()

# --- Buzzer moteur ---
def motor_buzzer_loop():
    buzzer.start(30)  # 30% duty cycle
    while recording:
        time.sleep(0.1)
    buzzer.stop()

# ============ GESTION LEDS ============
def set_led_recording(is_recording):
    if frame_recorded >= (MAX_FRAME_RECORDED_BY_FILE - (FPS_CAMERA_MODULE * 10)):
        GPIO.output(LED_PIN_GREEN, frame_recorded % 2 < 1)  # clignote
        GPIO.output(LED_PIN_RED, GPIO.HIGH)
    elif is_recording:
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
    GPIO.add_event_detect(KEY2_PIN, GPIO.FALLING, callback=on_save_lut, bouncetime=500)
    GPIO.add_event_detect(KEY2_PIN, GPIO.FALLING, callback=on_save, bouncetime=500)
    GPIO.add_event_detect(KEY5_PIN, GPIO.FALLING, callback=on_exit, bouncetime=500)
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