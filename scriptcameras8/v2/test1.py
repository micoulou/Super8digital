import cv2
import time
import threading
import subprocess
from gpiozero import Button
from signal import pause
from datetime import datetime

# === CONFIGURATION ===
TRIGGER_GPIO = 26
SAVE_GPIO = 19
MAX_DURATION = 180  # 3 minutes max
FPS = 25
WIDTH = 640
HEIGHT = 480
CODEC = "v4l2"  # backend pour OpenCV
DEVICE = "/dev/video0"

# === VARIABLES DE CONTRÔLE ===
is_recording = False
recording_start_time = None
video_writer = None
frames = []
recording_thread = None
stop_flag = False
output_filename = ""

# === INITIALISATION CAMERA ===
cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

# === GESTION DES BOUTONS ===
trigger = Button(TRIGGER_GPIO, pull_up=True)
save = Button(SAVE_GPIO, pull_up=True)

def generate_filename():
    return datetime.now().strftime("video_%Y%m%d_%H%M%S.avi")

def record():
    global frames, recording_start_time, is_recording, stop_flag
    recording_start_time = time.time()
    print("[INFO] Démarrage de l’enregistrement")

    while not stop_flag and (time.time() - recording_start_time) < MAX_DURATION:
        ret, frame = cap.read()
        if ret:
            frames.append(frame)
        time.sleep(1 / FPS)

    print("[INFO] Fin temporaire ou limite atteinte")
    is_recording = False

def start_recording():
    global is_recording, stop_flag, recording_thread
    if not is_recording and len(frames) / FPS < MAX_DURATION:
        stop_flag = False
        is_recording = True
        recording_thread = threading.Thread(target=record)
        recording_thread.start()

def stop_recording():
    global stop_flag, is_recording
    if is_recording:
        stop_flag = True
        recording_thread.join()

def finalize_recording():
    global frames, output_filename
    if frames:
        output_filename = generate_filename()
        print(f"[INFO] Sauvegarde dans {output_filename}")
        height, width, _ = frames[0].shape
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(output_filename, fourcc, FPS, (width, height))
        for frame in frames:
            out.write(frame)
        out.release()
        print("[INFO] Enregistrement terminé.")
        frames = []

def on_trigger_pressed():
    print("[GPIO] Gâchette appuyée")
    start_recording()

def on_trigger_released():
    print("[GPIO] Gâchette relâchée")
    stop_recording()

def on_save_pressed():
    print("[GPIO] Sauvegarde forcée")
    stop_recording()
    finalize_recording()

# === ASSIGNATION DES ACTIONS ===
trigger.when_pressed = on_trigger_pressed
trigger.when_released = on_trigger_released
save.when_pressed = on_save_pressed

print("[READY] Caméra prête. Appuyez sur la gâchette pour enregistrer.")
try:
    pause()
finally:
    cap.release()
    print("[EXIT] Caméra libérée")

