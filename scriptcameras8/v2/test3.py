from gpiozero import Button, LED, Buzzer
from signal import pause
import cv2
import time
import threading
import os

# Configuration des GPIO
record_button = Button(26)      # Gâchette d'enregistrement
stop_button = Button(19)        # Bouton pour terminer l'enregistrement et sauvegarder
exit_button = Button(13)        # Bouton pour quitter proprement
led = LED(21)                   # LED d'indication
buzzer = Buzzer(18)             # Buzzer actif (pas passif sinon PWM requis)

# Paramètres caméra
WIDTH, HEIGHT = 640, 480
FPS = 25
MAX_DURATION = 180  # secondes

# Initialisation OpenCV
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

fourcc = cv2.VideoWriter_fourcc(*'MJPG')
output_filename = ""
video_writer = None

recording = False
start_time = None
video_thread = None
exit_requested = False

def start_recording():
    global recording, video_writer, output_filename, start_time

    if not recording:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output_filename = f"super8_{timestamp}.avi"
        video_writer = cv2.VideoWriter(output_filename, fourcc, FPS, (WIDTH, HEIGHT))
        recording = True
        start_time = time.time()
        led.on()
        buzzer.on()
        print("🎥 Enregistrement démarré.")

def stop_recording(save=True):
    global recording, video_writer
    if recording:
        recording = False
        buzzer.off()
        led.off()
        if save and video_writer:
            video_writer.release()
            print(f"💾 Vidéo sauvegardée : {output_filename}")
        elif video_writer:
            video_writer.release()
            os.remove(output_filename)
            print("🗑️ Enregistrement annulé.")

def handle_exit():
    global exit_requested
    print("🚪 Arrêt demandé...")
    exit_requested = True
    stop_recording()
    cap.release()
    cv2.destroyAllWindows()

def record_loop():
    global recording, start_time, exit_requested
    while not exit_requested:
        if recording:
            ret, frame = cap.read()
            if ret:
                video_writer.write(frame)
            elapsed = time.time() - start_time
            if elapsed > MAX_DURATION:
                print("⏱️ Durée maximale atteinte.")
                stop_recording()
        time.sleep(1.0 / FPS)

# Lancement du thread vidéo
video_thread = threading.Thread(target=record_loop)
video_thread.start()

# Événements boutons
record_button.when_pressed = start_recording
record_button.when_released = lambda: stop_recording(save=True)
stop_button.when_pressed = lambda: stop_recording(save=True)
exit_button.when_pressed = handle_exit

print("📽️ Caméra Super 8 prête. Appuie sur la gâchette (GPIO 26) pour filmer.")
pause()

