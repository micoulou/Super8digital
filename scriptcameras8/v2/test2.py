import RPi.GPIO as GPIO
import cv2
import time
import threading
import signal
import sys

# Configuration GPIO
GPIO.setmode(GPIO.BCM)

TRIGGER_GPIO = 26  # bouton d'enregistrement
STOP_GPIO = 13     # bouton d'arrêt du script
BUZZER_GPIO = 18   # buzzer moteur

GPIO.setup(TRIGGER_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(STOP_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUZZER_GPIO, GPIO.OUT)

# Buzzer en PWM
buzzer = GPIO.PWM(BUZZER_GPIO, 1000)
buzzer_running = False

# Capture vidéo avec OpenCV
cap = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*'H264')
out = cv2.VideoWriter('video_super8.mp4', fourcc, 25.0, (640, 480))

recording = False
exit_requested = False

# Thread pour simuler le moteur
def moteur_sound():
    global buzzer_running
    buzzer_running = True
    buzzer.start(50)
    try:
        while buzzer_running:
            freq = 1000 + (time.time() * 100 % 300)
            buzzer.ChangeFrequency(freq)
            time.sleep(0.05)
    except:
        pass
    buzzer.stop()

def stop_script(channel=None):
    global exit_requested
    print("🔴 Arrêt demandé via GPIO 13.")
    exit_requested = True

GPIO.add_event_detect(STOP_GPIO, GPIO.FALLING, callback=stop_script, bouncetime=300)

print("🎬 Script prêt. Appuie sur GPIO 26 pour enregistrer, GPIO 13 pour quitter.")

try:
    moteur_thread = None

    while not exit_requested:
        if GPIO.input(TRIGGER_GPIO) == GPIO.LOW and not recording:
            print("⏺️ Enregistrement démarré")
            recording = True
            moteur_thread = threading.Thread(target=moteur_sound)
            moteur_thread.start()

        elif GPIO.input(TRIGGER_GPIO) == GPIO.HIGH and recording:
            print("⏹️ Enregistrement arrêté")
            recording = False
            buzzer_running = False
            if moteur_thread:
                moteur_thread.join()

        if recording:
            ret, frame = cap.read()
            if ret:
                out.write(frame)
            else:
                print("⚠️ Erreur de lecture caméra")

        time.sleep(0.01)

except KeyboardInterrupt:
    print("🛑 Interruption clavier")

finally:
    print("🔧 Nettoyage et fermeture...")
    buzzer_running = False
    if moteur_thread:
        moteur_thread.join()
    cap.release()
    out.release()
    GPIO.cleanup()
    print("✅ Script terminé proprement.")

