import RPi.GPIO as GPIO
import time

KEY1 = 20  # GPIO connecté à KEY1
KEY2 = 6
KEY3 = 13
KEY4 = 19
KEY5 = 26


GPIO.setmode(GPIO.BCM)
GPIO.setup(KEY1, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # si pas déjà en pull-up sur le module
GPIO.setup(KEY2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY4, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(KEY5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    while True:
        if GPIO.input(KEY1) == GPIO.LOW:
            print("Bouton appuyé !")
        if GPIO.input(KEY2) == GPIO.LOW:
            print("Bouton 2")
        if GPIO.input(KEY3) == GPIO.LOW:
            print("Bouton 3")
        if GPIO.input(KEY4) == GPIO.LOW:
            print("Bouton 4")
        if GPIO.input(KEY5)==GPIO.LOW:
            print("Bouton5")
        time.sleep(0.1)
finally:
    GPIO.cleanup()
