from hardware_controller import HardwareController
import time

def test_leds(ctrl):
    print("🔴 Test LED rouge ON 1s")
    ctrl.led_red.on()
    time.sleep(1)

    print("🟢 Test LED verte ON 1s")
    ctrl.led_green.on()
    time.sleep(1)

    print("🔁 Clignotement LED rouge rapide (0.2s)")
    ctrl.led_red.blink(0.2)
    time.sleep(2)

    print("🔁 Clignotement LED verte lent (0.5s)")
    ctrl.led_green.blink(0.5)
    time.sleep(2)

    print("🛑 Extinction des LEDs")
    ctrl.led_red.off()
    ctrl.led_green.off()
    time.sleep(1)

def test_buzzer(ctrl):
    print("🚗 Test buzzer bruit moteur")
    ctrl.buzzer.play_engine()
    time.sleep(1.5)

    print("📸 Test buzzer bruit déclencheur")
    ctrl.buzzer.play_click()
    time.sleep(0.5)

    print("🔇 Test arrêt du buzzer")
    ctrl.buzzer.stop()
    time.sleep(0.5)

if __name__ == "__main__":
    print("🧪 Début des tests du module hardware...")
    ctrl = HardwareController()
    try:
        test_leds(ctrl)
        test_buzzer(ctrl)
        print("✅ Tous les tests se sont déroulés correctement.")
    except Exception as e:
        print("❌ Erreur pendant les tests :", e)
    finally:
        print("🧼 Nettoyage des GPIOs...")
        ctrl.cleanup()
        print("🔚 Fin des tests.")

