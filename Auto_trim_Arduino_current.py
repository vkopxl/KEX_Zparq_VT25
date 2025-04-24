import time
import threading
import serial

# Denna klass hanterar kommunikationen med Arduino via Serial (USB)
class ArduinoSimulator:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, message_callback=None):
        # Öppnar seriell kommunikation till Arduino
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.running = True  # Flagga som styr om läsloopen kör
        self.message_callback = message_callback  # Callback-funktion för inkommande meddelanden
        self.thread = threading.Thread(target=self.read_loop)  # Startar tråd som läser från Arduino
        self.thread.start()

    def read_loop(self):
        # Oändlig loop som lyssnar efter meddelanden från Arduino
        while self.running:
            if self.ser.in_waiting:  # Om något finns att läsa
                line = self.ser.readline().decode('utf-8').strip()  # Läs och avkoda
                print(f"[Arduino] {line}")  # Skriv ut i terminalen
                if self.message_callback:
                    self.message_callback(line)  # Skicka meddelandet till callback-funktion

    def send_message(self, msg):
        # Skickar ett meddelande till Arduino
        if self.ser.is_open:
            self.ser.write((msg + "\n").encode())  # Skicka med newline

    def stop(self):
        # Avslutar tråden och stänger seriella porten
        self.running = False
        self.thread.join()
        self.ser.close()

# Huvudprogram
def main():
    stop_flag = threading.Event()  # Flagga som kan sättas för att avbryta optimering

    # Funktion som tar emot och tolkar meddelanden från Arduino
    def handle_message(msg):
        if msg == "STARTA":
            # Om användaren trycker START
            if not optimizer_thread.is_alive():
                print("[SIMULATOR] Startar optimering (10 sekunder)...")
                stop_flag.clear()  # Nollställ eventuell gammal avbryt-flagga
                optimizer_thread = threading.Thread(target=run_simulated_optimization)
                optimizer_thread.start()

        elif msg == "AVBRYT":
            # Om användaren trycker AVBRYT
            print("[SIMULATOR] Optimering avbruten.")
            stop_flag.set()  # Sätt flagga för att avbryta optimering

        elif "MANUELL TRIM UPP" in msg:
            print("[SIMULATOR] UP")  # Manuellt trimma upp

        elif "MANUELL TRIM NED" in msg:
            print("[SIMULATOR] DOWN")  # Manuellt trimma ner

    # Initiera Arduino-kommunikation och koppla ihop med callback
    arduino = ArduinoSimulator(port='/dev/ttyUSB0', message_callback=handle_message)

    # Funktion som simulerar en 10 sekunder lång optimering
    def run_simulated_optimization():
        for i in range(10):  # 10 sekunders körning
            if stop_flag.is_set():  # Om användaren tryckt AVBRYT
                arduino.send_message("AVBRUTEN")  # Skicka tillbaka till Arduino
                return
            print(f"[SIMULATOR] Optimerar... {i+1}/10")
            time.sleep(1)  # Vänta 1 sekund per iteration

        # När optimering är klar (ej avbruten)
        print("[SIMULATOR] Optimering klar!")
        arduino.send_message("KLAR")  # Informera Arduino

    # Förbered tråd för optimering (startas i callback)
    optimizer_thread = threading.Thread(target=run_simulated_optimization)

    try:
        print("[SIMULATOR] Väntar på kontrollpanel...")
        while True:
            time.sleep(0.1)  # Låt huvudtråden leva så vi kan få meddelanden från Arduino

    except KeyboardInterrupt:
        print("Avslutar simulering.")

    finally:
        # Se till att trådar och serialport stängs korrekt
        stop_flag.set()
        arduino.stop()

# Startar programmet
if __name__ == "__main__":
    main()
