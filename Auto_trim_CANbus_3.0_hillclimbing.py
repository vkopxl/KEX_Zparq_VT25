# Auto-Trim Version 3.0
# Baserad på 3.0_SIM verifierad på land.
# Uppdaterade Algorithm för att säkertställa funktionalitet.
# bugfixes
# Till nästa version:
# - Gradient Descent
# - Uppdaterade parameterar baserat på testrun 

#Importera nödvändiga bibliotek
import can  # För att kommunicera med CAN-bussen
import time  # För tidsstyrning och pauser
import threading  # För att hantera parallella trådar
import numpy as np  # För matematiska beräkningar
from collections import deque  # För att skapa buffertar
import csv # För att spara dokumentation i csv filer
import os # Filhantering för dokumentation


# Klass som hanterar CAN-kommunikation
class CANInterface:
    def __init__(self, channel='PCAN_USBBUS1', bitrate=1000000):
        self.bus = can.interface.Bus(interface='pcan', channel=channel, bitrate=bitrate)
        
        # Buffrar input data och tar genomsnitt för femte mätning
        self.speed_buffer = deque(maxlen=5) 
        self.vridmoment_buffer = deque(maxlen=5)
        self.varvtal_buffer = deque(maxlen=5)

        # max och min tilt för clamp
        self.min_tilt_percent = 500
        self.max_tilt_percent = 9500
        
        # initierar periodisk sändning av CAN-message för att undvika manual override
        self.current_tilt_percent = None
        self.running = False
        self.periodic_thread = None # Säkerställer att priodic_thread refereras från start

    # "clamp", hindrar programmet från att justera tilten utanför spannet [min_tilt max_tilt]
    # När CAN-message skickar tilt_percent, filteras värdet med följande logik:
    # Om tilt_percent > max_tilt_percent skickas max_tilt_percent
    # Om tilt_percent < min_tilt_percent skickas min_tilt percent
    def clamp_tilt_percent(self, tilt_percent):
        clamped = max(self.min_tilt_percent, min(tilt_percent, self.max_tilt_percent))
        if clamped == self.min_tilt_percent:
            print(f"Minsta vinkel nådd: {self.min_tilt_percent} %")
        elif clamped == self.max_tilt_percent:
            print(f"Största vinkel nådd: {self.max_tilt_percent} %")
        return clamped 

    # Tar ett genomsnitt av de 5 senaste lästa värdena.
    def smooth_data(self, buffer, new_value):
        buffer.append(new_value)
        return sum(buffer) / len(buffer)

    # Läser data från CANbus. Om ingen data läses inom 10s -> avbryts.
    def read_data(self, timeout=10.0):
        tilt_percent = vridmoment = varvtal = speed = None
        start_time = time.time()

    # Tar emot CANbus messages. exempel: "int.from_bytes(message.data[0:2]" läser byte 0 och 1 men inte 2.
    # Applicerar scaling och offsets.
        while time.time() - start_time < timeout:
            message = self.bus.recv(timeout=10.0)
            if message:
                if message.arbitration_id == 0x0CC6D4E1:  # Inverter Status
                    vridmoment = int.from_bytes(message.data[0:2], byteorder='little') * 0.0625 - 2048
                    varvtal = int.from_bytes(message.data[2:4], byteorder='little') * 0.5 - 16000
                elif message.arbitration_id == 0x14FF1140:  # GPS Status
                    speed = int.from_bytes(message.data[0:2], byteorder='little') * 0.01
                elif message.arbitration_id == 0x14FF0A50:  # Junction Box Status
                    tilt_percent = int.from_bytes(message.data[2:4], byteorder='little')

            if None not in [tilt_percent, vridmoment, varvtal, speed]:
                break

        if None in [tilt_percent, vridmoment, varvtal, speed]:
            print("Ingen data läst från CANbus!")

        # Applicerar filter
        filtered_vridmoment = self.smooth_data(self.vridmoment_buffer, vridmoment or 0)
        filtered_varvtal = self.smooth_data(self.varvtal_buffer, varvtal or 0)
        filtered_speed = self.smooth_data(self.speed_buffer, speed or 0)

        # P = M * 2pi*RPM / 60
        power = filtered_vridmoment * (2 * np.pi * filtered_varvtal / 60)

        return tilt_percent, power, filtered_speed

    # Skickar data till CANbus
    def send_tilt_percent(self, tilt_percent):
        # Filterar genom clamp
        tilt_percent = self.clamp_tilt_percent(int(tilt_percent))
        # Tilt to target (3)
        tilt_mode = 3
        # Gör om signalen till bytes 2 lång, little endian
        tilt_bytes = tilt_percent.to_bytes(2, byteorder='little')

        # Skapar och fyller 8 lång bytearray och fyller med nödvändig information
        data_bytes = bytearray(8)
        data_bytes[0] = tilt_mode
        data_bytes[2:4] = tilt_bytes # <--- Kolla [2:3] eller [2:4]

        # skapar CANbus message
        message = can.Message(
            arbitration_id=0x18FF1840,
            data=data_bytes,
            dlc=8,  # <-- Important
            is_extended_id=True
        )

        self.bus.send(message)

        print(f"Skickade tilt-procent: {tilt_percent / 100:.2f} % -- -- -- databyte = {data_bytes}")

    # Skapar separat tråd för att skicka periodiska meddelanden (100ms) och undvika manual override
    def start_periodic_sending(self, interval=0.1):
        self.running = True

        # skickar
        def sender():
            while self.running:
                if self.current_tilt_percent is not None:
                    self.send_tilt_percent(self.current_tilt_percent)
                time.sleep(interval)

        
        self.periodic_thread = threading.Thread(target=sender)
        self.periodic_thread.start()

    # stoppar separat tråd 
    def stop_periodic_sending(self):
        self.running = False
        # Länkar ihop trådarna efter stoppad
        if self.periodic_thread is not None and self.periodic_thread.is_alive():
            self.periodic_thread.join()
        self.periodic_thread = None

# Hanterar dokumentation. Skapar 
# nytt directory om inte redan finns.
# skapar csv-filer med data om tilt, speed, power, efficiency
# sparar filer med timestamp
class CSVLoggning:
    def __init__(self, directory="logs"):
        # skapar ny directory för dokumentation om det inte redan finns.
        # om dir redan finns skapar ingen ny. 
        self.directory = directory
        os.makedirs(self.directory, exist_ok=True)

    def save(self, filename_prefix, tilt_percent_history, speed_history, power_history, efficiency_history):
        # sparar csv filer med unika timestamps i filnamnen.
        filename = os.path.join(self.directory, f"{filename_prefix}_{int(time.time())}.csv")
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Iteration", "Tilt(%)", "Speed", "Power", "Efficiency"])
            for i in range(len(tilt_percent_history)):
                writer.writerow([i, tilt_percent_history[i]*0.01, speed_history[i], power_history[i], efficiency_history[i]])
        print(f"resultat sparade i {filename}")

# Trim-algoritm som optimerar tilt för bästa effektivitet
class TrimAlgorithm:
    def __init__(self, can_interface, csv_logger, step_size=100, tolerance=0.01, max_iterations=100):
        self.can = can_interface
        self.csv_logger = csv_logger
        self.step_size = step_size
        self.tolerance = tolerance
        self.max_iterations = max_iterations

        # Historik för att logga och plotta data
        self.tilt_percent_history = []
        self.speed_history = []
        self.power_history = []
        self.efficiency_history = []

        # Beräkna effektivitet som hastighet delat med effekt
    def measure_efficiency(self, speed, power):
        return speed / power if power else 0


    # Kör optimeringen
    def run(self):
        current_tilt, speed, power = self.can.read_data()
        current_tilt = self.can.clamp_tilt_percent(current_tilt)
        prev_efficiency = self.measure_efficiency(speed, power)
        self.can.current_tilt_percent = current_tilt # Krävs för att skicka periodiska msg

        self.tilt_percent_history.append(current_tilt)
        self.power_history.append(power)
        self.speed_history.append(speed)
        self.efficiency_history.append(prev_efficiency)

        step_direction = 1  # börja med att öka tilt
        step_size = self.step_size
        dircounter = 0
        for iteration in range(1, self.max_iterations + 1):
            # Föreslå nytt tilt-värde
            new_tilt = current_tilt + step_direction * step_size

            # Uppdatera tilt explicit via CAN
            self.can.current_tilt_percent = new_tilt

            # Vänta på stabilisering
            time.sleep(3)

            # Läs data efter tilt-justering
            _, power, speed = self.can.read_data()

            # Beräkna ny effektivitet
            new_efficiency = self.measure_efficiency(speed,power)
            error = new_efficiency - prev_efficiency

            # Spara historik
            self.tilt_percent_history.append(new_tilt)
            self.power_history.append(power)
            self.speed_history.append(speed)
            self.efficiency_history.append(new_efficiency)

            # Kontrollera tolerans
            if abs(error) < self.tolerance:
                print(f"Optimal tilt uppnådd: {new_tilt / 100:.2f}%")
                break
            
            # Justera riktning och stegstorlek
            if error > 0:
                current_tilt = new_tilt
                prev_efficiency = new_efficiency
            else:
                step_direction *= -1
                dircounter += 1
                if dircounter > 3:
                    step_size *= 0.5
     

            # Kort paus mellan iterationer
            time.sleep(0.5)
        else:
            print("Maximalt antal iterationer nått.")

        self.csv_logger.save("trim_results", self.tilt_percent_history, self.speed_history, self.power_history, self.efficiency_history)

def main():
    can_interface = CANInterface()
    csv_logger = CSVLoggning()
    first_tilt_percent, _, _ = can_interface.read_data()
    can_interface.current_tilt_percent = first_tilt_percent
    can_interface.start_periodic_sending()

    optimizer = TrimAlgorithm(can_interface,csv_logger)
    optimizer.run()

    can_interface.stop_periodic_sending()


if __name__ == "__main__":
    main()  


#       _
#      /|\
#     /_|_\
#   ____|____
#   \_o_o_o_/
#~~~~~~~ | ~~~~~~~~
#________t_________ 
