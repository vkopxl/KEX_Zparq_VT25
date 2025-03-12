# Auto-Trim Version 2.5_SIM
# Den här varianten av programmet är till för att testa algorithmen på land, utan data om speed, vridmotstånd, rpm.
# Programemt imsulerar istället en optimal tilt_percent. 


#Importera nödvändiga bibliotek
import can  # För att kommunicera med CAN-bussen
import time  # För tidsstyrning och pauser
import threading  # För att hantera parallella trådar
import numpy as np  # För matematiska beräkningar
import matplotlib.pyplot as plt  # För att rita grafer
from collections import deque  # För att skapa ringbuffertar för medelvärdesutjämning
import csv # För att spara dokumentation i csv filer
import os # Filhantering för dokumentation



# Klass som hanterar CAN-kommunikation
class CANInterface:
    def __init__(self, channel='PCAN_USBBUS1', bitrate=1000000):
        # Initiera anslutningen till CAN-bussen
        self.bus = can.interface.Bus(bustype='pcan', channel=channel, bitrate=bitrate)
        
        # Buffertar för att släta ut inkommande data
        self.speed_buffer = deque(maxlen=5)
        self.vridmoment_buffer = deque(maxlen=5)
        self.varvtal_buffer = deque(maxlen=5)

        # Begränsningar för tilt-procenten
        self.min_tilt_percent = 500
        self.max_tilt_percent = 9500
        
        # Nuvarande tilt-vinkel (procent)
        self.current_tilt_percent = None
        self.running = False  # Flagga för att hålla igång periodisk sändning

    # Säkerställ att tilt-procenten hålls inom tillåtna gränser
    def clamp_tilt_percent(self, tilt_percent):
        clamped = max(self.min_tilt_percent, min(tilt_percent, self.max_tilt_percent))
        if clamped == self.min_tilt_percent:
            print(f"Minsta vinkel nådd: {self.min_tilt_percent} %")
        elif clamped == self.max_tilt_percent:
            print(f"Största vinkel nådd: {self.max_tilt_percent} %")
        return clamped

    # Släta ut mätdata med hjälp av medelvärde av de senaste värdena
    def smooth_data(self, buffer, new_value):
        buffer.append(new_value)
        return sum(buffer) / len(buffer)

    # Läs data från CAN-bussen inom en tidsgräns
    def read_data(self, timeout=10.0):
        tilt_percent = vridmoment = varvtal = speed = None
        start_time = time.time()

        # Försök läsa all nödvändig data tills timeout nås
        while time.time() - start_time < timeout:
            message = self.bus.recv(timeout=1.0)
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

        # Släta ut värdena
        filtered_vridmoment = self.smooth_data(self.vridmoment_buffer, vridmoment or 0)
        filtered_varvtal = self.smooth_data(self.varvtal_buffer, varvtal or 0)
        filtered_speed = self.smooth_data(self.speed_buffer, speed or 0)

        # Beräkna effekt (power) från vridmoment och varvtal
        power = filtered_vridmoment * (2 * np.pi * filtered_varvtal / 60)

        return tilt_percent, power, filtered_speed

    # Skicka tilt-procent till CAN-bussen
    def send_tilt_percent(self, tilt_percent):
        tilt_percent = self.clamp_tilt_percent(int(tilt_percent))
        tilt_mode = 3
        tilt_mode = tilt_mode.to_bytes(1, byteorder='little')
        tilt_bytes = tilt_percent.to_bytes(2, byteorder='little')


        data_bytes = bytearray(4)
        data_bytes[0] = tilt_mode 
        data_bytes[2:3] = tilt_bytes # Target

        message = can.Message(
            arbitration_id=0x18FF1840,
            data=data_bytes,
            is_extended_id=True
        )

        self.bus.send(message)
        print(f"Skickade tilt-procent: {tilt_percent / 100:.2f} %")

    # Starta en tråd som periodiskt skickar tilt-värdet
    def start_periodic_sending(self, interval=0.1):
        self.running = True

        def sender():
            while self.running:
                if self.current_tilt_percent is not None:
                    self.send_tilt_percent(self.current_tilt_percent)
                time.sleep(interval)

        thread = threading.Thread(target=sender)
        thread.daemon = True
        thread.start()

    # Stoppa den periodiska sändningen
    def stop_periodic_sending(self):
        self.running = False

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
    def __init__(self, can_interface,  csv_logger, step_size=1.0, tolerance=1, max_iterations=1000):
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
    def simulate_efficiency(self, tilt_percent):
        optimal_tilt = 4000
        return (optimal_tilt - tilt_percent)**2 + 10

    # Kör optimeringen
    def run(self):
        current_tilt, power, speed = self.can.read_data()
        self.can.current_tilt_percent = current_tilt
        prev_efficiency = self.simulate_efficiency(current_tilt)

        self.tilt_percent_history.append(current_tilt)
        self.power_history.append(power)
        self.speed_history.append(speed)
        self.efficiency_history.append(prev_efficiency)

        for iteration in range(1, self.max_iterations + 1):
            current_tilt, power, speed = self.can.read_data()
            new_efficiency = self.simulate_efficiency(current_tilt)
            error = new_efficiency - prev_efficiency

            self.tilt_percent_history.append(current_tilt)
            self.power_history.append(power)
            self.speed_history.append(speed)
            self.efficiency_history.append(new_efficiency)

            if abs(error) < self.tolerance:
                print(f"Optimal vinkel hittad: {current_tilt:.2f} % efter {iteration} iterationer")
                break

            if error > 0:
                new_tilt = current_tilt + self.step_size
            else:
                self.step_size = -0.5 * self.step_size
                new_tilt = current_tilt + self.step_size

            self.can.current_tilt_percent = new_tilt
            prev_efficiency = new_efficiency
            time.sleep(abs(3))
        else:
            print("Maximalt antal iterationer uppnått.")

        self.csv_logger.save("trim_results", self.tilt_percent_history, self.power_history, self.speed_history, self.efficiency_history)
        

def main():
    can_interface = CANInterface()
    csv_logger = CSVLoggning()
    first_tilt_percent, _, _ = can_interface.read_data()
    can_interface.current_tilt_percent = first_tilt_percent
    can_interface.start_periodic_sending()

    optimizer = TrimAlgorithm(can_interface, csv_logger)
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