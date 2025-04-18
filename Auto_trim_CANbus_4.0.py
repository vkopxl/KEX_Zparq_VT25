# Auto-Trim Version 3.5 
# Baserad på 3.0_SIM verifierad på land.

# Gjort i denna verion
# - Promptar input frör val av algorithm
# - Gradient Descent tillagd
# - Lägga till så att data kan sparas även fast for loopen inte breakar av sig själv. "Press enter to end" --> sparar all tidigare data. 
# - Parametrar lagras i CSV för bättre vetenskaplig dokumentation
# - La till READ ONLY MODE för att läsa data från canbus utan att initiera algoritm

# Till nästa version:
# - Separera på vanilla och momentum gradient ascent, gör en egen klass för momentum

# Till ännu senare versioner:
# Spara tidigare data i Luts för initial gissning. 


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
        self.max_tilt_percent = 6000
        
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
                    varvtal = int.from_bytes(message.data[2:4], byteorder='little') * -1 * 0.5 - 16000  # Negativ RPM kan komma att ändras
                elif message.arbitration_id == 0x14FF1140:  # GPS Status
                    speed = int.from_bytes(message.data[0:2], byteorder='little') * 0.01
                elif message.arbitration_id == 0x14FF0A50:  # Junction Box Status
                    tilt_percent = int.from_bytes(message.data[2:4], byteorder='little')
            
            # printar mottagen data, kommentera bort om det blir jobbigt.
            # print(f"Received DATA: vridmoment: {vridmoment:.2f} -- varvtal: {varvtal:.2f} -- speed: {speed:.2f} -- tilt%: {tilt_percent:.2f}")

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

        return tilt_percent, power, filtered_speed, filtered_varvtal, filtered_vridmoment, speed, vridmoment, varvtal

    # Skickar data till CANbus
    def send_tilt_percent(self, tilt_percent, meddela = True):
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
        if meddela:
            print(f"Skickade tilt-procent: {tilt_percent / 100:.2f} % -- -- -- databyte = {data_bytes:.2f}")

    # Skapar separat tråd för att skicka periodiska meddelanden (100ms) och undvika manual override
    def start_periodic_sending(self, interval=0.1):
        self.running = True

        # skickar
        def sender():
            while self.running:
                if self.current_tilt_percent is not None:
                    self.send_tilt_percent(self.current_tilt_percent, meddela=False)
                time.sleep(interval)

        
        self.periodic_thread = threading.Thread(target=sender)
        self.periodic_thread.start()

    # Read only mode printar mottagen ofiltrerad data i terminalen, alternativet ges även att spara indata i ett CSV
    def read_only_mode(self, csv_logger = None, log = False):
        
        tilt_history = []
        power_history = []
        speed_history = []
        efficiency_history = []
        stepsize_history = []

        try:
            while True:
                tilt_percent, power, speed, filtered_varvtal, filtered_vridmoment, varvtal, vridmoment = self.read_data()

                if None not in [tilt_percent, power, speed]:
                    efficiency = speed / power if power else 0
                    
                    #lägg till buffrad/obuffrad speed
                    print(f"Tilt: {tilt_percent / 100:.2f}% | "
                        f"Hastighet: {speed:.2f} km/h | "
                        f"Effekt: {power:.2f} W | "
                        f"Vridmoment: {vridmoment:.2f} Nm |"
                        f"Buffrat Vridmoment: {filtered_vridmoment:.2f} Nm | "
                        f"Varvtal: {varvtal:.2f} RPM | "
                        f"Buffrat Varvtal: {filtered_varvtal:.2f} RPM | "
                        f"Effektivitet: {efficiency:.5f}")
                    
                if log:
                    tilt_history.append(tilt_percent)
                    power_history.append(power)
                    speed_history.append(speed)
                    efficiency_history.append(efficiency)
                    stepsize_history.append()

                else:
                    print("Väntar på komplett data...")

                time.sleep(0.5)

        except KeyboardInterrupt:
            print("READ MODE avslutat")
        
        if log and csv_logger:
            parameters_used = {
                "Algorithm": "READ_MODE_LOGGING",
                "Kommentar": "Rådata loggning utan optimering"
            }

            csv_logger.save(
                "ReadMode_results",
                parameters_used,
                tilt_history,
                speed_history,
                power_history,
                efficiency_history,
                stepsize_history
            )

    # stoppar separat tråd 
    def stop_periodic_sending(self):
        self.running = False
        # Länkar ihop trådarna efter stoppad
        if self.periodic_thread is not None and self.periodic_thread.is_alive():
            self.periodic_thread.join()
        self.periodic_thread = None

# Hanterar dokumentation. Skapar 
# nytt directory om inte redan finns.
# skapar csv-filer med data om använda parametrar, tilt, speed, power, efficiency
# sparar filer med timestamp
class CSVLoggning:
    def __init__(self, directory="logs"):
        # skapar ny directory för dokumentation om det inte redan finns.
        # om dir redan finns skapar ingen ny. 
        self.directory = directory
        os.makedirs(self.directory, exist_ok=True)

    def save(self, filename_prefix, parameters_used, tilt_percent_history, speed_history, power_history, efficiency_history, stepsize_history):
        # sparar csv filer med unika timestamps i filnamnen.
        filename = os.path.join(self.directory, f"{filename_prefix}_{int(time.time())}.csv")
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)

            # fyller parametrar
            writer.writerow(["parameter, value"])
            for key, value in parameters_used.items():
                writer.writerow([key, value])

            # tom rad
            writer.writerow([])

            # Skriver titel
            writer.writerow(["Iteration", "Tilt(%)", "Speed", "Power", "Efficiency"])

            #Fyller log-fil
            for i in range(len(tilt_percent_history)):
                writer.writerow([i, tilt_percent_history[i]*0.01, speed_history[i], power_history[i], efficiency_history[i], stepsize_history[i]])
        print(f"resultat sparade i {filename}")

# Trim-algoritm som optimerar tilt för bästa effektivitet (Gradient Ascent), har alternativet momentum gradient ascent. 
class GradientAscent:
    def __init__(self, can_interface, csv_logger, step_size=100, tolerance=0.01, max_iterations=100, alpha = 5000, beta = 0.95, v=0, use_momentum=False):
        self.can = can_interface
        self.csv_logger = csv_logger
        self.step_size = step_size
        self.tolerance = tolerance
        self.max_iterations = max_iterations
        self.alpha = alpha
        self.beta = beta
        self.v = v
        self.use_momentum = use_momentum

        # Historik för att logga och plotta data
        self.tilt_percent_history = []
        self.speed_history = []
        self.power_history = []
        self.efficiency_history = []
        self.stepsize_history =[]

        # Beräkna effektivitet som hastighet delat med effekt
    def measure_efficiency(self, speed, power):
        return speed / power if power else 0

    # Kör optimeringen
    def run(self):
        try:
            prev_tilt, speed, power = self.can.read_data()
            prev_tilt = self.can.clamp_tilt_percent(prev_tilt)
            prev_efficiency = self.measure_efficiency(speed, power)
            self.can.current_tilt_percent = prev_tilt # Krävs för att skicka periodiska msg
            alpha = self.alpha
            beta = self.beta

            self.tilt_percent_history.append(prev_tilt)
            self.power_history.append(power)
            self.speed_history.append(speed)
            self.efficiency_history.append(prev_efficiency)
            self.stepsize_history.append(step_size)

            step_size = self.step_size
            v = self.v
            
            for iteration in range(1, self.max_iterations + 1):

                new_tilt = prev_tilt - step_size
                if self.use_momentum:
                    new_tilt = prev_tilt + v
                else:
                    new_tilt = prev_tilt - step_size

                # Uppdatera tilt explicit via CAN
                self.can.current_tilt_percent = new_tilt

                # Vänta på stabilisering
                time.sleep(3)

                # Läs data efter tilt-justering
                _, power, speed = self.can.read_data()

                # Beräkna ny effektivitet
                new_efficiency = self.measure_efficiency(speed,power)
                error = new_efficiency - prev_efficiency

                # Estimera gradient (första derivatan efter det är envariabel)
                # gradient = (new_efficiency - prev_efficiency) / (new_tilt - prev_tilt)
                gradient = error / step_size

                print(f"Iteration {iteration}")
                print(f"Current Tilt: {new_tilt /100:.2f}%, Efficiency: {prev_efficiency:.2f}")
                print(f"New Tilt: {new_tilt /100:.2f}%, Efficiency: {new_efficiency:.2f}")

                # Spara historik
                self.tilt_percent_history.append(new_tilt)
                self.power_history.append(power)
                self.speed_history.append(speed)
                self.efficiency_history.append(new_efficiency)
                self.stepsize_history.append(step_size)

                # Kontrollera tolerans
                if abs(error) < self.tolerance:
                    print(f"Optimal tilt uppnådd: {new_tilt / 100:.2f}%")
                    break
                
                # Förbereda nästa iteration
                prev_efficiency = new_efficiency
                prev_tilt = new_tilt 

                if self.use_momentum:
                    v = beta*v - (alpha*gradient)
                    step_size = abs(v) # för logging
                else:
                    step_size = (alpha*gradient)

                # Kort paus mellan iterationer
                time.sleep(0.5)

            else:
                print("Maximalt antal iterationer nått.")
        
        except KeyboardInterrupt:
            print("Optimering avbrutet av användaren")

        finally: 
            parameters_used = {
                    "Algorithm": "Gradient",
                    "Tolerance": self.tolerance,
                    "Max Iterations": self.max_iterations,
                    "Alpha": self.alpha,
                    "Beta": self.beta,
                    "Momentum": self.use_momentum
                }
            
            self.csv_logger.save(
                "Gradient_results",
                parameters_used,
                self.tilt_percent_history,
                self.speed_history,
                self.power_history,
                self.efficiency_history,
                self.stepsize_history
            )
        
# Trim-algoritm som optimerar tilt för bästa effektivitet (Hill Climbing)
class HillClimbing:
    def __init__(self, can_interface, csv_logger, parameters_used, step_size=100, tolerance=0.01, max_iterations=100):
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
        self.stepsize_history =[]

        # Beräkna effektivitet som hastighet delat med effekt
    def measure_efficiency(self, speed, power):
        return speed / power if power else 0

    # Kör optimeringen
    def run(self):
        try:
            current_tilt, speed, power = self.can.read_data()
            current_tilt = self.can.clamp_tilt_percent(current_tilt)
            prev_efficiency = self.measure_efficiency(speed, power)
            self.can.current_tilt_percent = current_tilt # Krävs för att skicka periodiska msg

            self.tilt_percent_history.append(current_tilt)
            self.power_history.append(power)
            self.speed_history.append(speed)
            self.efficiency_history.append(prev_efficiency)
            self.stepsize_history.append(step_size)

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
                self.stepsize_history.append(step_size)

                # Kontrollera tolerans
                if abs(error) < self.tolerance:
                    print(f"Optimal tilt uppnådd: {new_tilt / 100:.2f}%")
                    break
                
                # Justera riktning och steglängd
                if error > 0:
                    current_tilt = new_tilt
                    prev_efficiency = new_efficiency
                else:
                    step_direction *= -1
                    dircounter += 1
                    # Hindrar algoritmen från att minska steglängden för tidigt
                    if dircounter > 3:
                        step_size *= 0.5
        

                # Kort paus mellan iterationer
                time.sleep(0.5)
            else:
                print("Maximalt antal iterationer nått.")
                
        except KeyboardInterrupt:
            print("Optimering avbröts av användaren")

        finally:    
            parameters_used = {
                    "Algorithm": "Hillclimbing",
                    "Step Size": self.step_size,
                    "Tolerance": self.tolerance,
                    "Max Iterations": self.max_iterations
                }
            
            self.csv_logger.save(
                "Hillclimbing_results",
                parameters_used,
                self.tilt_percent_history,
                self.speed_history,
                self.power_history,
                self.efficiency_history,
                self.stepsize_history
                )

def main():
    print("Välj algorithm: '1': Hillclimbing, '2': Vanilla Gradient Ascent, '3': Momentum Gradient Ascent, '4': READ ONLY Mode, '5' READ AND WRITE Mode")
    val = input()
    can_interface = CANInterface()
    csv_logger = CSVLoggning()

    can_interface.start_periodic_sending()


    try:
        if val == '1':
            print("Algoritm vald: HILL CLIMBING")
            optimizer = HillClimbing(can_interface, csv_logger)
        elif val == '2':
            print("Algoritm vald: Vanilla GRADIENT ASCENT")
            optimizer = GradientAscent(can_interface, csv_logger)
        elif val == '3':
            print("Algoritm vald: Momentum GRADIENT ASCENT")
            optimizer = GradientAscent(can_interface, csv_logger, use_momentum=True)
        elif val =='4':
            print("READ ONLY MODE")
            print("Visar data från CAN-bus")
            can_interface.read_only_mode()
            return
        elif val == '5':
            print("READ AND WRITE MODE")
            print("Visar och sparar data från CAN-bus")
            can_interface.read_mode(csv_logger, log=True)

        optimizer.run()

    except KeyboardInterrupt:
        print("Avbröts manuellt av användaren.")
    finally:
        can_interface.stop_periodic_sending()
        print("Avslutar CAN-kommunikation.")



if __name__ == "__main__":
    main()  


#         _
#        /|\
#       /_|_\
#     ____|____
#     \_o_o_o_/
#~~~~~~~~~~ | ~~~~~~~~
#___________t_________ 
