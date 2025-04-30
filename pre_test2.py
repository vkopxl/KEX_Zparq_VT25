# -- Auto-Trim Version 5.0 --

# Nytt:
# - Läser data i thread
# - OFÄRDIG OTESTAD LUTs
# - Gradient momentum
# - EMA Signalfilter
# - Timestamp för varje iteration (förplottar med tid på x-axel)

#Importera nödvändiga bibliotek
import can  # För att kommunicera med CAN-bussen
import time  # För tidsstyrning och pauser
import threading  # För att hantera parallella trådar
import numpy as np  # För matematiska beräkningar
import csv # För att spara dokumentation i csv filer
import os # Filhantering för dokumentation  
import serial # hanterar interaktion med arduino
import random # genererar slumpmässiga siffror
#from scipy.spatial import KDTree # hämtar kdtree ur scipy för att hitta närmsta grannar i LUTs

class CANInterface:
# Klass som hanterar CAN-kommunikation
    def __init__(self, channel='PCAN_USBBUS1', bitrate=1000000):
        self.bus = can.interface.Bus(interface='pcan', channel=channel, bitrate=bitrate)
        
        # Buffrar input data och tar genomsnitt
        self.speed_filter = SignalFilter()
        self.vridmoment_filter = SignalFilter()
        self.varvtal_filter = SignalFilter()

        # max och min tilt för clamp
        self.min_tilt_percent = 200
        self.max_tilt_percent = 9000
        
        # initierar periodisk sändning av CAN-message för att undvika manual override
        self.current_tilt_percent = None
        self.running = False
        self.periodic_thread = None # Säkerställer att priodic_thread refereras från start

        # ny tråd för att läsa canbus i bakgrunden
        self.read_thread = None
        self.read_running = False
        self.latest_data = {
            "tilt_percent": None,
            "speed": None,
            "vridmoment": None,
            "varvtal": None
        }
        self.data_lock = threading.Lock()

    def clamp_tilt_percent(self, tilt_percent, periodic = False):
    # "clamp", hindrar programmet från att justera tilten utanför spannet [min_tilt max_tilt]
    # När CAN-message skickar tilt_percent, filteras värdet med följande logik:
    # Om tilt_percent > max_tilt_percent skickas max_tilt_percent
    # Om tilt_percent < min_tilt_percent skickas min_tilt percent

        clamped = max(self.min_tilt_percent, min(tilt_percent, self.max_tilt_percent))
        if clamped == self.min_tilt_percent and periodic == False:
            print(f"Minsta vinkel nådd: {self.min_tilt_percent*0.01} %")
        elif clamped == self.max_tilt_percent and periodic == False:
            print(f"Största vinkel nådd: {self.max_tilt_percent*0.01} %")
        return clamped

    def start_reading(self, fake = False):
        self.read_running = True

        if fake == False:
            def reader():
                while self.read_running:
            # Tar emot CANbus messages. exempel: "int.from_bytes(message.data[0:2]" läser byte 0 och 1 men inte 2.
            # Applicerar scaling och offsets.
                    message = self.bus.recv(timeout=10.0)
                    if message:
                        if message.arbitration_id == 0x0CC6D4E1:  # Inverter Status
                            self.latest_data["vridmoment"] = (int.from_bytes(message.data[0:2], byteorder='little') * 0.0625 - 2048) * -1
                            self.latest_data["varvtal"] = (int.from_bytes(message.data[2:4], byteorder='little') * 0.5 - 16000) * -1  # Negativ RPM kan komma att ändras
                        elif message.arbitration_id == 0x14FF1140:  # GPS Status
                            self.latest_data["speed"] = int.from_bytes(message.data[0:2], byteorder='little') * 0.01
                        elif message.arbitration_id == 0x14FF0A50:  # Junction Box Status
                            self.latest_data["tilt_percent"] = int.from_bytes(message.data[2:4], byteorder='little')

        elif fake == True:
            def fake_reader():
                while self.read_running:
                    message = self.bus.recv(timeout=10.0)
                    if message:
                        if message.arbitration_id == 0x14FF0A50:
                            self.latest_data["tilt_percent"] = int.from_bytes(message.data[2:4], byteorder='little')
                            self.latest_data["speed"] =  int.from_bytes(message.data[2:4], byteorder='little') - 1000
                        else:
                            self.latest_data["vridmoment"] = 10 * random.randint(1, 50)
                            self.latest_data["varvtal"] = 10 * random.randint(51, 100)

        if fake == False:    
            self.read_thread = threading.Thread(target=reader)
        elif fake == True:
            self.read_thread = threading.Thread(target=fake_reader)
        self.read_thread.start()            

    def stop_read(self):
        self.read_running = False
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join()

    def get_latest_data(self):
        with self.data_lock:
            tilt_percent = self.latest_data["tilt_percent"]
            vridmoment = self.latest_data["vridmoment"]
            varvtal = self.latest_data["varvtal"]
            speed = self.latest_data["speed"]

            if None in [tilt_percent, vridmoment, varvtal, speed]:
                print("Ingen data läst från CANbus!")
                return None, None, None, None, None, None, None, None

            # Applicerar filter
            filtered_vridmoment = self.vridmoment_filter.update(vridmoment or 0)
            filtered_varvtal = self.varvtal_filter.update(varvtal or 0)
            filtered_speed = self.speed_filter.update(speed or 0)

            # P = M * 2pi*RPM / 60
            power = filtered_vridmoment * (2 * np.pi * filtered_varvtal / 60)

            return tilt_percent, power, filtered_speed, filtered_varvtal, filtered_vridmoment, speed, vridmoment, varvtal

    # väntar in tillräckligt mycket data för att starta pålitlig trim optimering
    def wait_for_valid_data(self, timeout=10.0):
        start_time = time.time()
        while time.time() - start_time < timeout:
            _, power, speed, *_ = self.get_latest_data()
            if None not in [power, speed] and power > 0 and speed > 0:
                return True
            print("Väntar på giltig data från CAN...")
            time.sleep(0.5)
        print("Timeout: Kunde inte hämta giltig data.")
        return False

    # Skickar data till CANbus
    def send_tilt_percent(self, tilt_percent, meddela = True, periodic = False):
        # Filterar genom clamp
        tilt_percent = self.clamp_tilt_percent(int(tilt_percent), periodic=periodic)
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

    def start_periodic_sending(self, interval=0.1):
    # Skapar separat tråd för att skicka periodiska meddelanden (100ms) och undvika manual override
        self.running = True

        # skickar
        def sender():
            while self.running:
                if self.current_tilt_percent is not None:
                    self.send_tilt_percent(self.current_tilt_percent, meddela=False, periodic = True)
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

    # Read only mode printar mottagen ofiltrerad data i terminalen, alternativet ges även att spara indata i en CSV-fil
    def read_only_mode(self, csv_logger = None, log = False):
        
        tilt_history = []
        power_history = []
        speed_history = []
        efficiency_history = []
        timestamps = []

        start_time = time.time()

        try:
            while True:
                tilt_percent, power, filtered_speed, filtered_varvtal, filtered_vridmoment, speed, vridmoment, varvtal = self.get_latest_data()

                if None not in [tilt_percent, power, speed]:
                    efficiency = speed / power if power else 0
                    
                    #lägg till buffrad/obuffrad speed
                    print(f"Tilt: {tilt_percent / 100:.2f}% | "
                        f"Hastighet: {speed:.2f} m/s | " f"Buffrat hastighet: {filtered_speed} m/s | "
                        f"Effekt: {power:.2f} W | "
                        f"Vridmoment: {vridmoment:.2f} Nm |" f"Buffrat Vridmoment: {filtered_vridmoment:.2f} Nm | "
                        f"Varvtal: {varvtal:.2f} RPM | " f"Buffrat Varvtal: {filtered_varvtal:.2f} RPM | "
                        f"Effektivitet: {efficiency*100:.5f}"
                        f"Filter delta Hastighet: {abs(filtered_speed - speed)}"
                        f"Filter delta Vridmoment: {abs(filtered_vridmoment - vridmoment)}"
                        f"Filter delta Varvtal: {abs(filtered_varvtal - varvtal)}"
                        )

                    
                if log:
                    tilt_history.append(tilt_percent)
                    power_history.append(power)
                    speed_history.append(speed)
                    efficiency_history.append(efficiency)
                    timestamps.append(time.time() - start_time)

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
                timestamps
            )

class ControlPanel:
# hanterar interaktion med arduino
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600, message_callback = None):

        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.running = True
        self.message_callback = message_callback
        self.can = None

        self.thread = threading.Thread(target=self.read_loop)
        self.efficiency_thread = threading.Thread(target=self.send_efficiency_loop)

        self.thread.start()
        self.efficiency_thread.start()

    def read_loop(self):
        while self.running:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8').strip()
                print(f"[Arduino {line}]")
                self.handle_message(line)

    # uppdatera def handle message så att den kan interagera med resten av programmet sen
    def handle_message(self, message):
        if message == "STARTA":
            print("Arduino requested START")
            # Optimizer start lokig här

        elif message == "AVBRYT":
            # Om användaren trycker AVBRYT
            print("[SIMULATOR] Optimering avbruten.")

    def send_message(self,msg):
        self.ser.write((msg + "\n").encode())

    def send_efficiency_loop(self):
        while self.running:
            if self.can is not None:
                _, power, filtered_speed, *_ = self.can.get_latest_data()
                if power > 0 and filtered_speed > 0:
                    effektivitet = filtered_speed / power
                    effektivitet = effektivitet * 100000  # Skala upp
                    effektivitet = int(effektivitet)      # Ta bort decimaler
                    # Begränsa till max 9999
                    effektivitet = min(effektivitet, 9999)
                    effektivitet = str(effektivitet)
                    self.send_effektivitet(effektivitet)
                time.sleep(0.5)

    def send_effektivitet(self,effektivitet):
         self.ser.write((f"{effektivitet}\n").encode())

    def stop(self):
        self.running = False
        self.thread.join()
        self.efficiency_thread.join()
        self.ser.close()

class SignalFilter: 
# Exponential Moving Average (EMA) filter för att hantera brus
    def __init__(self, base_alpha=0.05, max_alpha=0.2):
        self.base_alpha = base_alpha
        self.max_alpha = max_alpha
        self.previous = None

    def update(self, new_value):
        if self.previous is None:
            self.previous = new_value
            return new_value
        
        delta = abs(new_value - self.previous)
        alpha = min(self.max_alpha, self.base_alpha + delta * 0.1)
        smoothed = alpha * new_value + (1 - alpha) * self.previous
        self.previous = smoothed
        return smoothed

class CSVLoggning:
# Hanterar dokumentation. Skapar nytt directory om inte redan finns.
# skapar csv-filer med data om använda parametrar, tilt, speed, power, efficiency
# sparar filer med timestamp
    def __init__(self, directory="logs"):
        # skapar ny directory för dokumentation om det inte redan finns.
        # om dir redan finns skapar ingen ny. 
        self.directory = directory
        os.makedirs(self.directory, exist_ok=True)

    def save(self, filename_prefix, parameters_used, tilt_percent_history, speed_history, power_history, efficiency_history, timestamps):
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
            writer.writerow(["Timestamp", "Iteration", "Tilt(%)", "Speed", "Power", "Efficiency"])

            #Fyller log-fil
            for i in range(len(tilt_percent_history)):
                writer.writerow([timestamps[i],
                                i,
                                tilt_percent_history[i]*0.01,
                                speed_history[i],
                                power_history[i],
                                efficiency_history[i]])
        print(f"resultat sparade i {filename}")

class GradientAscent:
# Trim-algoritm som optimerar tilt för bästa effektivitet (Gradient Ascent), har alternativet momentum gradient ascent.
    def __init__(self, can_interface, csv_logger, step_size=100, tolerance=0.00002, max_iterations=100, alpha=100, beta=0.9, v=0, use_momentum=False, fake = False):
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
        self.timestamps = []
        self.start_time = time.time()

        # Beräkna effektivitet som hastighet delat med effekt
    def measure_efficiency(self, speed, power):
        return speed / power if power else 0

    # Kör optimeringen
    def run(self):
        # Vänta på initial giltig data innan optimering
        if not self.can.wait_for_valid_data(timeout=10):
            print("Ingen giltig data tillgänglig – avbryter optimering.")
            return

        try:
            prev_tilt, power, filtered_speed, _, _, _, _, _, = self.can.get_latest_data()
            prev_tilt = self.can.clamp_tilt_percent(prev_tilt)
            prev_efficiency = self.measure_efficiency(filtered_speed, power)
            self.can.current_tilt_percent = prev_tilt # Krävs för att skicka periodiska msg
            alpha = self.alpha
            beta = self.beta

            self.tilt_percent_history.append(prev_tilt)
            self.power_history.append(power)
            self.speed_history.append(filtered_speed)
            self.efficiency_history.append(prev_efficiency)
            self.timestamps.append(time.time() - self.start_time)

        

            step_size = self.step_size
            v = self.v
            
            for iteration in range(1, self.max_iterations + 1):

                if self.use_momentum:
                    new_tilt = prev_tilt + v
                else:
                    new_tilt = prev_tilt - step_size

                # Uppdatera tilt explicit via CAN
                self.can.current_tilt_percent = new_tilt

                # Vänta på stabilisering
                time.sleep(3)

                # Läs data efter tilt-justering
                _, power, filtered_speed, _, _, _, _, _, = self.can.get_latest_data()

                # Beräkna ny effektivitet
                new_efficiency = self.measure_efficiency(filtered_speed,power)
                error = new_efficiency - prev_efficiency
                
                # Estimera gradient (första derivatan efter det är envariabel)
                # gradient = (new_efficiency - prev_efficiency) / (new_tilt - prev_tilt)

                print(f"Iteration {iteration}")
                #print(f"Current Tilt: {new_tilt /100:.2f}%, Efficiency: {prev_efficiency:.2f}")
                #print(f"New Tilt: {new_tilt /100:.2f}%, Efficiency: {new_efficiency:.2f}")
                print(f"Tiltjustering (procentenheter): {(new_tilt-prev_tilt)/100:.2f}%")
                print(f"Förändring i effektivitet (error): {error:.10f}")

                # Spara historik
                self.tilt_percent_history.append(new_tilt)
                self.power_history.append(power)
                self.speed_history.append(filtered_speed)
                self.efficiency_history.append(new_efficiency)
                self.timestamps.append(time.time() - self.start_time)
                
                # Kontrollera tolerans
                if abs(error) < self.tolerance:
                    print(f"Optimal tilt uppnådd: {new_tilt / 100:.2f}%")
                    break
                
                # Förbereda nästa iteration
                prev_efficiency = new_efficiency
                prev_tilt = new_tilt 

                if self.use_momentum:
                    v = beta*v - (alpha*error/step_size)
                    step_size = abs(v) # för logging
                else:
                    step_size = (alpha*error/step_size)

                # Kort paus mellan iterationer
                time.sleep(1.5)

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
                self.timestamps
            )

class HillClimbing:
# Trim-algoritm som optimerar tilt för bästa effektivitet (Hill Climbing)
    def __init__(self, can_interface, csv_logger, step_size=100, tolerance=0.00002, max_iterations=100, fake = False):
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
        self.timestamps = []
        self.start_time = time.time()

        # Beräkna effektivitet som hastighet delat med effekt
    def measure_efficiency(self, speed, power):
        return speed / power if power else 0

    # Kör optimeringen
    def run(self):
        # Vänta på initial giltig data innan optimering
        if not self.can.wait_for_valid_data(timeout=10):
            print("Ingen giltig data tillgänglig – avbryter optimering.")
            return
        
        try:
            current_tilt, power, filtered_speed, _, _, _, _, _, = self.can.get_latest_data()
            current_tilt = self.can.clamp_tilt_percent(current_tilt)
            prev_efficiency = self.measure_efficiency(filtered_speed, power)
            self.can.current_tilt_percent = current_tilt # Krävs för att skicka periodiska msg

            self.tilt_percent_history.append(current_tilt)
            self.power_history.append(power)
            self.speed_history.append(filtered_speed)
            self.efficiency_history.append(prev_efficiency)
            self.timestamps.append(time.time() - self.start_time)

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
                _, power, filtered_speed, _, _, _, _, _, = self.can.get_latest_data()

                # Beräkna ny effektivitet
                new_efficiency = self.measure_efficiency(filtered_speed,power)
                error = new_efficiency - prev_efficiency

                print(f"Iteration {iteration}")
                #print(f"Current Tilt: {new_tilt /100:.2f}%, Efficiency: {prev_efficiency:.2f}")
                #print(f"New Tilt: {new_tilt /100:.2f}%, Efficiency: {new_efficiency:.2f}")
                print(f"Tiltjustering (procentenheter): {(new_tilt-current_tilt)/100:.2f}%")
                print(f"Förändring i effektivitet (error): {error:.10f}")

                # Spara historik
                self.tilt_percent_history.append(new_tilt)
                self.power_history.append(power)
                self.speed_history.append(filtered_speed)
                self.efficiency_history.append(new_efficiency)
                self.timestamps.append(time.time() - self.start_time)

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
                time.sleep(1.5)
            else:
                print("Maximalt antal iterationer nått.")
                
        except KeyboardInterrupt:
            print("Optimering avbröts av användaren")

        finally:    
            parameters_used = {
                    "Algorithm": "Hillclimbing",
                    "Initial Step Size": step_size,
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
                self.timestamps
                )

class LookupTable:
# sparar och hämtar data i lookup table för initial gissning innan algorithm. 
    def __init__(self, filename="lookup_table.csv"):
        self.filename = filename
        self.data = []  # Lista av dictionaries: {'speed': ..., 'rpm': ..., 'torque': ..., 'tilt': ...}
        self.points = None  # NumPy array (N,3) med inputs
        self.tilts = None  # NumPy array (N,) med tiltvärden
        self.kdtree = None  # KDTree för effektiv närmaste-granne-sökning
        self.load() # metod för att läsa in data från CSV

    # Läser tidigare data från CSV
    def load(self):
        if not os.path.exists(self.filename):
            print("Ingen tidigare lookup-tabell hittades.")
            return  # Hoppar över om filen inte finns
    
        with open(self.filename, mode='r') as file:
            reader = csv.DictReader(file)
            self.data = [
                {
                    'speed': float(row['speed']),
                    'rpm': float(row['rpm']),
                    'torque': float(row['torque']),
                    'tilt': float(row['tilt']),
                }
                for row in reader
            ]
        self._rebuild_index() # KDTree
        print(f"Laddade {len(self.data)} datapunkter från lookup-tabell.")

    # Skriver all data till CSV
    def save(self):
        with open(self.filename, mode='w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=['speed', 'rpm', 'torque', 'tilt'])
            writer.writeheader()
            for entry in self.data:
                writer.writerow(entry)
        print(f"Lookup-tabell sparad till {self.filename}")

    # Uppdaterar interna KDTree för snabb sökning
    def _rebuild_index(self):
        if not self.data:
            self.points = self.tilts = self.kdtree = None
            return
        self.points = np.array([
            (entry['speed'], entry['rpm'], entry['torque']) for entry in self.data
        ])
        self.tilts = np.array([entry['tilt'] for entry in self.data])
        self.kdtree = KDTree(self.points)

    # Lägger till ny datapunkt om den inte redan finns (inom tolerans)
    def add_entry(self, speed, rpm, torque, tilt, tolerance=1.0):
        if self.kdtree:
            dist, idx = self.kdtree.query([speed, rpm, torque])
            if dist < tolerance:
                print("Liknande datapunkt finns redan, hoppar över inlärning.")
                return  # Undviker duplicat

        entry = {'speed': speed, 'rpm': rpm, 'torque': torque, 'tilt': tilt}
        self.data.append(entry)
        self._rebuild_index()
        self.save()
        print(f"La till ny datapunkt: speed={speed}, rpm={rpm}, torque={torque}, tilt={tilt}")

    # Hämtar tilt för närmaste datapunkt baserat på input
    def find_closest_tilt(self, speed, rpm, torque):
        if self.kdtree is None or len(self.data) == 0:
            print("Lookup-tabell tom – inget initialt förslag.")
            return None

        query = np.array([speed, rpm, torque])
        dist, idx = self.kdtree.query(query)
        closest_tilt = self.tilts[idx]
        print(f"Närmsta match: speed={self.points[idx][0]}, rpm={self.points[idx][1]}, torque={self.points[idx][2]} → tilt={closest_tilt}")
        return closest_tilt

def main():
    print("'1' : styr med terminal, '2': styr med kontrollpanel. välj(1/2)")
    val0 = input().strip()
    if val0 == '1':
        print("Styr med terminal")
        try:
            can_interface = CANInterface()
            can_interface.start_reading()
            csv_logger = CSVLoggning()
            can_interface.start_periodic_sending()

            print("Välj algorithm:")
            print("'1': Hillclimbing")
            print("'2': Vanilla Gradient Ascent")
            print("'3': Momentum Gradient Ascent")
            print("'4': READ ONLY Mode")
            print("'5': READ AND WRITE Mode")
            print("'6': Fake ahh data test")
            val = input()

            if val == '1':
                print("Algoritm vald: HILL CLIMBING")
                optimizer = HillClimbing(can_interface, csv_logger)
                optimizer.run()
            elif val == '2':
                print("Algoritm vald: VANILLA GRADIENT ASCENT")
                optimizer = GradientAscent(can_interface, csv_logger)
                optimizer.run()
            elif val == '3':
                print("Algoritm vald: MOMENTUM GRADIENT ASCENT")
                optimizer = GradientAscent(can_interface, csv_logger, use_momentum=True)
                optimizer.run()
            elif val == '4':
                print("READ ONLY MODE – Visar data från CAN-bus")
                can_interface.read_only_mode()
                return
            elif val == '5':
                print("READ AND WRITE MODE – Visar och sparar data")
                can_interface.read_only_mode(csv_logger, log=True)
                return
            elif val == '6':
                print("Fake Data test")
                optimizer = GradientAscent(can_interface, csv_logger, fake=True)
                can_interface.start_reading(fake=True)
                optimizer.run()
            else:
                print("Ogiltigt val – avslutar.")
                return
            
        except KeyboardInterrupt:
            print("Avbröts manuellt av användaren.")
        finally:
            can_interface.stop_periodic_sending()
            can_interface.stop_read()
            print("Avslutar CAN-kommunikation.")


            # Efter optimering – hämta bästa tilt från historik
            # if optimizer.efficiency_history and val2 == 'y':
            #     final_idx = np.argmax(optimizer.efficiency_history)
            #     best_tilt = optimizer.tilt_percent_history[final_idx]
            #     can_interface.current_tilt_percent = best_tilt

            #     # Läs aktuella värden direkt från CAN
            #     tilt_percent, power, speed, filtered_rpm, filtered_vridmoment, raw_hastighet, , raw_rpm = can_interface.get_latest_data()

            #     # Spara till lookup
            #     lookup.add_entry(
            #         speed=speed,
            #         rpm=filtered_rpm,
            #         torque=filtered_torque,
            #         tilt=best_tilt
            #     )
    elif val0 == '2':
        print("Styr med kontrollpanel")
        can_interface = CANInterface()
        can_interface.start_reading()
        csv_logger = CSVLoggning()
        can_interface.start_periodic_sending()
        # controlpanel = ControlPanel()
        # controlpanel.send_efficiency_loop()

        optimizer = GradientAscent(can_interface, csv_logger)
        stop_flag = threading.Event()

        def handle_arduino_message(msg):
        # hanterar kommunikation med arduino
            if msg == "STARTA":
                if not optimizer_thread.is_alive():
                    print("[PYTHON] Startar optimering")
                    stop_flag.clear()
                    optimizer_thread = threading.Thread(target=run_optimizer)
                    optimizer_thread.start()
                
            elif msg == "AVBRYT":
                print("[PYTHON] Avbryter optimering")
                stop_flag.set()

        arduino = ControlPanel(port='/dev/ttyUSB0', message_callback=handle_arduino_message)
        arduino.can = can_interface

        def run_optimizer():
            try:
                optimizer.run()
                if not stop_flag.is_set():
                    arduino.send_message("KLAR")
                else:
                    arduino.send_message("AVBRUTEN")
            except Exception as e:
                print(f"[PYTHON] Fel under optimering: {e}")
                arduino.send_message("AVBRUTEN")
        
        optimizer_thread = threading.Thread(target=run_optimizer)

        try:
            print("[PYTHON] Väntar på kontrollpanelkommande")
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Avslutar....")

        finally:
            stop_flag.set()
            arduino.stop()
            can_interface.stop_periodic_sending()
            can_interface.stop_read()
    else:
        print("Ogiltigt val - Programmet avslutas")

if __name__ == "__main__":
    main()  

# # # # # # # # # # # # #
#                       #
#         _             #
#        /|\            #
#       /_|_\           #
#     ____|____         #
#     \_o_o_o_/         #
#~~~~~~~~~~ | ~~~~~~~~~~#
#___________t___________#
# # # # # # # # # # # # #
