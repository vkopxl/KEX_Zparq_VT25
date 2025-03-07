import can
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

class CANinterface:
    def __init__(self, channel='PCan_USBBUS1', bitrate=1000000):
        self.bus = can.interface.Bus(bustype='pcan', channel = channel, bitrate = bitrate)
        self.speed_buffer = deque(maxlen=5)
        self.vridmoment_buffer = deque(maxlen=5)
        self.varvtal_buffer = deque(maxlen=5)
        self.min_tilt_percent = 500
        self.max_tilt_percent = 9500

    def clamp_tilt_percent(self,tilt_percent):
        clamped_tilt_percent = max(self.min_tilt_percent, min(tilt_percent, self.max_tilt_percent))
        if clamped_tilt_percent == self.min_tilt_percent:
            print("Minsta vinkel nådd: {} grader".format(self.min_tilt_percent))
        elif clamped_tilt_percent == self.max_tilt_percent:
            print("Största vinkel nådd: {} grader".format(self.max_tilt_percent))
        return clamped_tilt_percent
    
    def smooth_data(self, buffer, new_value):
        buffer.append(new_value)
        return (sum(buffer) / len(buffer))
    
    def read_can_data(self, timeout=10.0):
        
        tilt_percent = vridmoment = varvtal = speed = None
        start_time = time.time()
        
        # Lyssna på CAN-meddelanden
        while time.time() - start_time < 10.0: # Timeout efter 10s om ingen data tar emot
            message = self.bus.recv(timeout=1.0)  # Vänta 1 sek på meddelande
            if message:
                if message.arbitration_id == 0x14FF0A50:
                    tilt_percent = int.from_bytes(message.data(2), byteorder='little')|int.from_bytes(message.data(3), byteorder='little')
                elif message.arbitration_id == 0x0CC6D4E1: 
                    vridmoment = int.from_bytes(message.data(0), byteorder='little') |int.from_bytes(message.data(1), byteorder='little')
                    vridmoment = (vridmoment*0.0625)-2048 # Offset och scaling, [Nm]
                elif message.arbitration_id == 0x0CC6D4E1: 
                    varvtal = int.from_bytes(message.data(2), byteorder='little') |int.from_bytes(message.data(3), byteorder='little')
                    varvtal = (varvtal*0.5)-16000 # Offset och scaling, [RPM]
                elif message.arbitration_id == 0x14FF1140: 
                    speed = int.from_bytes(message.data(0), byteorder='little') |int.from_bytes(message.data(1), byteorder='little')
                    speed = (speed*0.1) # Scaling, [m/s]
            
                if tilt_percent is not None and vridmoment is not None and varvtal is not None and speed is not None:
                    break  # Avsluta när alla värden är inlästa
        
            if tilt_percent in None or power is None or speed is None:
                print("NO DATA READ FROM CANBUS")

        filtered_vridmoment = smooth_data(self.vridmoment_buffer, vridmoment)
        filtered_varvtal = smooth_data(self.varvtal_buffer, varvtal)
        filtered_speed = smooth_data(self.speed_buffer, speed)

        power = filtered_vridmoment * (2 * 3.1416 * filtered_varvtal / 60)  # [W]

        return tilt_percent, power, filtered_speed

    def send_tilt_percent(self, tilt_percent):
        tilt_percent = self.clamp_angle(tilt_percent)
        
        tilt_bytes = tilt_percent.to_bytes(2, byteorder='little')

        data_bytes = bytearray(8) 
        data_bytes[0:1] = tilt_bytes 

        message = can.Message(
            arbitration_id=0x18FF1840,
            data=data_bytes,
            is_extended_id=True
        )

        self.bus.send(message)
        

class trim_algorithm:
    def __init__(self, can_interface, step_size=1.0, tolerance=0.01, vinkelhastighet = 1, max_iterations=1000):
        self.can = can_interface
        self.step_size = step_size
        self.tolerance = tolerance
        self.vinkelhastighet = vinkelhastighet
        self.max_iterations = max_iterations
        self.efficiency_history = []
        self.tilt_percent_history = []
        self.speed_history = []
        self.power_history = []

    def measure_efficiency(self, speed, power):
        return speed / power if power else 0    # [m/J]
        
    def run(self):
        current_tilt, power, speed = self.can.read_data()
        prev_efficiency = self.measure_efficiency(speed, power)
        
        self.tilt_percent_history.append(current_tilt)
        self.power_history.append(power)
        self.speed_history.append(speed)
        self.efficiency_history.append(prev_efficiency)

        self.can.send_tilt_percent(current_tilt + self.step_size)
        time.sleep()

        # FORTSÄTT SKRIV KLASSEN HÄR 
        

# ----------------Pill parametrar--------------- #

step_size = 1.0  # Vinkelförändrningens storlek
tolerance = 0.01  
vinkelhastighet = 1

# ------------------FAILSAFES-------------------- #

# Gränser för trimvinkel
MIN_PERCENT = 500 # minsta tillåtna vinkel
MAX_PERCENT = 9500 # Största tilåtna vinkel

# Clamp för trimvinkel, returnerar max eller min vinkel om vinkeln överskrids i algoritmen
def clamp_percent(angle):
    clamped_angle = max(MIN_PERCENT, min(angle, MAX_PERCENT))
    if clamped_angle == MIN_PERCENT:
        print("Minsta vinkel nådd: {} procent".format(MIN_PERCENT))
    elif clamped_angle == MAX_PERCENT:
        print("Största vinkel nådd: {} procent".format(MAX_PERCENT))
    return clamped_angle

# Räknar kvot mellan hastighet och tid
def measure_efficiency(speed,power):
    return (speed/power)  # Effektiviteten minskar när vi avviker från optimal vinkel

# Buffrar för sensordata
speed_buffer = deque(maxlen=5) # Hastighet
vridmoment_buffer = deque(maxlen=5) # Vridmoment
varvtal_buffer = deque(maxlen=5) # Varvtal
#power_buffer = deque(maxlen=5) # Effekt

def smooth_data(buffer, new_value):
    # filter för inkommande sensordata med glidande medelvärde
    buffer.append(new_value)
    return sum(buffer) / len(buffer)

# ------------------CANbus-------------------- #

# CANbus setup (justera för din hårdvara)
bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=1000000)

def read_can_data():
    # Läs vinkel, effekt och hastighet från CANbus.
    tilt_percent, vridmoment, varvtal, speed = None, None, None, None
    start_time = time.time()
    # Lyssna på CAN-meddelanden
    while time.time() - start_time < 10.0: # Timeout efter 10s om ingen data tar emot
        message = bus.recv(timeout=1.0)  # Vänta 1 sek på meddelande
        if message:
            if message.arbitration_id == 0x14FF0A50:
                tilt_percent = int.from_bytes(message.data(2), byteorder='little')|int.from_bytes(message.data(3), byteorder='little')
            elif message.arbitration_id == 0x0CC6D4E1: 
                vridmoment = int.from_bytes(message.data(0), byteorder='little') |int.from_bytes(message.data(1), byteorder='little')
            elif message.arbitration_id == 0x0CC6D4E1: 
                varvtal = int.from_bytes(message.data(2), byteorder='little') |int.from_bytes(message.data(3), byteorder='little')
            elif message.arbitration_id == 0x14FF1140: 
                speed = int.from_bytes(message.data(0), byteorder='little') |int.from_bytes(message.data(1), byteorder='little')
            
            if tilt_percent is not None and vridmoment is not None and varvtal is not None and speed is not None:
                break  # Avsluta när alla värden är inlästa
        
        if tilt_percent in None or power is None or speed is None:
            print("NO DATA READ FROM CANBUS")

    filtered_vridmoment = smooth_data(vridmoment_buffer, vridmoment)
    filtered_varvtal = smooth_data(varvtal_buffer, varvtal)
    filtered_speed = smooth_data(speed_buffer, speed)
    
    power = filtered_vridmoment*(filtered_varvtal*3.1416/60) # Beräkna effekten
    
    return tilt_percent, power, filtered_speed

def send_can_angle(angle):
    # Skicka ny vinkel till motorn via CANbus.
    angle = clamp_angle(angle) # Failsafe, ser till att vinkeln är inom säkra gränser
    
    message = can.Message(arbitration_id=0x18FF1840, data=angle.to_bytes(2, byteorder='big'), is_extended_id=True)
    bus.send(message) # send periodic!!!
    print(f"Skickade ny vinkel: {angle} grader")

# CANBUS -> class

# ------------------ALGORITHM-------------------- #

def main():
    # Initialisering
    max_iterations = 1000  
    iteration = 0
    efficiency_history = []
    tilt_percent_history = []
    speed_history = []
    power_history = []

    current_tilt_percent, power, speed = read_can_data()
    prev_efficiency = measure_efficiency(speed,power)

    efficiency_history.append(prev_efficiency)
    tilt_percent_history.append(current_tilt_percent)
    speed_history.append(speed)
    power_history.append(power)

    # Initierande vinkeländring
    new_tilt_percent =  current_tilt_percent + step_size
    send_can_angle(new_tilt_percent)

    # Tid det kommer ta motorn att justera vinkeln till den nya
    tid = abs(step_size)/vinkelhastighet

    # Tidsfördröjning till nästa iteration så motorn har hunnit ändra till den nya vinkel
    time.sleep(tid+1)   

    # Huvudoptimeringsloop
    while iteration < max_iterations:
        iteration += 1
        
        # Läs av motorns vinkel, effekt och hastighet
        current_tilt_percent, power, speed = read_can_data()
        
        # Lagra värden till plot
        tilt_percent_history.append(current_tilt_percent)
        speed_history.append(speed)
        power_history.append(power)

        # Räkna ut kvoten mellan hastighet och effekt
        new_efficiency = measure_efficiency(speed,power)

        # Lagra effektiviteten för plot
        efficiency_history.append(new_efficiency)

        # Beräkna fel (trend för förändring av effektivitet)        
        error = new_efficiency - prev_efficiency
        
        # Kontrollera konvergens
        if abs(error) < tolerance:
            print(f'Optimal vinkel funnen: {current_angle:.2f} grader efter {iteration} iterationer')
            break

        if error > 0:
            # Effektiviteten förbättrades, fortsätt i samma riktning
            new_tilt_percent =  current_tilt_percent + step_size
            
            # Skicka ut vinkeln som motorn ska ändra till --> motorn ändrar vinkel
            send_can_angle(new_tilt_percent)
            
        else:
            # Effektiviteten försämrades, vänd riktning och minska steglängden
            step_size = -0.5 * step_size

            new_tilt_percent =  current_tilt_percent + step_size
            
            # Skicka ut vinkeln som motorn ska ändra till --> motorn ändrar vinkel
            send_can_angle(new_tilt_percent)

        # Lagra effektiviteten till nästa iteration
        prev_efficiency = new_efficiency
        
        # Tid det kommer ta motorn att justera vinkeln till den nya
        tid = vinkelhastighet/abs(step_size)

        # Tidsfördröjning till nästa iteration så motorn har hunnit ändra till den nya vinkel
        time.sleep(tid+1)

    # Kontrollera om max antal iterationer uppnåddes
    if iteration == max_iterations:
        print('Maximalt antal iterationer uppnått. Konvergens ej garanterad.')

    # Skapa iterationsvektor för plottning
    iterations = np.arange(len(tilt_percent_history))

    # ------------------PLOTS-------------------- #

    # Plotta resultatet
    plt.figure(figsize=(8, 5))
    plt.plot(iterations, angle_history, markersize=3, label='Vinkel')
    plt.axhline(y=15, color='r', linestyle='--', label='Optimal vinkel (konstant)')
    plt.xlabel('Iterationer')
    plt.ylabel('Trimvinkel (grader)')
    plt.title('Motorns vinkel över antalet iternationer')
    plt.legend()
    plt.grid()
    plt.show()

    plt.figure(figsize=(8, 5))
    plt.plot(iterations, efficiency_history, markersize=3, label='Effektivitet')
    plt.axhline(y=15, color='r', linestyle='--', label='Optimal vinkel (konstant)')
    plt.xlabel('Iterationer')
    plt.ylabel('Effektivitet (m/J)')
    plt.title('Båtens effektivitet över antalet iternationer')
    plt.legend()
    plt.grid()
    plt.show()


    plt.figure(figsize=(8, 5))
    plt.plot(iterations, power_history, markersize=3, label='Effekt')
    plt.axhline(y=15, color='r', linestyle='--', label='Optimal vinkel (konstant)')
    plt.xlabel('Iterationer')
    plt.ylabel('Effekt (W)')
    plt.title('Motorns effekt över antalet iternationer')
    plt.legend()
    plt.grid()
    plt.show()


    plt.figure(figsize=(8, 5))
    plt.plot(iterations, speed_history, markersize=3, label='Båtens hastighet')
    plt.axhline(y=15, color='r', linestyle='--', label='Optimal vinkel (konstant)')
    plt.xlabel('Iterationer')
    plt.ylabel('Hastighet (ENHET)')
    plt.title('Båtens hastighet över antalet iternationer')
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == "__main__":
    main()


#       _
#      /|\
#     /_|_\
#   ____|____
#   \_o_o_o_/
#~~~~~~~ | ~~~~~~~~
#________t_________ 
