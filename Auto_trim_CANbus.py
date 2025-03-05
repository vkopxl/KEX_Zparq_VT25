import can
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

# ----------------Pill parametrar--------------- #

step_size = 1.0  # Vinkelförändrningens storlek
tolerance = 0.01  
vinkelhastighet = 1

# ------------------FAILSAFES-------------------- #

# Gränser för trimvinkel
MIN_ANGLE = 500 # minsta tillåtna vinkel
MAX_ANGLE = 9500 # Största tilåtna vinkel

# Clamp för trimvinkel, returnerar max eller min vinkel om vinkeln överskrids i algoritmen
def clamp_angle(angle):
    clamped_angle = max(MIN_ANGLE, min(angle, MAX_ANGLE))
    if clamped_angle == MIN_ANGLE:
        print("Minsta vinkel nådd: {} grader".format(MIN_ANGLE))
    elif clamped_angle == MAX_ANGLE:
        print("Största vinkel nådd: {} grader".format(MAX_ANGLE))
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
    angle, vridmoment, varvtal, speed = None, None, None, None
    start_time = time.time()
    # Lyssna på CAN-meddelanden
    while time.time() - start_time < 10.0: # Timeout efter 10s om ingen data tar emot
        message = bus.recv(timeout=1.0)  # Vänta 1 sek på meddelande
        if message:
            if message.arbitration_id == 0x100:  
                angle = int.from_bytes(message.data, byteorder='big')
            elif message.arbitration_id == 0x101:  # Effekt-ID
                vridmoment = int.from_bytes(message.data, byteorder='big')
            elif message.arbitration_id == 0x102:  # Hastighet-ID
                varvtal = int.from_bytes(message.data, byteorder='big')
            elif message.arbitration_id == 0x102:  # Hastighet-ID
                speed = int.from_bytes(message.data, byteorder='big')
            
            if angle is not None and vridmoment is not None and varvtal is not None and speed is not None:
                break  # Avsluta när alla värden är inlästa
        
        if angle is None or vridmoment is None or varvtal is None or speed is None:
            print("NO DATA READ FROM CANBUS")

    filtered_vridmoment = smooth_data(vridmoment_buffer, vridmoment)
    filtered_varvtal = smooth_data(varvtal_buffer, varvtal)
    filtered_speed = smooth_data(speed_buffer, speed)

    power = filtered_vridmoment*(2*3.1416*filtered_varvtal/60)

    return current_angle, power, filtered_speed

def send_can_angle(angle):
    # Skicka ny vinkel till motorn via CANbus.
    angle = clamp_angle(angle) # Failsafe, ser till att vinkeln är inom säkra gränser
    message = can.Message(arbitration_id=0x200, data=angle.to_bytes(2, byteorder='big'), is_extended_id=True)
    bus.send(message) # send periodic!!!
    print(f"Skickade ny vinkel: {angle} grader")

# CANBUS -> class

# ------------------ALGORITHM-------------------- #

# Initialisering
max_iterations = 1000  
iteration = 0
efficiency_history = []
angle_history = []
speed_history = []
power_history = []

current_angle, power, speed = read_can_data()
prev_efficiency = measure_efficiency(speed,power)

efficiency_history.append(prev_efficiency)
angle_history.append(current_angle)
speed_history.append(speed)
power_history.append(power)

# Initierande vinkeländring
new_angle =  current_angle + step_size
send_can_angle(new_angle)

# Tid det kommer ta motorn att justera vinkeln till den nya
tid = abs(step_size)/vinkelhastighet

# Tidsfördröjning till nästa iteration så motorn har hunnit ändra till den nya vinkel
time.sleep(tid+1)   

# Huvudoptimeringsloop
while iteration < max_iterations:
    iteration += 1
    
    # Läs av motorns vinkel, effekt och hastighet
    current_angle, power, speed = read_can_data()
    
    # Lagra värden till plot
    angle_history.append(current_angle)
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
        new_angle =  current_angle + step_size

        # Skicka ut vinkeln som motorn ska ändra till --> motorn ändrar vinkel
        send_can_angle(new_angle)
        
    else:
        # Effektiviteten försämrades, vänd riktning och minska steglängden
        step_size = -0.5 * step_size

        new_angle =  current_angle + step_size
        
        # Skicka ut vinkeln som motorn ska ändra till --> motorn ändrar vinkel
        send_can_angle(new_angle)

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
iterations = np.arange(len(angle_history))

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



#       _
#      /|\
#     /_|_\
#   ____|____
#   \_o_o_o_/
#~~~~~~~ | ~~~~~~~~
#________t_________ 