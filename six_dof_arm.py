import serial
import time

arduino = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)  # Wait for Arduino to initialize

# please provide inverse kinematic angle here 
angles = [90,90,90,90,90,90]
# angles = [-3.0262 ,  76.5548,  -43.7422 ,  -0.8602, -105.3913 , -90.0000]


data = ",".join(map(str, angles)) + "\n"

arduino.write(data.encode())
print(f"Sent: {data.strip()}")

arduino.close()
