import serial
import time

def check_for_response():
    while True:
        line_of_data = serial_port.readline().decode(encoding = "ascii")
        if len(line_of_data.rstrip()) > 0:
            break
    print(line_of_data)

arduino_com_port = "dev/ttyACM0"
baud_rate = 9600
serial_port = serial.Serial(arduino_com_port, baud_rate, timeout=1)

time.sleep(2) # THIS DELAY IS IMPORTANT AND THIS SCRIPT DOESN'T WORK WITHOUT IT

check_for_response()