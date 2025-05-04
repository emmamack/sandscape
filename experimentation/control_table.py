import serial
import time

def check_for_response():
    while True:
        line_of_data = serial_port.readline().decode(encoding = "ascii")
        if len(line_of_data.rstrip()) > 0:
            break
    print(line_of_data)

def write_point(x):
    print(f"writing: {bytes(x, 'utf-8')}")
    serial_port.write(bytes(x,  'utf-8'))


points = [(0,0), (2000,2000), (2000,7000), (50,7000)]

arduino_com_port = "COM6"
baud_rate = 9600
serial_port = serial.Serial(arduino_com_port, baud_rate, timeout=1)

time.sleep(2) # THIS DELAY IS IMPORTANT AND THIS SCRIPT DOESN'T WORK WITHOUT IT

for r, theta in points:
    formatted_point = f"{str(r).zfill(4)}{str(theta).zfill(4)}"
    write_point(formatted_point)
    check_for_response()

# TODO: why did the lin motor go the wrong way when homing at the end of a sequence?