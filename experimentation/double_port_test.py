import serial
import time

def check_for_response(serial_port):
    while True:
        line_of_data = serial_port.readline().decode(encoding = "ascii")
        if len(line_of_data.rstrip()) > 0:
            break
    print(line_of_data)

def write_line(serial_port, x):
    if type(x) == str:
        print(f"{time.time():.5f} | writing: {bytes(x, 'utf-8')}")
        serial_port.write(bytes(x,  'utf-8'))
    else:
        print(f"{time.time():.5f} | writing: {x}")
        serial_port.write(x)
    

arduino_com_port_0 = "/dev/ttyACM0"
baud_rate_0 = 9600
serial_port_0 = serial.Serial(arduino_com_port_0, baud_rate_0, timeout=1)

time.sleep(2) # THIS DELAY IS IMPORTANT AND THIS SCRIPT DOESN'T WORK WITHOUT IT

arduino_com_port_1 = "/dev/ttyACM1"
baud_rate_1 = 115200
serial_port_1 = serial.Serial(arduino_com_port_1, baud_rate_1, timeout=1)

check_for_response(serial_port_0)
write_line(serial_port_0, "?")
check_for_response(serial_port_1)


