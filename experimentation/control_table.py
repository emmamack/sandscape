import serial
import time
import math

def check_for_response():
    while True:
        line_of_data = serial_port.readline().decode(encoding = "ascii")
        if len(line_of_data.rstrip()) > 0:
            break
    print(line_of_data)

def write_point(x):
    print(f"writing: {bytes(x, 'utf-8')}")
    serial_port.write(bytes(x,  'utf-8'))

def get_rot_speed(r0,r1,t0,t1):
    #normalized
    r1_n = r1/11000
    r0_n = r0/11000
    t1_n = t1/8000
    t0_n = t0/8000

    distance = math.sqrt((r1_n-r0_n)**2 + (math.pi*(r0_n+r1_n)*(t1_n-t0_n))**2)
    delta_theta = t1-t0

    if distance == 0:
        rot_ratio = 198000 # TODO: this is arbitrary
    else:
        rot_ratio = delta_theta/distance
    
    if rot_ratio > 198000:
        rot_ratio = 198000
    
    # TODO: tune all these constants

    return int(100 / abs(int(rot_ratio/2000))) # TODO: get rid of 100 / when have PWM drivers


points = []
for i in range(250):
    r = i*40
    theta = (i*800)%8000
    points.append((r,theta))

arduino_com_port = "COM6"
baud_rate = 9600
serial_port = serial.Serial(arduino_com_port, baud_rate, timeout=1)

time.sleep(2) # THIS DELAY IS IMPORTANT AND THIS SCRIPT DOESN'T WORK WITHOUT IT

r_prev = 0
theta_prev = 0

for r, theta in points:
    rot_speed = get_rot_speed(r_prev, r, theta_prev, theta)
    formatted_point = f"{str(r).zfill(5)}{str(theta).zfill(4)}{str(rot_speed).zfill(2)}"
    write_point(formatted_point)
    check_for_response()

    r_prev = r
    theta_prev = theta