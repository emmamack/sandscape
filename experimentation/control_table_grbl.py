import serial
import time
import math

def check_for_response(timeout=5, reset=True):
    if reset:
        serial_port.reset_input_buffer()
    all_data = ""
    start_time = time.time()
    end_time = start_time + timeout
    print(f"listening starting {start_time} ...")
    while time.time() < end_time:
        line_of_data = serial_port.readline().decode(encoding = "ascii")
        if len(line_of_data.rstrip()) > 0:
            print(line_of_data)
            all_data = all_data + line_of_data
            if "ok" in line_of_data.rstrip() or ">" in line_of_data:
                print("done listening")
                break
        print("cfr sleeping")
        time.sleep(0.05)
    if time.time() > end_time:
        print("check_for_response timed out - done listening")
    return all_data

def write_line(x):
    print(f"writing: {bytes(x, 'utf-8')}")
    serial_port.write(bytes(x,  'utf-8'))
    
def send_grbl_clear():
    print(f"writing: {bytes([0x85])}")
    serial_port.write(bytes([0x85]))

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


# points = []
# for i in range(250):
#     r = i*40
#     theta = (i*800)%8000
#     points.append((r,theta))
    
points = []
for i in range(100):
    r = i*10
    theta = (i*10)
    points.append((r,theta))

arduino_com_port = "COM8"
baud_rate = 115200
serial_port = serial.Serial(arduino_com_port, baud_rate, timeout=1)

time.sleep(2) # THIS DELAY IS IMPORTANT AND THIS SCRIPT DOESN'T WORK WITHOUT IT

send_grbl_clear()
write_line("?")
check_for_response()

write_line(f"$X")
write_line(f"G1 X100 F100\n")
write_line(f"G1 X200 F100\n")
write_line(f"G1 X250 F100\n")
write_line(f"G1 X200 F200\n")
time.sleep(2)
write_line("?")
check_for_response()
write_line("!\n")
time.sleep(1)
write_line("?")
check_for_response()

print("writing Ctr+X")
serial_port.write(bytes([0x18]))
time.sleep(1)
write_line("?")
check_for_response()
write_line(f"$X")
write_line("?")

time.sleep(2)
write_line("~\n")
time.sleep(1)
write_line("?")
check_for_response()


write_line(f"G1 X10 F200\n")

write_line("?")
check_for_response()


# SEQUENCE TO STOP, CLEAR BUFFER, UNLOCK:
# serial_port.write(bytes([0x18]))
# write_line(f"$X")

# r_prev = 0
# theta_prev = 0

# for r, theta in points:
#     write_line("?")
#     status = check_for_response()
#     if "Idle" in status:
#     rot_speed = get_rot_speed(r_prev, r, theta_prev, theta) * 2000
#     write_line(f"G1 X{str(r)} Y{str(theta)} F{str(rot_speed)}\n")
#     check_for_response() # expect "ok"

#     r_prev = r
#     theta_prev = theta
    
# print("done")

    # expect <Idle|MPos:10.000,0.000,0.000|FS:0,0|Ov:100,100,100>

# i = 0
# r_prev = 0
# theta_prev = 0
# rot_speed = 2000
# runloop = True
# while runloop:
#     write_line("?")
#     status = check_for_response()
#     if "Idle" in status:
#         i = i+1
#         print(f"Next point is {points[i]}.")
#         r, theta = points[i]
#         # rot_speed = get_rot_speed(points[i-1][0], points[i][0], points[i-1][1], points[i][1]) * 1000
#         rot_speed = 100
#         write_line(f"G1 X{str(r)} Y{str(theta)} F{str(rot_speed)}\n")
#     # time.sleep(0.02)
    
    

