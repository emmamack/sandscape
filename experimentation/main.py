import serial
import time
import math
import threading
import queue
import json
from datetime import datetime


from parse_grbl_status import *

class Mode:
    """
    Base class for different operational modes.
    Subclasses define specific behaviors and configurations.
    """
    def __init__(self):
        # Common default attributes
        self.mode_type = "base"  # Should be overridden by subclasses
        self.segment_length = 5
        self.linspeed = 3000 #mm/min
        self.r_dir = 1
        self.theta_dir = 1
        self.pitch = 14  # Default, can be overridden by specific modes
        self.waypoints_xy = []
        self.waypoints_rt = []
        self.waypoints_i = 0
        self.done = False

        # Default behavioral flags - subclasses can override as needed
        self.connect_to_nano = False
        self.connect_to_uno = True  # Common default
        self.check_dials = False
        self.check_touch_sensors = False
        self.check_grbl_status = True  # Common default

    def next_move(self, rt, limits_hit, dials, touch_sensors, grbl_status):
        print("Base next_move method called. Override in subclass.")

    @staticmethod
    def all_modes_in_list():
        """Returns a list containing an instance of each defined mode."""
        # Order matches the original mode_types_list implicitly
        return [
            WaypointMode(),
            SpiralMode(),
            ReactiveOnlyMode(),
            ReactiveSpiralRippleMode()
        ]

class WaypointMode(Mode):
    """Mode to send the marble to a waypoint, default is (0,0)."""
    def __init__(self):
        super().__init__()
        self.mode_type = "waypoint"
        self.waypoints_xy = [(0,0)]
        # Behavioral flags are inherited from Mode base.
        # Original `become_home` flags matched these defaults:
        # self.connect_to_nano = False
        # self.connect_to_uno = True
        # self.check_dials = False
        # self.check_touch_sensors = False
        # self.check_grbl_status = True
        
    def next_move(self, rt, limits_hit, dials, touch_sensors, grbl_status):
        # TODO: know when you have reached a waypoint and don't overshoot
        if self.waypoints_i < len(self.waypoints_xy):
            self.done = True
            return None
        else:
            self.done = False
            xy = rt2xy(rt)
            vector = get_direction(xy, self.waypoints_xy[self.waypoints_i])*self.segment_length
            new_xy = [xy[0] + vector[0], xy[1] + vector[1]]
            new_rt = xy2rt(new_xy)
            return new_rt

class SpiralMode(Mode):
    """Mode for the marble to draw a spiral outwards from its current location."""
    def __init__(self):
        super().__init__()
        self.mode_type = "spiral"
        # Behavioral flags are inherited from Mode base.
        # Original `become_spiral` flags matched these defaults.
        
    def next_move(self, rt, limits_hit, dials, touch_sensors, grbl_status):
        r = rt[0]
        theta = rt[1]

        if limits_hit["r_max"] == True and self.r_dir == 1:
            self.done = True
            return None
        elif limits_hit["r_min"] == True and self.r_dir == -1:
            self.done = True
            return None
        else:
            self.done = False
            if r>30:
                seg_angle = 360 * self.segment_length/r*2*math.pi
            else:
                seg_angle = 360
            
            new_theta = theta + seg_angle
            new_r = r + self.pitch*seg_angle/360
            
            # grbl_assumed_dist = math.sqrt((r-new_r)**2 + (theta-new_theta)**2)
            # compensation_ratio = grbl_assumed_dist/self.segment_length
            # new_speed = self.linspeed*compensation_ratio
            
            pi=math.pi
            r_ave = (r+new_r)/2
            new_speed = -2*pi*r_ave + self.linspeed + 360
            return [new_r, new_theta, new_speed]
        
class ReactiveOnlyMode(Mode):
    """Mode where the marble only moves due to touch sensor activation."""
    def __init__(self):
        super().__init__()
        self.mode_type = "reactive only"
        # Behavioral flags are inherited from Mode base.
        # Original `become_reactive` flags matched these defaults.

class ReactiveSpiralRippleMode(Mode):
    """
    Mode for the marble to draw a spiral that can be affected by touch sensor activation.
    (Behavioral flags for this mode were not fully defined in the original implementation)
    """
    def __init__(self):
        super().__init__()
        self.mode_type = "reactive spiral ripple"
        # Inherits default behavioral flags from Mode base.
        # The original `become_reactive_spiral_ripple` method only set the mode_type.
        # If specific flags are needed (e.g., self.check_touch_sensors = True),
        # they should be set here.

def normalize_vector(xy):
    length = math.sqrt(xy[0]**2 + xy[1]**2)
    return [xy[0]/length, xy[1]/length]

def get_direction(curr_xy, next_xy):
    dx = next_xy[0] - curr_xy[0]
    dy = next_xy[1] - curr_xy[1]
    return normalize_vector([dx, dy])

def xy2rt(xy):
    r = math.sqrt(xy[0]**2 + xy[1]**2)
    theta = math.atan2(xy[1], xy[0])
    return [r, theta]

def rt2xy(rt):
    x = rt[0]*math.cos(rt[1])
    y = rt[0]*math.sin(rt[0])
    return [x, y]

def ticks2rt(ticks):
    r = ticks[0]/TICKS_PER_MM_R
    theta = ticks[1]/TICKS_PER_DEG_THETA
    return [r, theta]

def ticks2xy(ticks):
    return rt2xy(ticks2rt(ticks))

def rt2ticks(rt):
    r = rt[0]*TICKS_PER_MM_R
    theta = rt[1]*TICKS_PER_DEG_THETA
    return [r, theta]

def xy2ticks(xy):
    return rt2ticks(xy2rt(xy))

def est_serial_con(arduino_com_port, baud_rate):
    serial_port = serial.Serial(arduino_com_port, baud_rate, timeout=1)
    time.sleep(2) # Wait for connection to establish!
    return serial_port

def read_from_port(ser, stop_event, data_queue):
    """
    Reads data from the serial port line by line in a dedicated thread.

    Args:
        ser (serial.Serial): The initialized serial port object.
        stop_event (threading.Event): The event to signal the thread to stop.
    """
    print("Reader thread started.")
    while not stop_event.is_set():
        try:
            # The readline() function will block until a newline character
            # is received, or until the timeout (set during port initialization)
            # is reached.
            line = ser.readline()

            # If a line was actually read (i.e., not a timeout)
            if line:
                # Decode the bytes into a string, using UTF-8 encoding.
                # 'errors='ignore'' will prevent crashes on decoding errors.
                # .strip() removes leading/trailing whitespace, including the newline.
                decoded_line = line.decode('utf-8', errors='ignore').strip()
                print(f"Received: {decoded_line}")
                if len(decoded_line) > 0:
                    data_queue.put(decoded_line)


        except serial.SerialException as e:
            # Handle cases where the serial port is disconnected or an error occurs
            print(f"Serial port error: {e}. Stopping thread.")
            break
        except Exception as e:
            # Handle other potential exceptions
            print(f"An unexpected error occurred: {e}")
            if not stop_event.is_set():
                # Avoid flooding the console with error messages
                time.sleep(1)

    print("Reader thread finished.")

def check_for_response_q(thread_data_queue, timeout=5):
    start_time = time.time()
    end_time = start_time + timeout
    print(f"listening starting {start_time} ...")
    while time.time() < end_time:
        try:
            line_of_data = thread_data_queue.nowait()
            if len(line_of_data.rstrip()) > 0:
                print(line_of_data)
            all_data = all_data + line_of_data
            if "ok" in line_of_data.rstrip() or ">" in line_of_data:
                print("done listening")
                break
        except queue.Empty:
            time.sleep(0.05)
        
        print("cfr sleeping")
        
    if time.time() > end_time:
        print("check_for_response timed out - done listening")
    return all_data

def check_for_response(serial_port, timeout=5, reset=True):
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

def write_line(serial_port, x):
    if type(x) == str:
        print(f"writing: {bytes(x, 'utf-8')}")
        serial_port.write(bytes(x,  'utf-8'))
    else:
        print(f"writing: {x}")
        serial_port.write(x)
    
def send_with_check(serial_port, x, data_queue, q_timeout=5):
    # make sure the queue is empty
    while not data_queue.empty():
        missed_msg = data_queue.get()
        # print(f"missed message: {missed_msg}")
    if data_queue.empty():
        write_line(serial_port, x)
        try:
            response = data_queue.get(timeout=q_timeout)
            print(f"Re: {x} -> {response}")
            if "ok" in response:
                data_queue.task_done()
                return True
            elif "error" in response:
                data_queue.task_done()
                return False
            elif "<" in response and ">" in response:
                data_queue.task_done()
                return True
            elif (   "Grbl 1.1h ['$' for help]" in response 
                  or "[MSG:'$H'|'$X' to unlock]" in response
                  or "ALARM:3" in response 
                  or "[MSG:Caution: Unlocked]" in response):
                data_queue.task_done()
                if x == GRBL_SOFT_RESET or x == GRBL_UNLOCK or x == GRBL_HOLD:
                    return True
                else:
                    return False
            else:
                print(f"UNEXPECTED REPONSE - Re: {x} -> {response}")
                data_queue.task_done()
                return False
        except queue.Empty:
            print(f"NO RESPONSE to {x}.")

    
def send_new_grbl_move(serial_port, r_theta_speed, grbl_status, queue):
    """Send a new command to GRBL after checking it won't drive more into limit switches."""
    if check_move(r_theta_speed, limits_hit, grbl_status):
        return send_with_check(serial_port, f"G1 X{r_theta_speed[0]:.2f} Z{r_theta_speed[1]:.2f} F{r_theta_speed[2]:.2f}\n", queue)
    else:
        print("Requested move {r_theta_speed} was not sent because it failed checks.")
        return False
    
def check_move(r_theta_speed, limits_hit, grbl_status):
    """Check move validity based on limits and grbl status."""
    r = r_theta_speed[0]
    # If r limit has been hit, next move needs to be in oppsite direction
    if limits_hit["r_min"] and r < grbl_status["MPos"][0]:
        print("Requested move is into the r_min limit switch.")
        return False
    elif limits_hit["r_max"] and r > grbl_status["MPos"][0]:
        print("Requested move is into the r_max limit switch.")
        return False
    elif r >= R_MAX or r < R_MIN:
        print("Requested move is out of bounds.")
        return False
    else:
        print("Requested move is valid.")
        return True
    

    
def send_grbl_hold(serial_port):
    """Stop immediately, no change to buffer or task."""
    write_line(serial_port, f"!\n")
    if grbl_status["State"] != "Hold":
        print("GRBL hold feed command failed.")
        return False
    else:
        return True
    
def send_grbl_resume(serial_port):
    """Resume after hold, no change to buffer or task."""
    write_line(serial_port, f"~\n")
    write_line(serial_port, f"?")
    time.sleep(0.1)
    if grbl_status["State"] == "Hold":
        print("GRBL resume command failed.")
        return False
    else:
        return True
    
def send_grbl_soft_reset(serial_port):
    """Stop immediately, retain position info but clear buffer and current task, then unlock to idle."""
    print(f"writing: {bytes([0x18])}")
    serial_port.write(bytes([0x18]))
    write_line(serial_port, f"$X")
    write_line(serial_port, f"?")
    time.sleep(0.1)
    # grbl_status = get_grbl_status(serial_port)
    if grbl_status["State"] != "Idle" or grbl_status['PlannerBuffer'] != 15:
        print("GRBL soft reset failed.")
        return False
    else:
        return True
                
    
def get_grbl_status(serial_port, data_queue):
    # make sure the queue is empty
    while not data_queue.empty():
        missed_msg = data_queue.get()
        print(f"missed message: {missed_msg}")
    if data_queue.empty():
        write_line(serial_port,"?")
        try:
            response = data_queue.get(timeout=5)
            if "<" in response and ">" in response:
                status_dict = parse_grbl_status(response)
                if status_dict == None:
                    print(f"ISSUE WITH STATUS PARSING - Re: ? ->: {response}")
                    return None
                path_history.append(status_dict["MPos"])
                return status_dict
            else:
                print(f"UNEXPECTED RESPONSE - Re: ? ->: {response}")
        except queue.Empty:
            print(f"No status message received")
    

def restart_uno_hard_reset(nano_serial_port):
    """Stop immediately, use nano to restart uno (to zero position info and clear buffer), and re-establish serial connection."""
    
    # Tell nano to send uno reset signal
    # TODO
    
    # Restablish serial connection with uno and confirm status
    uno_serial_port = est_serial_con(UNO_SERIAL_PORT_NAME, UNO_BAUD_RATE)
    send_grbl_soft_reset(uno_serial_port)
    # grbl_status = get_grbl_status(uno_serial_port)
    if grbl_status["State"] == "Idle":
        return uno_serial_port
    else:
        return None

def homing_sequence(serial_port, data_queue):
    q_timeout=60
    # make sure the queue is empty
    while not data_queue.empty():
        missed_msg = data_queue.get()
        # print(f"missed message: {missed_msg}")
    if data_queue.empty():
        write_line(serial_port, GRBL_HOME)
        start_time = time.time()
        end_time = start_time + q_timeout
        print("Homing...", end="", flush=True)
        while time.time() < end_time:
            if data_queue.empty():
                time.sleep(1)
                print(".", end="", flush=True)
            else:
                print()
                try:
                    response = data_queue.get(timeout=q_timeout)
                    print(f"Re: {GRBL_HOME} -> {response}")
                    if "ok" in response:
                        data_queue.task_done()
                        return True
                    else:
                        print(f"UNEXPECTED RESPONSE - Re: {GRBL_HOME} -> {response}")
                        data_queue.task_done()
                        return False
                except queue.Empty:
                    print(f"NO RESPONSE to {GRBL_HOME}.")





def get_rot_speed_ratio(r0,r1,t0,t1):
    #normalized
    r1_n = r1#/11000
    r0_n = r0#/11000
    t1_n = t1#/8000
    t0_n = t0#/8000

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

# === MAIN CONTROL SCRIPT ======================================================

# --- ARDUINO UNO MOTOR CONTROLLER CONFIGURATION ---
UNO_SERIAL_PORT_NAME = 'COM8'
UNO_BAUD_RATE = 115200

# --- ARDUINO NANO I/O CONTROLLER CONFIGURATION ---
NANO_SERIAL_PORT_NAME = 'COM6' #TODO
NANO_BAUD_RATE = 9600

# --- PROGRAM OPTIONS ---
flag_log_commands = True
flag_log_path = True
flag_grbl_homing_on = True

# --- SAND TABLE INFORMATION ---
TICKS_PER_MM_R = 40 # not used here, set in GRBL $100=40
TICKS_PER_DEG_THETA = 22.222 # not used here, set in GRBL $102=22.222
DISH_RADIUS_MM = 280
MARBLE_DIAMETER_MM = 14
MARBLE_WAKE_PITCH_MM = 14
R_MIN = 0
R_MAX = DISH_RADIUS_MM - MARBLE_DIAMETER_MM/2


# --- GRBL COMMANDS ---
GRBL_STATUS = bytes("?",  'utf-8')
GRBL_HOLD = bytes("!\n",  'utf-8')
GRBL_RESUME = bytes("~\n",  'utf-8')
GRBL_SOFT_RESET = bytes([0x18])
# GRBL_HARD_RESET = bytes([0x85]) ???? maybe? i don't know what this does
GRBL_UNLOCK = bytes("$X",  'utf-8')
GRBL_HOME = bytes("$H\n",  'utf-8')



# --- SETUP ---

limits_hit = {
    "r_min": False,
    "r_max": False,
    "theta_zero": False}
dials = {"speed": 0.5,
         "brightness": 0.5}
touch_sensors = [False, False, False, False, False, False, False, False,
                 False, False, False, False, False, False, False, False]
grbl_status = {'State': None,
               'MPos': [None,None],
               'WPos': [None,None], # not used
               'FeedRate': None,
               'PlannerBuffer': None,
               'RxBuffer': None,
               'SpindleSpeed': None, # not used
               'FeedOverride': None, # not used
               'RapidOverride': None, # not used
               'SpindleOverride': None} # not used

grbl_last_move = [] # format [r (X), theta (Z), speed (F)]

grbl_command_log = [] # only filled if flag_log_commands = True
moves_sent = 0

path_history = []


theta_correction = 0 # TODO

modes = [WaypointMode(), SpiralMode()]
mode_index = 1
mode = modes[mode_index]

if mode.connect_to_nano:
    print("Establishing serial connection to Arduino Nano...")
else:
    print("Not connecting to Arduino Nano.")

if mode.connect_to_uno:
    print("Establishing serial connection to Arduino Uno...")
    uno_serial_port = serial.Serial(UNO_SERIAL_PORT_NAME, UNO_BAUD_RATE, timeout=1)
    time.sleep(2) # Wait for connection to establish!
    
    data_queue = queue.Queue()
    stop_event = threading.Event()
    reader_thread = threading.Thread(target=read_from_port, args=(uno_serial_port, stop_event, data_queue))
    reader_thread.daemon = True
    reader_thread.start()
    time.sleep(2)
    
    run_control_loop = send_with_check(uno_serial_port, GRBL_SOFT_RESET, data_queue)
    time.sleep(2)

    print("Checking GRBL status...")
    grbl_status = get_grbl_status(uno_serial_port, data_queue)
    print(grbl_status)
    
    if flag_grbl_homing_on:
        print("Sending homing sequence...")
        run_control_loop = homing_sequence(uno_serial_port, data_queue)
        if not run_control_loop:
            print("Homing sequence failed. Ending program.")
    
    if mode.check_grbl_status:
        print("Checking GRBL status...")
        grbl_status = get_grbl_status(uno_serial_port, data_queue)
        print(grbl_status)
        next_move = grbl_status["MPos"]
    
    
    
    
else:
    print("Not connecting to Arduino Uno.")

# --- MAIN CONTROL LOOP --------------------------------------------------------
run_control_loop = True
while run_control_loop:
    flag_input_change = False
    flag_buffer_space = False
    flag_need_reset = False
    flag_send_move = False
    
    grbl_last_move = next_move.copy()
    prev_limits_hit = limits_hit.copy()
    prev_dials = dials.copy()
    prev_touch_sensors = touch_sensors.copy()
    prev_grbl_status = grbl_status.copy()
    
    print(f"Loop Start --------------- moves sent: {moves_sent}")

# --- SENSE --------------------------------------------------------------------
    
    # TODO get limit switches, put data in limits_hit = {"r_min": False,"r_max": False, "theta_zero": False}
    
    if mode.check_dials:
        print("Checking control dials...")
    else:
        print("Ignoring control dials.")
    
    if mode.check_touch_sensors:
        print("Checking touch sensors...")
    else:
        print("Ignoring touch sensors.")
    
    if mode.check_grbl_status:
        print("Checking GRBL status...")
        grbl_status = get_grbl_status(uno_serial_port, data_queue)
        print(f"Main Loop grbl_status: {grbl_status}")
        if grbl_status == None:
            print(f"Unsuccessful GRBL status request. Ending program.")
            break
        
        # Check software "limit switches"
        if grbl_status["MPos"][0] <= R_MIN:
            limits_hit["r_min"] = True
        else:
            limits_hit["r_min"] = False
        if grbl_status["MPos"][0] >= R_MAX:
            limits_hit["r_max"] = True
        else:
            limits_hit["r_max"] = False
        
        if flag_log_path:
            path_history.append(grbl_status["MPos"])
    else:
        print("Ignoring GRBL status.")
    
    # if prev_dials == dials and prev_touch_sensors == touch_sensors and prev_grbl_status == grbl_status:
    #         flag_input_change = False
    # else:
    #     flag_input_change = True
    
# --- THNK ---------------------------------------------------------------------
    
    # Check if limit hits have just been hit
    if prev_limits_hit != limits_hit:
        if limits_hit["r_min"] or limits_hit["r_max"]:
            flag_input_change = True
    
    # Check if input (sensors or dials) has changed
    if prev_dials != dials and prev_touch_sensors != touch_sensors:
        flag_input_change = False
    
    # Check if grbl's buffer has space
    if grbl_status["PlannerBuffer"] >= 1:
        # PlannerBuffer is 0 when full, 15 when empty
        # RxBuffer is 0 when full, 128 when empty
        flag_buffer_space = True
    else:
        flag_buffer_space = False
    
    # If the mode is done, cyle to next mode
    if mode.done and grbl_status["State"] == "Idle" and grbl_status["PlannerBuffer"] ==15:
        print(f"Mode {mode.mode_type} is done.")
        mode_index = (mode_index + 1) % len(modes)
        mode = modes[mode_index]
        flag_input_change = True
    
    # If an input has changed, plan to send reset 
    # and calc next move from current position
    if flag_input_change:
        print("Calculating next move from current position...")
        next_move = mode.next_move(grbl_status["MPos"], limits_hit, dials, touch_sensors, grbl_status)
    
    # If input has not changed and buffer is low, 
    # calc next move from last sent move. Otherwise, do nothing.
    elif flag_buffer_space:
        print("Calculating next move from last sent move...")
        next_move = mode.next_move(grbl_last_move, limits_hit, dials, touch_sensors, grbl_status)

            
        
# --- ACT ----------------------------------------------------------------------
    
    if flag_input_change and grbl_status["State"] != "Idle":
        print("Running flag_input_change actions...")
        run_control_loop = send_with_check(uno_serial_port, GRBL_HOLD, data_queue)
        time.sleep(0.5)
        run_control_loop = send_with_check(uno_serial_port, GRBL_SOFT_RESET, data_queue)
        time.sleep(0.5)
        run_control_loop = send_with_check(uno_serial_port, GRBL_UNLOCK, data_queue)
        time.sleep(0.5)
    
    if (flag_buffer_space or flag_input_change) and next_move is not None:
        if mode.connect_to_uno:
            run_control_loop = send_new_grbl_move(uno_serial_port, next_move, grbl_status, data_queue)
            moves_sent += 1
            if flag_log_commands:
                grbl_command_log.append(next_move)
            
    time.sleep(0.5)
# --- END OF MAIN CONTROL LOOP -------------------------------------------------

print("Control loop exited. Performing safe stop...")
send_with_check(uno_serial_port, GRBL_HOLD, data_queue)
time.sleep(0.1)
get_grbl_status(uno_serial_port, data_queue)
time.sleep(0.1)
send_with_check(uno_serial_port, GRBL_SOFT_RESET, data_queue)
time.sleep(0.1)

if flag_log_path or flag_log_commands:
    print("Saving data...")
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"table_log_{timestamp}.json"

    data_to_save = {
        "path_history": path_history,
        "grbl_command_log": grbl_command_log
    }

    with open(filename, 'w') as f:
        json.dump(data_to_save, f, indent=4)

    print(f"Data saved to {filename}")