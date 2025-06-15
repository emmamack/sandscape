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

        if limits_hit["soft_r_max"] == True and self.r_dir == 1:
            self.done = True
            return None
        elif limits_hit["soft_r_min"] == True and self.r_dir == -1:
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


class State:
    def __init__(self):
        self.limits_hit = {
            "soft_r_min": True,
            "soft_r_max": True,
            "hard_r_min": True,
            "hard_r_max": True,
            "theta_zero": True}
        self.dials = {"speed": 0.5,
                "brightness": 0.5}
        self.touch_sensors = [False, False, False, False, False, False, False, False,
                        False, False, False, False, False, False, False, False]
        self.grbl_status = {'State': None,
                    'MPos': [None,None],
                    'WPos': [None,None], # not used
                    'FeedRate': None,
                    'PlannerBuffer': None,
                    'RxBuffer': None,
                    'Pn': None, # limit switches
                    'SpindleSpeed': None, # not used
                    'FeedOverride': None, # not used
                    'RapidOverride': None, # not used
                    'SpindleOverride': None} # not used

        self.next_move = [] # format [r (X), theta (Z), speed (F)]
        self.grbl_command_log = [] # only filled if flag_log_commands = True
        self.moves_sent = 0
        self.path_history = []
        self.flags = {
            "flag_input_change": False,
            "flag_buffer_space": False,
            "flag_need_reset": False,
            "flag_send_move": False,
            "flag_log_commands": True,
            "flag_log_path": True,
            "flag_grbl_homing_on": True,
            "flag_connect_to_uno": True,
            "flag_connect_to_nano": False,
        }
        
        self.prev_move = next_move.copy()
        self.prev_limits_hit = limits_hit.copy()
        self.prev_dials = dials.copy()
        self.prev_touch_sensors = touch_sensors.copy()
        self.prev_grbl_status = grbl_status.copy()
        


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

def read_from_port(ser, stop_event, data_queue):
    """
    Reads data from the serial port line by line in a dedicated thread.

    Args:
        ser (serial.Serial): The initialized serial port object.
        stop_event (threading.Event): The event to signal the thread to stop.
    """
    last_msg_timestamp = time.time()
    time_since_last_msg = time.time() - last_msg_timestamp
    last_msg_timestamp = time.time()
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
                time_since_last_msg = time.time() - last_msg_timestamp
                last_msg_timestamp = time.time()
                decoded_line = line.decode('utf-8', errors='ignore').strip()
                print(f"{time.time():.5f} | {time_since_last_msg:.5f}s | Received: {decoded_line}")
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

def write_line(serial_port, x):
    if type(x) == str:
        print(f"{time.time():.5f} | writing: {bytes(x, 'utf-8')}")
        serial_port.write(bytes(x,  'utf-8'))
    else:
        print(f"{time.time():.5f} | writing: {x}")
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

    
def send_new_grbl_move(serial_port, r_theta_speed, grbl_status, queue, override=False):
    """Send a new command to GRBL after checking it won't drive more into limit switches."""
    if check_move(r_theta_speed, limits_hit, grbl_status) or override:
        return send_with_check(serial_port, f"G1 X{r_theta_speed[0]:.2f} Z{r_theta_speed[1]:.2f} F{r_theta_speed[2]:.2f}\n", queue)
    else:
        print("Requested move {r_theta_speed} was not sent because it failed checks.")
        return False
    
def check_move(r_theta_speed, limits_hit, grbl_status):
    """Check move validity based on limits and grbl status."""
    r = r_theta_speed[0]
    # If r limit has been hit, next move needs to be in oppsite direction
    if limits_hit["soft_r_min"] and r < grbl_status["MPos"][0]:
        print("Requested move is into the r_min limit switch.")
        return False
    elif limits_hit["soft_r_max"] and r > grbl_status["MPos"][0]:
        print("Requested move is into the r_max limit switch.")
        return False
    elif r >= R_MAX or r < R_MIN:
        print("Requested move is out of bounds.")
        return False
    else:
        print("Requested move is valid.")
        return True

def get_grbl_status(serial_port, data_queue, reattempts=5):
    # make sure the queue is empty
    for _ in range(reattempts):
        while not data_queue.empty():
            missed_msg = data_queue.get()
            print(f"missed message: {missed_msg}")
        if data_queue.empty():
            write_line(serial_port,"?")
            try:
                response = data_queue.get(timeout=5)
                if "<" in response and ">" in response:
                    status_dict = parse_grbl_status(response)
                    data_queue.task_done()
                    if status_dict == None:
                        print(f"ISSUE WITH STATUS PARSING - Re: ? ->: {response}")
                    else:
                        path_history.append(status_dict["MPos"])
                        return status_dict
                else:
                    print(f"UNEXPECTED RESPONSE - Re: ? ->: {response}")
            except queue.Empty:
                print(f"No status message received")
        time.sleep(.1)

def homing_sequence(serial_port, data_queue, grbl_status, limits_hit):
    q_timeout=60
    r = grbl_status["MPos"][0]
    t = grbl_status["MPos"][1]
    # if hard limits are hit
    if limits_hit["hard_r_min"] or limits_hit["hard_r_max"]:
        # reset and unlock to leave alarm state
        send_with_check(serial_port, GRBL_SOFT_RESET, data_queue)
        time.sleep(.5)
        send_with_check(serial_port, GRBL_UNLOCK, data_queue)
        time.sleep(.5)
        # move away from switch
        if limits_hit["hard_r_min"]:
            print("Moving off r_min switch before homing...")
            success = send_new_grbl_move(serial_port,[r+30, t, 100], grbl_status, data_queue, override=True)
            if not success:
                return False
        if limits_hit["hard_r_max"]:
            print("Moving off r_max switch before homing...")
            success = send_new_grbl_move(serial_port,[r-30, t, 100], grbl_status, data_queue, override=True)
            if not success:
                return False
        time.sleep(.1)
        while grbl_status["State"] != "Idle":
            time.sleep(.5)
            grbl_status = get_grbl_status(serial_port, data_queue)
            if grbl_status == None:
                send_with_check(serial_port, GRBL_SOFT_RESET, data_queue)
                time.sleep(.5)
                send_with_check(serial_port, GRBL_UNLOCK, data_queue)
                time.sleep(.5)
                grbl_status = get_grbl_status(serial_port, data_queue)
    
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
flag_connect_to_uno = True
flag_connect_to_nano = False

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
GRBL_UNLOCK = bytes("$X\n",  'utf-8')
GRBL_HOME = bytes("$H\n",  'utf-8')
# GRBL_HARD_RESET = bytes([0x85]) ???? maybe? i don't know what this does

# --- SETUP ---

limits_hit = {
    "soft_r_min": True,
    "soft_r_max": True,
    "hard_r_min": True,
    "hard_r_max": True,
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
               'Pn': None, # limit switches
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

if flag_connect_to_nano:
    print("Establishing serial connection to Arduino Nano...")
else:
    print("Not connecting to Arduino Nano.")

if flag_connect_to_uno:
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
    if "Pn" in grbl_status:
        if "X" in grbl_status["Pn"]:
            limits_hit["hard_r_min"] = True
        else:
            limits_hit["hard_r_min"] = False
        if "Y" in grbl_status["Pn"]: # R_max lim is wired to board Y+
            limits_hit["hard_r_max"] = True
        else:
            limits_hit["hard_r_max"] = False
    else:
        limits_hit["hard_r_min"] = False
        limits_hit["hard_r_max"] = False

    if flag_grbl_homing_on:
        print("Sending homing sequence...")
        run_control_loop = homing_sequence(uno_serial_port, data_queue, grbl_status, limits_hit)
        if not run_control_loop:
            print("Homing sequence failed. Ending program.")

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
        
        # Check software and hardware "limit switches"
        if grbl_status["MPos"][0] <= R_MIN:
            limits_hit["soft_r_min"] = True
        else:
            limits_hit["soft_r_min"] = False
        if grbl_status["MPos"][0] >= R_MAX:
            limits_hit["soft_r_max"] = True
        else:
            limits_hit["soft_r_max"] = False
            
        
        if "Pn" in grbl_status:
            if "X" in grbl_status["Pn"]:
                limits_hit["hard_r_min"] = True
            else:
                limits_hit["hard_r_min"] = False
            if "Y" in grbl_status["Pn"]: # R_max lim is wired to board Y+
                limits_hit["hard_r_max"] = True
            else:
                limits_hit["hard_r_max"] = False
        else:
            limits_hit["hard_r_min"] = False
            limits_hit["hard_r_max"] = False
        
        if flag_log_path:
            path_history.append(grbl_status["MPos"])
    else:
        print("Ignoring GRBL status.")
    
# --- THNK ---------------------------------------------------------------------

    # Check if limit hits have just been hit
    if prev_limits_hit != limits_hit:
        if limits_hit["soft_r_min"] or limits_hit["soft_r_max"]:
            flag_input_change = True
    
    # Check if input (sensors or dials) has changed
    if prev_dials != dials and prev_touch_sensors != touch_sensors:
        flag_input_change = True
    
    # Check if grbl's buffer has space
    if grbl_status["PlannerBuffer"] >= 1:
        # PlannerBuffer is 0 when full, 15 when empty
        # RxBuffer is 0 when full, 128 when empty
        flag_buffer_space = True
    
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
        run_control_loop = send_with_check(uno_serial_port, GRBL_SOFT_RESET, data_queue)
        time.sleep(0.05)
        run_control_loop = send_with_check(uno_serial_port, GRBL_UNLOCK, data_queue)
        time.sleep(0.05)
        
    if limits_hit["hard_r_min"] or limits_hit["hard_r_max"]:
        if flag_grbl_homing_on:
            print("Hardware limits hit. Rehoming.")
            run_control_loop = homing_sequence(uno_serial_port, data_queue, grbl_status, limits_hit)
            next_move = None # do not move, restart loop from beginning
        else:
            run_control_loop = False
            print("Hardware limits hit. Ending program.")
            break

    
    if ((flag_buffer_space or flag_input_change) 
        and next_move is not None
        and run_control_loop):
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
