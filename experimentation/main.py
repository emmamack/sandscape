import serial
import time
import math
import threading
import queue
import json
from datetime import datetime
from dataclasses import dataclass
from typing import Optional, Union
from enum import Enum

from parse_grbl_status import *

@dataclass
class Move:
    r: Optional[float] = None
    t: Optional[float] = None
    s: Optional[float] = None
    done: Optional[bool] = False

    def is_empty(self):
        if self.r == None and self.t == None and self.s == None:
            return True
        else:
            return False

# --- GRBL COMMANDS ---
class GrblCmd(Enum):
    EMPTY = bytes("",  'utf-8')
    PING = bytes("\n",  'utf-8')
    STATUS = bytes("?",  'utf-8')
    HOLD = bytes("!\n",  'utf-8')
    RESUME = bytes("~\n",  'utf-8')
    SOFT_RESET = bytes([0x18])
    UNLOCK = bytes("$X\n",  'utf-8')
    HOME = bytes("$H\n",  'utf-8')
    # HARD_RESET = bytes([0x85]) ???? maybe? i don't know what this does

class GrblSendMsgType(Enum):
    EMPTY = "empty"
    CMD = "cmd"
    MOVE = "move"
    SETTING = "setting"
    
class GrblRespType(Enum):
    NONE = "none"
    RESP_OTHER = "resp_other"
    RESP_STATUS = "resp_status"
    RESP_OK = "resp_ok"
    RESP_ALARM = "resp_alarm"
    RESP_ERROR = "resp_error"
    RESP_STARTUP = "resp_startup"
    RESP_CHECKLIMITS = "resp_checklimits"
    RESP_NEEDUNLOCK = "resp_needunlock"
    RESP_UNLOCKED = "resp_unlocked"
    
class GrblStatusOptions(Enum):
    IDLE = "Idle"
    RUN = "Run"
    HOLD = "Hold"
    ALARM = "Alarm"

@dataclass
class GrblSendMsg:
    msg_type: Optional[str] = GrblSendMsgType.NONE
    msg: Optional[str] = GrblCmd.EMPTY
    move: Optional[Move] = Move()
    sent: Optional[bool] = False
    response: Optional[str] = ""
    received: Optional[bool] = False
    
class GrblRespMsg:
    msg_type: Optional[str] = GrblRespType.NONE
    msg: Optional[str] = ""
    handled: Optional[bool] = False

@dataclass
class LimitsHit:
    soft_r_min: bool = True
    soft_r_max: bool = True
    hard_r_min: bool = True
    hard_r_max: bool = True
    theta_zero: bool = False

@dataclass
class ControlPanel:
    speed: float
    brightness: float

@dataclass
class Grbl:
    status: str
    mpos_r: float
    mpos_t: float
    feed_rate: float
    planner_buffer: int
    rx_buffer: float
    pnX: bool
    pnY: bool

@dataclass
class Flags:
    # flags that reset per loop
    input_change: bool = False
    buffer_space: bool = False
    need_reset: bool = False
    need_status: False
    need_unlock: bool = False
    need_move_off_switch: bool = False
    need_homing: bool = False
    need_calc_next_move: bool = False
    need_send_next_move: bool = False
    
    # user set program settings flags
    log_commands: bool = True
    log_path: bool = True
    grbl_homing_on: bool = True
    connect_to_uno: bool = True
    connect_to_nano: bool = False
    
    # status flags
    run_control_loop: bool = False
    in_sense_phase: bool = False
    in_think_phase: bool = False
    in_act_phase: bool = False

@dataclass
class State:
    limits_hit: LimitsHit = LimitsHit()
    control_panel: ControlPanel = ControlPanel()
    grbl: Grbl = Grbl()
    flags: Flags = Flags()
    next_move: Move = Move()

    prev_limits_hit: LimitsHit = LimitsHit()
    prev_control_panel: ControlPanel = ControlPanel()
    prev_grbl: Grbl = Grbl()
    prev_flags: Flags = Flags()
    prev_move: Move = Move()

    desired_linspeed: int = 3000 #mm/min
    moves_sent: int = 0
    path_history: list = []
    grbl_command_log: list = []
    
    theta_correction: float = 0 # TODO
    
    last_grbl_resp: GrblRespMsg = GrblRespMsg()
    next_grbl_msg: GrblSendMsg = GrblSendMsg()

    def iterate(self):
        self.prev_limits_hit = self.limits_hit
        self.prev_control_panel = self.control_panel
        self.prev_grbl = self.grbl
        self.prev_flags = self.flags
        self.prev_move = self.next_move
        self.flags.input_change = False
        self.flags.buffer_space = False
        self.flags.need_reset = False
        self.flags.send_next_move = False

    def update_limits_hit(self):
        self.soft_r_min = self.grbl.mpos_r <= R_MIN
        self.soft_r_max = self.grbl.mpos_r >= R_MAX
        self.hard_r_min = self.grbl.pnX
        self.hard_r_max = self.grbl.pnY

    def update_from_grbl_msg(self, grbl_msg):
        """Take uno serial msg and integrate into state."""
        status_dict = parse_grbl_status(grbl_msg)
        self.status = status_dict["State"]
        self.mpos_r = status_dict["MPos"][0]
        self.mpos_t = status_dict["MPos"][1]
        self.feed_rate = status_dict["FeedRate"]
        self.planner_buffer = status_dict["PlannerBuffer"]
        self.rx_buffer = status_dict["RxBuffer"]
        if "Pn" in status_dict:
            self.pnX = "X" in status_dict["Pn"]
            self.pnY = "Y" in status_dict["Pn"]
        else:
            self.pnX = False
            self.pnY = False
        self.update_limits_hit()
        
    def update_from_nano_msg(self, nano_msg):
        """TODO: Take nano serial msg and integrate into state."""
        pass

@dataclass
class Mode:
    """
    Base class for different operational modes.
    Subclasses define specific behaviors and configurations.
    """
    mode_name: str = "base"  # Should be overridden by subclasses
    segment_length: int = 5
    base_linspeed: int = 3000 #mm/min
    r_dir: int = 1
    theta_dir: int = 1
    pitch: int = 14 
    waypoints_xy: list = []
    waypoints_rt: list = []
    waypoints_i: int = 0

    need_touch_sensors: bool = False
    need_control_panel: bool = False
    need_grbl: bool = True
    
    def next_move(self, move_from):
        print("Base next_move method called. Override in subclass.")
        return Move()

    def is_done(self):
        """
        Check if the mode is done if this mode has an end. Here in case any instantanious checks need to be made to determine if done.
        """
        return self.done
    
    def update(self):
        """
        To be called once per loop regardless if move is needed. 
        Override in subclass if mode needs to track position
        """
        pass

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

@dataclass
class WaypointMode(Mode):
    mode_name: str = "waypoint"
    
    def next_move(self, move_from):
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
    mode_name: str = "spiral"
        # Behavioral flags are inherited from Mode base.
        # Original `become_spiral` flags matched these defaults.
        
    def next_move(self, move_from):
        r = move_from.r
        theta = move_from.t

        if state.limits_hit.soft_r_max == True and self.r_dir == 1:
            self.done = True
            return None
        elif state.limits_hit.soft_r_min == True and self.r_dir == -1:
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
    mode_name = "reactive only"

class ReactiveSpiralRippleMode(Mode):
    """
    Mode for the marble to draw a spiral that can be affected by touch sensor activation.
    (Behavioral flags for this mode were not fully defined in the original implementation)
    """
    mode_name = "reactive spiral ripple"


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

def read_from_port(serial_port, stop_event, data_queue):
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
            line = serial_port.readline()

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
    
def send_with_check(serial_port, x, grbl_data_queue, q_timeout=5):
    # make sure the queue is empty
    while not grbl_data_queue.empty():
        missed_msg = grbl_data_queue.get()
        # print(f"missed message: {missed_msg}")
    if grbl_data_queue.empty():
        write_line(serial_port, x)
        try:
            response = grbl_data_queue.get(timeout=q_timeout)
            print(f"Re: {x} -> {response}")
            if "ok" in response:
                grbl_data_queue.task_done()
                return True
            elif "error" in response:
                grbl_data_queue.task_done()
                return False
            elif "<" in response and ">" in response:
                grbl_data_queue.task_done()
                return True
            elif (   "Grbl 1.1h ['$' for help]" in response 
                  or "[MSG:'$H'|'$X' to unlock]" in response
                  or "ALARM:3" in response 
                  or "[MSG:Caution: Unlocked]" in response):
                grbl_data_queue.task_done()
                if x == GRBL_SOFT_RESET or x == GRBL_UNLOCK or x == GRBL_HOLD:
                    return True
                else:
                    return False
            else:
                print(f"UNEXPECTED REPONSE - Re: {x} -> {response}")
                grbl_data_queue.task_done()
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
    if state.limits_hit.soft_r_min and r < grbl_status["MPos"][0]:
        print("Requested move is into the r_min limit switch.")
        return False
    elif state.limits_hit.soft_r_max and r > grbl_status["MPos"][0]:
        print("Requested move is into the r_max limit switch.")
        return False
    elif r >= R_MAX or r < R_MIN:
        print("Requested move is out of bounds.")
        return False
    else:
        print("Requested move is valid.")
        return True

def get_grbl_status(serial_port, grbl_data_queue, reattempts=5):
    # make sure the queue is empty
    for _ in range(reattempts):
        while not grbl_data_queue.empty():
            missed_msg = grbl_data_queue.get()
            print(f"missed message: {missed_msg}")
        if grbl_data_queue.empty():
            write_line(serial_port,"?")
            try:
                response = grbl_data_queue.get(timeout=5)
                if "<" in response and ">" in response:
                    status_dict = parse_grbl_status(response)
                    grbl_data_queue.task_done()
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

def homing_sequence(serial_port, grbl_data_queue, grbl_status, limits_hit):
    q_timeout=60
    r = grbl_status["MPos"][0]
    t = grbl_status["MPos"][1]
    # if hard limits are hit
    if state.limits_hit.hard_r_min or state.limits_hit.hard_r_max:
        # reset and unlock to leave alarm state
        send_with_check(serial_port, GRBL_SOFT_RESET, grbl_data_queue)
        time.sleep(.5)
        send_with_check(serial_port, GRBL_UNLOCK, grbl_data_queue)
        time.sleep(.5)
        # move away from switch
        if state.limits_hit.hard_r_min:
            print("Moving off r_min switch before homing...")
            success = send_new_grbl_move(serial_port,[r+30, t, 100], grbl_status, grbl_data_queue, override=True)
            if not success:
                return False
        if state.limits_hit.hard_r_max:
            print("Moving off r_max switch before homing...")
            success = send_new_grbl_move(serial_port,[r-30, t, 100], grbl_status, grbl_data_queue, override=True)
            if not success:
                return False
        time.sleep(.1)
        while grbl_status["State"] != "Idle":
            time.sleep(.5)
            grbl_status = get_grbl_status(serial_port, grbl_data_queue)
            if grbl_status == None:
                send_with_check(serial_port, GRBL_SOFT_RESET, grbl_data_queue)
                time.sleep(.5)
                send_with_check(serial_port, GRBL_UNLOCK, grbl_data_queue)
                time.sleep(.5)
                grbl_status = get_grbl_status(serial_port, grbl_data_queue)
    
    # make sure the queue is empty
    while not grbl_data_queue.empty():
        missed_msg = grbl_data_queue.get()
        # print(f"missed message: {missed_msg}")
    if grbl_data_queue.empty():
        write_line(serial_port, GRBL_HOME)
        start_time = time.time()
        end_time = start_time + q_timeout
        print("Homing...", end="", flush=True)
        while time.time() < end_time:
            if grbl_data_queue.empty():
                time.sleep(1)
                print(".", end="", flush=True)
            else:
                print()
                try:
                    response = grbl_data_queue.get(timeout=q_timeout)
                    print(f"Re: {GRBL_HOME} -> {response}")
                    if "ok" in response:
                        grbl_data_queue.task_done()
                        return True
                    else:
                        print(f"UNEXPECTED RESPONSE - Re: {GRBL_HOME} -> {response}")
                        grbl_data_queue.task_done()
                        return False
                except queue.Empty:
                    print(f"NO RESPONSE to {GRBL_HOME}.")

def grbl_resp_msg_txt_to_obj(msg_txt):
    msg_obj = GrblRespMsg()
    msg_obj.msg = msg_txt
    # determine response type
    if "<" in msg_txt and ">" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_STATUS
    elif "ok" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_OK
    elif "ALARM" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_ALARM
    elif "error" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_ERROR
    elif "Grbl 1.1h ['$' for help]" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_STARTUP
    elif "[MSG:Check Limits]" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_CHECKLIMITS
    elif "[MSG:'$H'|'$X' to unlock]" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_NEEDUNLOCK
    elif "[MSG:Caution: Unlocked]" in msg_txt:
        msg_obj.msg_type = GrblRespType.RESP_UNLOCKED
    else:
        msg_obj.msg_type = GrblRespType.RESP_OTHER
    return msg_obj

def handle_grbl_response():
    """Sets state.flags in state based on state.last_grbl_resp."""
    isgood = False
    state.next_grbl_msg.response = state.last_grbl_resp.msg
    if state.last_grbl_resp.msg_type == GrblRespType.RESP_STATUS:
        if state.next_grbl_msg.msg == GrblCmd.STATUS:
            isgood = True
            state.flags.need_status = False
            state.update_from_grbl_msg(state.last_grbl_resp.msg)
    elif state.last_grbl_resp.msg_type == GrblRespType.RESP_OK:
        if (state.next_grbl_msg.msg == GrblCmd.PING 
            or state.next_grbl_msg.msg == GrblCmd.HOME
            or state.next_grbl_msg.msg == GrblCmd.HOLD
            or state.next_grbl_msg.msg == GrblCmd.RESUME
            or state.next_grbl_msg.msg_type == GrblSendMsgType.MOVE
            or state.next_grbl_msg.msg_type == GrblSendMsgType.SETTING):
            isgood = True
    elif state.last_grbl_resp.msg_type == GrblRespType.RESP_ALARM:
        state.grbl.status = GrblStatusOptions.ALARM
        state.flags.need_reset = True
        state.flags.need_unlock = True
        state.flags.need_status = True
    elif state.last_grbl_resp.msg_type == GrblRespType.RESP_ERROR:
        state.flags.need_reset = True
        state.flags.need_unlock = True
        state.flags.need_status = True
        state.flags.run_control_loop = False
    elif state.last_grbl_resp.msg_type == GrblRespType.RESP_STARTUP:
        state.flags.need_reset = True
        state.flags.need_unlock = True
        state.flags.need_status = True
    elif state.last_grbl_resp.msg_type == GrblRespType.RESP_CHECKLIMITS:
        state.flags.need_unlock = True
        state.flags.need_status = True
    elif state.last_grbl_resp.msg_type == GrblRespType.RESP_NEEDUNLOCK:
        state.flags.need_unlock = True
        state.flags.need_status = True
    elif state.last_grbl_resp.msg_type == GrblRespType.RESP_UNLOCKED:
        state.flags.need_status = True
    elif state.last_grbl_resp.msg_type == GrblRespType.RESP_OTHER:
        state.flags.need_status = True
            
    if isgood:
        state.next_grbl_msg.received = True
        state.next_grbl_msg.response = state.last_grbl_resp.msg
        if state.next_grbl_msg.msg_type == GrblSendMsgType.MOVE:
            state.next_move.done = True

def gen_msg_from_state():
    """Uses state.flags and internal logic to determine which messages to send to GRBL. 
    Sets state.next_grbl_msg"""
    
def grbl_write_next_msg(serial_port):
    """Converts state.next_msg to bytes and writes to serial port."""
    x = state.next_grbl_msg.msg
    if type(x) == str:
        print(f"{time.time():.5f} | writing to grbl: {bytes(x, 'utf-8')}")
        serial_port.write(bytes(x,  'utf-8'))
    else:
        print(f"{time.time():.5f} | writing to grbl: {x}")
        serial_port.write(x)

def run_grbl_communicator(timeout=2):
    """Sends messages to grbl and manages state based on response. Should be the only function that main control loop runs to communicate with grbl."""
    global grbl_data_queue, uno_serial_port
    # assume all previous msgs are handled
    # if there is a message to send
    while True:
        # handle missed messages
        while not grbl_data_queue.empty():
            msg_txt = grbl_data_queue.get()
            grbl_data_queue.task_done()
            state.last_grbl_resp = grbl_resp_msg_txt_to_obj(msg_txt)
            handle_grbl_response()
        # generate state.next_grbl_msg
        gen_msg_from_state()
        # while there is still a message to send

        # wait extra long if we're running homing
        if state.next_grbl_msg.msg == GrblCmd.HOME:
            timeout = 60
        grbl_write_next_msg(uno_serial_port) # send state.next_grbl_msg
        start_time = time.time()
        end_time = start_time + timeout
        while True:
            if time.time() > end_time:
                state.last_grbl_resp = GrblRespMsg()
                break
            if grbl_data_queue.empty():
                time.sleep(0.02)
            else:
                msg_txt = grbl_data_queue.get()
                grbl_data_queue.task_done()
                state.last_grbl_resp = grbl_resp_msg_txt_to_obj(msg_txt)
                break
        handle_grbl_response()
        gen_msg_from_state() # generates empty msg if no further com needed
        if state.next_grbl_msg.msg_type != GrblSendMsgType.EMPTY:
            break

# === MAIN CONTROL SCRIPT ======================================================

# --- ARDUINO UNO MOTOR CONTROLLER CONFIGURATION ---
UNO_SERIAL_PORT_NAME = 'COM8'
UNO_BAUD_RATE = 115200

# --- ARDUINO NANO I/O CONTROLLER CONFIGURATION ---
NANO_SERIAL_PORT_NAME = 'COM4' #TODO
NANO_BAUD_RATE = 9600

# --- SAND TABLE INFORMATION ---
TICKS_PER_MM_R = 40 # not used here, set in GRBL $100=40
TICKS_PER_DEG_THETA = 22.222 # not used here, set in GRBL $102=22.222
DISH_RADIUS_MM = 280
MARBLE_DIAMETER_MM = 14
MARBLE_WAKE_PITCH_MM = 14
R_MIN = 0
R_MAX = DISH_RADIUS_MM - MARBLE_DIAMETER_MM/2


state = State()

def main():
    global uno_serial_port, grbl_data_queue
    # state is already global


    # --- PROGRAM OPTIONS ---
    state.flags.log_commands = True
    state.flags.log_path = True
    state.flags.grbl_homing_on = True
    state.flags.connect_to_uno = False
    state.flags.connect_to_nano = True

    # --- SETUP ---

    modes = [WaypointMode(), SpiralMode()]
    mode_index = 1
    mode = modes[mode_index]

    stop_event = threading.Event()

    if state.flags.connect_to_nano:
        print("Establishing serial connection to Arduino Nano...")
        nano_serial_port = serial.Serial(NANO_SERIAL_PORT_NAME, NANO_BAUD_RATE, timeout=1)
        time.sleep(2) # Wait for connection to establish!
        
        # Set up sensor monitoring thread
        sensor_data_queue = queue.Queue()
        sensor_reader_thread = threading.Thread(target=read_from_port, args=(nano_serial_port, stop_event, sensor_data_queue))
        sensor_reader_thread.daemon = True
        sensor_reader_thread.start()
        time.sleep(2)
    else:
        print("Not connecting to Arduino Nano.")

    if state.flags.connect_to_uno:
        print("Establishing serial connection to Arduino Uno...")
        uno_serial_port = serial.Serial(UNO_SERIAL_PORT_NAME, UNO_BAUD_RATE, timeout=1)
        time.sleep(2) # Wait for connection to establish!
        
        # Set up grbl monitoring thread
        grbl_data_queue = queue.Queue()
        grbl_reader_thread = threading.Thread(target=read_from_port, args=(uno_serial_port, stop_event, grbl_data_queue))
        grbl_reader_thread.daemon = True
        grbl_reader_thread.start()
        time.sleep(2)
        
        state.flags.run_control_loop = send_with_check(uno_serial_port, GrblCmd.SOFT_RESET, grbl_data_queue)
        time.sleep(2)

        print("Checking GRBL status...")
        state.flags.need_status = True
        run_grbl_communicator()

        if state.flags.grbl_homing_on:
            print("Sending homing sequence...")
            state.flags.run_control_loop = homing_sequence(uno_serial_port, grbl_data_queue, grbl_status, limits_hit)
            if not state.flags.run_control_loop:
                print("Homing sequence failed. Ending program.")

            print("Checking GRBL status...")
            grbl_status = get_grbl_status(uno_serial_port, grbl_data_queue)
            print(grbl_status)
            next_move = grbl_status["MPos"]
    else:
        print("Not connecting to Arduino Uno.")

    # --- MAIN CONTROL LOOP --------------------------------------------------------
    state.flags.run_control_loop = True
    while state.flags.run_control_loop:
        state.iterate()
        
        print(f"Loop Start --------------- moves sent: {moves_sent}")

    # --- SENSE --------------------------------------------------------------------
        
        if mode.need_control_panel:
            print("Checking control panel...")
        else:
            print("Ignoring control panel.")
        
        if mode.need_touch_sensors:
            print("Checking touch sensors...")
        else:
            print("Ignoring touch sensors.")
        
        if mode.need_grbl:
            print("Checking GRBL status...")
            grbl_status = get_grbl_status(uno_serial_port, grbl_data_queue)
            print(f"Main Loop grbl_status: {grbl_status}")
            if grbl_status == None:
                print(f"Unsuccessful GRBL status request. Ending program.")
                break
            
            # Check software and hardware "limit switches"
            if grbl_status["MPos"][0] <= R_MIN:
                state.limits_hit.soft_r_min = True
            else:
                state.limits_hit.soft_r_min = False
            if grbl_status["MPos"][0] >= R_MAX:
                state.limits_hit.soft_r_max = True
            else:
                state.limits_hit.soft_r_max = False
                
            
            if "Pn" in grbl_status:
                if "X" in grbl_status["Pn"]:
                    state.limits_hit.hard_r_min = True
                else:
                    state.limits_hit.hard_r_min = False
                if "Y" in grbl_status["Pn"]: # R_max lim is wired to board Y+
                    state.limits_hit.hard_r_max = True
                else:
                    state.limits_hit.hard_r_max = False
            else:
                state.limits_hit.hard_r_min = False
                state.limits_hit.hard_r_max = False
            
            if flag_log_path:
                path_history.append(grbl_status["MPos"])
        else:
            print("Ignoring GRBL status.")
        
    # --- THNK ---------------------------------------------------------------------

        # Check if limit hits have just been hit
        if state.prev_limits_hit != state.limits_hit:
            if state.limits_hit.soft_r_min or state.limits_hit.soft_r_max:
                state.flag_input_change = True
        
        # Check if input (sensors or dials) has changed
        if (state.prev_control_panel != state.control_panel 
            and state.prev_touch_sensors != state.touch_sensors):
            flag_input_change = True
        
        # Check if grbl's buffer has space
        if state.grbl.planner_buffer >= 1:
            # PlannerBuffer is 0 when full, 15 when empty
            # RxBuffer is 0 when full, 128 when empty
            state.flag_buffer_space = True
        
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
            state.flags.run_control_loop = send_with_check(uno_serial_port, GRBL_SOFT_RESET, grbl_data_queue)
            time.sleep(0.05)
            state.flags.run_control_loop = send_with_check(uno_serial_port, GRBL_UNLOCK, grbl_data_queue)
            time.sleep(0.05)
            
        if state.limits_hit.hard_r_min or state.limits_hit.hard_r_max:
            if flag_grbl_homing_on:
                print("Hardware limits hit. Rehoming.")
                state.flags.run_control_loop = homing_sequence(uno_serial_port, grbl_data_queue, grbl_status, limits_hit)
                next_move = None # do not move, restart loop from beginning
            else:
                state.flags.run_control_loop = False
                print("Hardware limits hit. Ending program.")
                break

        
        if ((flag_buffer_space or flag_input_change) 
            and next_move is not None
            and state.flags.run_control_loop):
            if mode.connect_to_uno:
                state.flags.run_control_loop = send_new_grbl_move(uno_serial_port, next_move, grbl_status, grbl_data_queue)
                moves_sent += 1
                if flag_log_commands:
                    grbl_command_log.append(next_move)
                
        time.sleep(0.5)
    # --- END OF MAIN CONTROL LOOP -------------------------------------------------

    print("Control loop exited. Performing safe stop...")
    send_with_check(uno_serial_port, GRBL_HOLD, grbl_data_queue)
    time.sleep(0.1)
    get_grbl_status(uno_serial_port, grbl_data_queue)
    time.sleep(0.1)
    send_with_check(uno_serial_port, GRBL_SOFT_RESET, grbl_data_queue)
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

if __name__ == "__main__":
    main()