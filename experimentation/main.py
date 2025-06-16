import serial
import time
import math
import threading
import queue
import json
import signal
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional, List, Union
from enum import Enum

from parse_grbl_status import *

@dataclass
class Move:
    r: Optional[float] = None
    t: Optional[float] = None
    s: Optional[float] = None
    received: Optional[bool] = False

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
    msg_type: Optional[str] = GrblSendMsgType.EMPTY
    msg: Optional[str] = GrblCmd.EMPTY
    move: Optional[Move] = Move()
    sent: Optional[bool] = False
    response: Optional[str] = ""
    received: Optional[bool] = False
    def __repr__(self):
        if type(self.msg) == GrblCmd:
            msg_str = self.msg.name
        else:
            msg_str = self.msg
        return f"GrblSendMsg(msg_type={self.msg_type.name}, msg={msg_str}, move={self.move}, sent={self.sent}, response={self.response}, received={self.received})"
    
class GrblRespMsg:
    msg_type: Optional[str] = GrblRespType.NONE
    msg: Optional[str] = ""
    handled: Optional[bool] = False
    def __repr__(self):
        return f"GrblRespMsg(msg_type={self.msg_type.name}, msg={self.msg}, handled={self.handled})"

@dataclass
class LimitsHit:
    soft_r_min: bool = True
    soft_r_max: bool = True
    hard_r_min: bool = True
    hard_r_max: bool = True
    theta_zero: bool = False

@dataclass
class ControlPanel:
    speed: float = 1
    brightness: float = 1

@dataclass
class Grbl:
    status: str = ""
    mpos_r: float = 0
    mpos_t: float = 0
    feed_rate: float = 0
    planner_buffer: int = 15
    rx_buffer: float = 128
    pnX: bool = False
    pnY: bool = False

@dataclass
class Flags:
    # flags that reset per loop
    input_change: bool = False
    buffer_space: bool = False
    need_reset: bool = False
    need_status: bool = False
    need_unlock: bool = False
    expecting_extra_msg = False
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
    touch_sensors: list = field(default_factory=list)
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

    last_grbl_resp: GrblRespMsg = GrblRespMsg()
    next_grbl_msg: GrblSendMsg = GrblSendMsg()

    theta_correction: float = 0 # TODO
    path_history: list = field(default_factory=list)
    grbl_command_log: list = field(default_factory=list)
    

        
    def __repr__(self):
        return (
            f"State(\n"
            f"  limits_hit={self.limits_hit},\n"
            f"  control_panel={self.control_panel},\n"
            f"  touch_sensors={self.touch_sensors},\n"
            f"  grbl={self.grbl},\n"
            f"  flags={self.flags},\n"
            f"  next_move={self.next_move},\n"
            f"  prev_limits_hit={self.prev_limits_hit},\n"
            f"  prev_control_panel={self.prev_control_panel},\n"
            f"  prev_grbl={self.prev_grbl},\n"
            f"  prev_flags={self.prev_flags},\n"
            f"  prev_move={self.prev_move},\n"
            f"  desired_linspeed={self.desired_linspeed},\n"
            f"  moves_sent={self.moves_sent},\n"
            f"  last_grbl_resp={self.last_grbl_resp},\n"
            f"  next_grbl_msg={self.next_grbl_msg}\n"
            f")"
        )


    def iterate(self):
        self.prev_limits_hit = self.limits_hit
        self.prev_control_panel = self.control_panel
        self.prev_grbl = self.grbl
        self.prev_flags = self.flags
        self.prev_move = self.next_move
        self.flags.input_change = False
        self.flags.buffer_space = False
        self.flags.need_reset = False
        self.flags.need_send_next_move = False

    def update_limits_hit(self):
        print("Updating limits hit...")
        self.limits_hit.soft_r_min = self.grbl.mpos_r <= R_MIN
        self.limits_hit.soft_r_max = self.grbl.mpos_r >= R_MAX
        self.limits_hit.hard_r_min = self.grbl.pnX
        self.limits_hit.hard_r_max = self.grbl.pnY
        print(f"Updated limits hit: {self.limits_hit}")
    
    def flags_to_setup(self):
        self.flags.in_sense_phase = True
        self.flags.in_think_phase = True
        self.flags.in_act_phase = True
        
    def flags_to_sense_phase(self):
        self.flags.in_sense_phase = True
        self.flags.in_think_phase = False
        self.flags.in_act_phase = False
        
    def flags_to_think_phase(self):
        self.flags.in_sense_phase = False
        self.flags.in_think_phase = True
        self.flags.in_act_phase = False
        
    def flags_to_act_phase(self):
        self.flags.in_sense = False
        self.flags.in_think_phase = False
        self.flags.in_act_phase = True

    def update_from_grbl_msg(self, grbl_msg):
        """Take uno serial msg and integrate into state."""
        print("Updating status from grbl msg...")
        status_dict = parse_grbl_status(grbl_msg)
        self.grbl.status = status_dict["State"]
        self.grbl.mpos_r = status_dict["MPos"][0]
        self.grbl.mpos_t = status_dict["MPos"][1]
        self.grbl.feed_rate = status_dict["FeedRate"]
        self.grbl.planner_buffer = status_dict["PlannerBuffer"]
        self.grbl.rx_buffer = status_dict["RxBuffer"]
        if "Pn" in status_dict:
            self.grbl.pnX = "X" in status_dict["Pn"]
            self.grbl.pnY = "Y" in status_dict["Pn"]
        else:
            self.grbl.pnX = False
            self.grbl.pnY = False
        self.update_limits_hit()
        print(f"Updated status: {self}")
        
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
    base_linspeed: int = 8000 #mm/min
    r_dir: int = 1
    theta_dir: int = 1
    pitch: int = 14 
    waypoints_xy: list = field(default_factory=list)
    waypoints_rt: list = field(default_factory=list)
    waypoints_i: int = 0
    done: bool = False

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
@dataclass
class SpiralMode(Mode):
    """Mode for the marble to draw a spiral outwards from its current location."""
    mode_name: str = "spiral"
        # Behavioral flags are inherited from Mode base.
        # Original `become_spiral` flags matched these defaults.
        
    def next_move(self, move_from):
        r = move_from.r
        theta = move_from.t
        if self.is_done():
            return Move()
        else:
            if r>30:
                seg_angle = 360 * self.segment_length/r*2*math.pi
            else:
                seg_angle = 360

            new_theta = theta + seg_angle*self.theta_dir
            new_r = r + self.pitch*seg_angle/360*self.r_dir

            # grbl_assumed_dist = math.sqrt((r-new_r)**2 + (theta-new_theta)**2)
            # compensation_ratio = grbl_assumed_dist/self.segment_length
            # new_speed = self.linspeed*compensation_ratio

            pi=math.pi
            r_ave = (r+new_r)/2
            new_speed = (-2*pi*r_ave + self.base_linspeed + 360) * state.control_panel.speed
            new_move = Move(r=new_r, t=new_theta, s=new_speed)
            if check_move(new_move):
                self.done = False
                return new_move
            else:
                self.done = True
                return Move()

    def is_done(self):
        if state.limits_hit.soft_r_max == True and self.r_dir == 1:
            self.done = True
        elif state.limits_hit.soft_r_min == True and self.r_dir == -1:
            self.done = True
        return self.done


        
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
    # state.flags.expecting_extra_msg = False
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
            or state.next_grbl_msg.msg == GrblCmd.UNLOCK
            or state.next_grbl_msg.msg_type == GrblSendMsgType.MOVE
            or state.next_grbl_msg.msg_type == GrblSendMsgType.SETTING
            or state.next_grbl_msg.msg_type == GrblSendMsgType.EMPTY):
            isgood = True
            if state.next_grbl_msg.msg == GrblCmd.HOME:
                state.flags.need_homing = False
        # state.flags.expecting_extra_msg = False
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
        state.flags.need_reset = False
        state.flags.need_unlock = True
        state.flags.need_status = True
    elif state.last_grbl_resp.msg_type == GrblRespType.RESP_CHECKLIMITS:
        state.flags.expecting_extra_msg = True
        state.flags.need_unlock = True
        state.flags.need_status = True
    elif state.last_grbl_resp.msg_type == GrblRespType.RESP_NEEDUNLOCK:
        state.flags.need_unlock = True
        state.flags.need_status = True
    elif state.last_grbl_resp.msg_type == GrblRespType.RESP_UNLOCKED:
        state.flags.need_unlock = False
        state.flags.need_status = True
        state.flags.expecting_extra_msg = True
    elif state.last_grbl_resp.msg_type == GrblRespType.RESP_OTHER:
        state.flags.need_status = True
    else:
        print(f"Unrecognized GRBL response: {state.last_grbl_resp.msg}")
        state.flags.need_reset = True
        state.flags.need_unlock = True
        state.flags.need_status = True
        state.flags.run_control_loop = False
    if isgood:
        state.next_grbl_msg.received = True
        state.next_grbl_msg.response = state.last_grbl_resp.msg
        if state.next_grbl_msg.msg_type == GrblSendMsgType.MOVE:
            state.next_move.received = True
            state.flags.need_send_next_move = False
        if state.next_grbl_msg.msg == GrblCmd.UNLOCK:
            state.flags.need_unlock = False
        if state.next_grbl_msg.msg == GrblCmd.SOFT_RESET:
            state.flags.need_reset = False

def format_move(move):
    return f"G1 X{move.r:.2f} Z{move.t:.2f} F{move.s:.2f}\n"

def check_move(move):
    """Check move validity based on limits and grbl status."""
    # If r limit has been hit, next move needs to be in oppsite direction
    print(f"Checking move {move}...")
    if state.limits_hit.hard_r_min and move.r < state.grbl.mpos_r:
        print(f"Requested move {move} is into the hard_r_min limit switch.")
        return False
    if state.limits_hit.hard_r_max and move.r > state.grbl.mpos_r:
        print(f"Requested move {move} is into the hard_r_max limit switch.")
        return False
    if not state.flags.need_homing:
        if state.limits_hit.soft_r_min and move.r < state.grbl.mpos_r:
            print(f"Requested move {move} is into the soft_r_min limit switch.")
            return False
        elif state.limits_hit.soft_r_max and move.r > state.grbl.mpos_r:
            print(f"Requested move {move} is into the soft_r_max limit switch.")
            return False
        elif move.r >= R_MAX or move.r < R_MIN:
            print(f"Requested move {move} is out of bounds.")
            return False
    print(f"Checks passed.")
    return True


def next_move_to_msg():
    """Check move validity based on limits and grbl status, then make next msg next_move."""
    # If r limit has been hit, next move needs to be in oppsite direction
    if state.next_move != None and not state.next_move.is_empty():
        if check_move(state.next_move):
            state.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.MOVE, msg=format_move(state.next_move))
            print(f"Next msg: {state.next_grbl_msg}")
    else:
        state.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.EMPTY)

def homing_next_msg():
    if state.limits_hit.hard_r_min:
        state.next_move = Move(r=state.grbl.mpos_r+30, t=state.grbl.mpos_t)
    elif state.limits_hit.hard_r_max:
        state.next_move = Move(r=state.grbl.mpos_r-30, t=state.grbl.mpos_t)
    else:
        state.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.HOME)
        return
    next_move_to_msg()


def gen_msg_from_state():
    """Uses state.flags and internal logic to determine which messages to send to GRBL. 
    Sets state.next_grbl_msg"""
    if state.flags.in_sense_phase:
        if (state.flags.need_reset 
            and state.last_grbl_resp.msg_type == GrblRespType.RESP_ALARM):
            state.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.SOFT_RESET)
        elif state.flags.need_status:
            state.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.STATUS)
        else:
            state.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.EMPTY)
    if state.flags.in_act_phase:
        if state.flags.need_reset:
            state.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.SOFT_RESET)
        elif state.flags.need_unlock:
            state.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.CMD, msg=GrblCmd.UNLOCK)
        elif state.flags.need_homing:
            homing_next_msg()
        elif state.flags.need_send_next_move:
            next_move_to_msg()
        else:
            state.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.EMPTY)

def grbl_write_next_msg(serial_port):
    """Converts state.next_msg to bytes and writes to serial port."""
    x = state.next_grbl_msg.msg
    if type(x) == GrblCmd:
        print(f"{time.time():.5f} | writing to grbl: {x.value}")
        serial_port.write(x.value)
    elif type(x) == str:
        print(f"{time.time():.5f} | writing to grbl: {bytes(x, 'utf-8')}")
        serial_port.write(bytes(x,  'utf-8'))
    else:
        print(f"{time.time():.5f} | writing to grbl: {x}")
        serial_port.write(x)

def run_grbl_communicator(timeout=.5):
    """Sends messages to grbl and manages state based on response. Should be the only function that main control loop runs to communicate with grbl."""
    global grbl_data_queue, uno_serial_port
    # assume all previous msgs are handled
    print("Running GRBL communicator...")
    while True:
        print(state)
        # if state.flags.expecting_extra_msg:
        #     time.sleep(0.1)
        # handle missed messages
        while not grbl_data_queue.empty():
            msg_txt = grbl_data_queue.get()
            grbl_data_queue.task_done()
            state.last_grbl_resp = grbl_resp_msg_txt_to_obj(msg_txt)
            handle_grbl_response()
        # generate state.next_grbl_msg
        gen_msg_from_state()
        # if there is still a message to send
        if (state.next_grbl_msg.msg_type != GrblSendMsgType.EMPTY):
            # wait extra long if we're running homing
            if state.next_grbl_msg.msg == GrblCmd.HOME:
                timeout = 60
            grbl_write_next_msg(uno_serial_port) # send state.next_grbl_msg
            state.next_grbl_msg.sent = True
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
        if (state.next_grbl_msg.msg_type == GrblSendMsgType.EMPTY):
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

    signal.signal(signal.SIGINT, sig_handler)
    # --- PROGRAM OPTIONS ---
    state.flags.log_commands = True
    state.flags.log_path = True
    state.flags.grbl_homing_on = True
    state.flags.connect_to_uno = True
    state.flags.connect_to_nano = False

    # --- SETUP ---
    state.flags_to_setup() # allow all types of actionsC
    state.flags.run_control_loop = True
    
    modes = [SpiralMode(mode_name="spiral out"), SpiralMode(mode_name="spiral in", r_dir=-1)]
    mode_index = 0
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
        
        print("Resetting and unlocking GRBL...")
        state.flags.need_reset = True
        state.flags.need_unlock = True
        run_grbl_communicator()

        print("Checking GRBL status...")
        state.flags.need_status = True
        run_grbl_communicator()

        if state.flags.grbl_homing_on:
            print("Starting homing...")
            state.flags.need_homing = True
            run_grbl_communicator()

            if not state.flags.run_control_loop:
                print("Homing sequence failed. Ending program.")

            print("Checking GRBL status...")
            state.flags.need_status = True
            run_grbl_communicator()
    else:
        print("Not connecting to Arduino Uno.")

# --- MAIN CONTROL LOOP --------------------------------------------------------
    while state.flags.run_control_loop:
        state.iterate()
        
        print(f"Loop Start --------------- moves sent: {state.moves_sent}")
        print(f"Current mode: {mode.mode_name}")
        print(mode)

# --- SENSE --------------------------------------------------------------------
        print("Sensing...")
        state.flags_to_sense_phase()
        if state.flags.connect_to_nano:
            if mode.need_control_panel:
                print("Checking control panel...")
            else:
                print("Ignoring control panel.")
            
            if mode.need_touch_sensors:
                print("Checking touch sensors...")
            else:
                print("Ignoring touch sensors.")
        else:
            print("Not connected to Arduino Nano - ignoring control panel and touch sensors.")
            if mode.need_control_panel or mode.need_touch_sensors:
                print(f"Skipping mode: {mode.mode_name}")
                mode = modes[(mode_index + 1) % len(modes)]
        
        if state.flags.connect_to_uno:
            if mode.need_grbl:
                print("Checking GRBL status...")
                state.flags.need_status = True
                run_grbl_communicator()
                
                if state.flags.log_path:
                    state.path_history.append([time.time(), state.grbl.mpos_r, state.grbl.mpos_t])
            else:
                print("Ignoring GRBL status.")
        else:
            print("Not connected to Arduino Uno - ignoring GRBL status.")
            if mode.need_grbl:
                print(f"Skipping mode: {mode.mode_name}")
                mode = modes[(mode_index + 1) % len(modes)]
        
# --- THNK ---------------------------------------------------------------------
        print("Thinking...")
        state.flags_to_think_phase()
        # Check if limits_hit have just been hit
        if state.prev_limits_hit != state.limits_hit:
            if state.limits_hit.soft_r_min or state.limits_hit.soft_r_max:
                state.flags.input_change = True
            if state.limits_hit.hard_r_min or state.limits_hit.hard_r_max:
                state.flags.input_change = True
                state.flags.need_homing = True
        
        # Check if input (sensors or dials) has changed
        if (state.prev_control_panel != state.control_panel 
            and state.prev_touch_sensors != state.touch_sensors):
            state.flags.input_change = True
        
        # Check if grbl's buffer has space
        if state.grbl.planner_buffer >= 1:
            # PlannerBuffer is 0 when full, 15 when empty
            # RxBuffer is 0 when full, 128 when empty
            state.flags.buffer_space = True
        
        # If the mode is done, cyle to next mode
        if (mode.is_done() 
            and state.grbl.status == GrblStatusOptions.IDLE.value 
            and state.grbl.planner_buffer == 15):
            print(f"Mode {mode.mode_name} is done.")
            print(mode)
            mode_index = (mode_index + 1) % len(modes)
            mode = modes[mode_index]
            mode.done = False
            state.flags.input_change = True
            print(f"Next mode: {mode.mode_name}")
            print(mode)
        
        # If an input has changed, plan to send reset 
        # and calc next move from current position
        if state.flags.input_change:
            state.flags.need_reset = True
            state.flags.need_send_next_move = True
            print("Calculating next move from current position...")
            state.next_move = mode.next_move(Move(r=state.grbl.mpos_r, t=state.grbl.mpos_t, s=0))
            print(f"Next move: {state.next_move}")
        
        # If input has not changed and buffer is low, 
        # calc next move from last sent move. Otherwise, do nothing.
        elif state.flags.buffer_space:
            state.flags.need_send_next_move = True
            print("Calculating next move from last sent move...")
            state.next_move = mode.next_move(state.prev_move)
            print(f"Next move: {state.next_move}")

# --- ACT ----------------------------------------------------------------------
        print("Acting...")
        state.flags_to_act_phase()
        if state.flags.run_control_loop:
            if state.flags.need_send_next_move and state.flags.log_commands:
                state.grbl_command_log.append([time.time(), state.next_move.r, state.next_move.t, state.next_move.s])
            run_grbl_communicator()
        else:
            print("Not performing grbl tasks because run_control_loop is set to False.")
        time.sleep(0.5)
    # --- END OF MAIN CONTROL LOOP -------------------------------------------------

    print("Control loop exited. Performing safe stop...")
    state.flags_to_sense_phase()
    state.flags.need_reset = True
    state.flags.need_status = True
    run_grbl_communicator()

    if state.flags.log_path or state.flags.log_commands:
        print("Saving data...")
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"table_log_{timestamp}.json"

        data_to_save = {
            "path_history": state.path_history,
            "grbl_command_log": state.grbl_command_log
        }

        with open(filename, 'w') as f:
            json.dump(data_to_save, f, indent=4)

        print(f"Data saved to {filename}")

def sig_handler(sig, frame):
    print("Program terminated by user. Sending soft reset to GRBL...")
    uno_serial_port.write(GrblCmd.SOFT_RESET.value)
    print("Done.")
    exit(0)

if __name__ == "__main__":
    main()