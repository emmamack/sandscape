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

import debugger_window

from parse_grbl_status import *
from parse_svg import SVGParser, create_polar_plot

EXPECTED_SENSOR_BYTESTRING_LENGTH = 17

@dataclass
class Move:
    r: Optional[float] = None
    t: Optional[float] = None
    s: Optional[float] = None
    received: Optional[bool] = False
    t_grbl: Optional[float] = None

    def is_empty(self):
        if self.r == None and self.t == None and self.s == None:
            return True
        else:
            return False
        
    def __repr__(self):
        r_str = "None" if self.r == None else f"{self.r:.3f}"
        t_str = "None" if self.t == None else f"{self.t:.3f}"
        s_str = "None" if self.s == None else f"{self.s:.3f}"
        t_str = "None" if self.t_grbl == None else f"{self.t:.3f}"
        return f"Move(r={r_str}, t={t_str}, s={s_str}, t_grbl={t_str}, received={self.received})"

@dataclass
class CartesianPt:
    x: int
    y: int

    def to_tuple(self):
        return (self.x, self.y)

@dataclass
class PolarPt:
    r: float
    t: float

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
    move: Optional[Move] = field(default_factory=lambda: Move())
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
    input_change: bool = True
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
    send_grbl_settings: bool = False
    
    # status flags
    run_control_loop: bool = False
    in_setup_phase: bool = False
    in_sense_phase: bool = False
    in_think_phase: bool = False
    in_act_phase: bool = False

@dataclass
class State:
    limits_hit: LimitsHit = field(default_factory=lambda: LimitsHit())
    control_panel: ControlPanel = field(default_factory=lambda: ControlPanel())
    touch_sensors: list = field(default_factory=list)
    grbl: Grbl = field(default_factory=lambda: Grbl())
    flags: Flags = field(default_factory=lambda: Flags())
    next_move: Move = Move(r=0,t=0)

    prev_limits_hit: LimitsHit = LimitsHit()
    prev_control_panel: ControlPanel = ControlPanel()
    prev_touch_sensors: list = field(default_factory=list)
    prev_grbl: Grbl = Grbl()
    prev_flags: Flags = Flags()
    prev_move: Move = Move(r=0,t=0)

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
        self.prev_touch_sensors = self.touch_sensors
        self.prev_grbl = self.grbl
        self.prev_flags = self.flags
        if self.next_move.received:
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
    base_linspeed: int = 9000 #mm/min
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

    def __post_init__(self):
        self.startup()
        
    def set_next_speed(self):
        if state.next_move.r != None:
            state.next_move.s = (-2*math.pi*state.next_move.r + self.base_linspeed + 360) * state.control_panel.speed
    
    def startup(self):
        """To be called once when the mode is started, for operations like path-precalculation or other initialization. Override in subclass if needed."""
        pass
    
    def next_move(self, move_from):
        """Calculate and return the next move for this mode. Calulate based on the move_from.r and move_from.t in the Move() object given, assuming that all other state parameters will stay the same."""
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
        """Returns a list containing an instance of each working mode."""
        # Order matches the original mode_types_list implicitly
        return [
            SpiralMode(),
            SpikyBallMode(),
            SVGMode(),
            # ReactiveOnlyDirectMode(),
            # ReactiveSpiralRippleMode()
        ]

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
                seg_angle = 60 * self.segment_length/r*2*math.pi
            else:
                seg_angle = 60

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

SENSOR_THETA_MASK = [i*22.5 for i in range(16)]

@dataclass
class ReactiveOnlyDirectMode(Mode):
    """Mode where the marble only moves due to touch sensor activation.
    Marble goes directly towards hand, as opposed to in the cardinal direction
    of the touch."""
    mode_name: str = "reactive only"
    need_touch_sensors: bool = True

    def next_move(self, move_from):
        r = move_from.r
        theta = move_from.t

        speed = 10 # TODO: this is a placeholder

        selected_thetas = [j for i,j in zip(state.touch_sensors, SENSOR_THETA_MASK) if i==1]
        print(selected_thetas)
        if selected_thetas == []: # no touch, so no move
            return Move()
        
        avg_theta = sum(selected_thetas) / len(selected_thetas)
        r1, t1 = (280, avg_theta)
        print(r1, t1)

        x0, y0 = polar_to_cartesian_non_object(r, theta)
        x1, y1 = polar_to_cartesian_non_object(r1, t1)

        (dir_x, dir_y)  = ((x1 - x0), (y1 - y0))
        len_dir_vector = math.sqrt(dir_x**2 + dir_y**2)
        (x_to_travel, y_to_travel) = (dir_x*speed/len_dir_vector, dir_y*speed/len_dir_vector)
        (x_next, y_next) = (x0 + x_to_travel, y0 + y_to_travel)

        print(x_next, y_next)

        r_next, t_next = cartesian_to_polar_non_object(x_next, y_next)

        t_next = t_next % 360

        print(r_next, t_next)
        
        return Move(r=r_next, t=t_next, s=3000)

@dataclass
class ReactiveSpiralRippleMode(Mode):
    """
    Mode for the marble to draw a spiral that can be affected by touch sensor activation.
    (Behavioral flags for this mode were not fully defined in the original implementation)
    """
    mode_name: str = "reactive spiral ripple"

@dataclass
class SpikyBallMode(Mode):
    mode_name: str = "spiky ball"
    width_step: int = 4

    def next_move(self, move_from):
        # TODO: update when main loop handling of crossing 360 is updated
        # this is all janky as fuck but doesn't require more previous steps and is continuous
        if move_from.r == 0 and move_from.t % (self.width_step*2) == 0:
            r_next = R_MAX 
            t_next = move_from.t
        if move_from.r != 0 and move_from.t % (self.width_step*2) == 0:
            r_next = R_MAX 
            t_next = move_from.t + self.width_step
        if move_from.r != 0 and move_from.t % (self.width_step*2) != 0:
            r_next = 0
            t_next = move_from.t
        if move_from.r == 0 and move_from.t % (self.width_step*2) != 0:
            r_next = 0
            t_next = move_from.t + self.width_step
        
        return Move(r=r_next, t=t_next, s=4000)

@dataclass
class SVGMode(Mode):
    polar_pts: List[PolarPt] = field(default_factory=list)
    mode_name: str = "svg"
    svg_file_path: str = "youre_hot.svg"
    pt_index: int = 0

    def startup(self):
        svg_parser = SVGParser()
        pts = svg_parser.get_pts_from_file(self.svg_file_path)
        pts = svg_parser.scale_and_center(pts)
        self.polar_pts = svg_parser.convert_to_table_axes(pts)
        # create_polar_plot(self.polar_pts)
        self.pt_index = 0
        
        # if we are already on the outside, go to the correct theta before starting the svg to avoid spiralling
        if state.grbl.mpos_r > R_MAX-2:
            first_t = self.polar_pts[0].t
            self.polar_pts.insert(0, PolarPt(r=state.grbl.mpos_r, t=first_t))
    
    def next_move(self, move_from):
        next_pt = self.polar_pts[self.pt_index]
        self.pt_index += 1
        if self.pt_index >= len(self.polar_pts):
            self.done = True
            return Move()
        
        return Move(r=next_pt.r, t=next_pt.t, s=2000)


def normalize_vector(xy):
    length = math.sqrt(xy[0]**2 + xy[1]**2)
    return [xy[0]/length, xy[1]/length]

def get_direction(curr_xy, next_xy):
    dx = next_xy[0] - curr_xy[0]
    dy = next_xy[1] - curr_xy[1]
    return normalize_vector([dx, dy])

def cartesian_to_polar_non_object(x, y):
    r = math.sqrt(x**2 + y**2)
    t = math.atan2(y, x)*180/math.pi
    return float(r), float(t)

def polar_to_cartesian_non_object(r, t):
    print(f">>>>> r: {r}")
    print(f">>>>> t: {t}")
    t = t*math.pi/180
    x = r * math.cos(t)
    y = r * math.sin(t)
    return float(x), float(y)

def cartesian_to_polar(pt: CartesianPt) -> PolarPt:
    r = math.sqrt(pt.x**2 + pt.y**2)
    t = math.atan2(pt.y, pt.x)*180/math.pi
    return PolarPt(float(r), float(t))


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
    return f"G1 X{move.r:.2f} Z{move.t_grbl:.2f} F{move.s:.2f}\n"

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
        elif move.r > R_MAX or move.r < R_MIN:
            print(f"Requested move {move} is out of bounds.")
            return False
    print(f"Checks passed.")
    return True


def next_move_to_msg():
    """Check move validity based on limits and grbl status, then make next msg next_move."""
    set_t_grbl()
    # If r limit has been hit, next move needs to be in oppsite direction
    if state.next_move != None and not state.next_move.is_empty():
        if check_move(state.next_move):
            state.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.MOVE, msg=format_move(state.next_move))
            print(f"Next msg: {state.next_grbl_msg}")
    else:
        state.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.EMPTY)

def homing_next_msg():
    if state.limits_hit.hard_r_min:
        state.next_move = Move(r=state.grbl.mpos_r+30, t=state.grbl.mpos_t, s=3000)
        set_t_grbl()
    elif state.limits_hit.hard_r_max:
        state.next_move = Move(r=state.grbl.mpos_r-30, t=state.grbl.mpos_t, s=3000)
        set_t_grbl()
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
    dw.show("state", state)
    dw.next()
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
            dw.show("state", state)
            dw.next()
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
    dw.show("state", state)
    dw.next()

def update_sensor_data(sensor_data_queue):
    if not sensor_data_queue.empty():
        # Get the most recent item
        raw = None
        while not sensor_data_queue.empty():
            raw = sensor_data_queue.get()
            
        if raw and len(raw) == EXPECTED_SENSOR_BYTESTRING_LENGTH:
            touch_sensors = [int(char) for char in raw[0:16]]
            state.touch_sensors = touch_sensors
            print(f">>>>>> state.touch_sensors {state.touch_sensors}")
            prox_status = int(raw[16])
            state.limits_hit.theta_zero = bool(prox_status)


def set_t_grbl():
    # standard case
    if (state.prev_move.t_grbl != None 
        and state.prev_move.t_grbl != None 
        and state.next_move.t != None):
        delta = state.next_move.t - state.prev_move.t
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        state.next_move.t_grbl = state.prev_move.t_grbl + delta
        return True
    elif state.next_move.t == None:
        return False
    # if  state.prev_move.t == None:
    #     if state.prev_move.t_grbl == None:
    #         state.prev_move.t_grbl = state.grbl.mpos_t
    #     state.prev_move.t = state.prev_move.t_grbl % 360
    # if  state.prev_move.t_grbl == None:
    #     if state.prev_move.t == None:
    #         state.prev_move.t_grbl = 0
    #     else:
    #         state.prev_move.t = state.prev_move.t_grbl

def send_grbl_settings(grbl_settings):
    for key, value in grbl_settings.items():
        state.next_grbl_msg = GrblSendMsg(msg_type=GrblSendMsgType.SETTING, msg=f"${key}={value}\n")
        

# === MAIN CONTROL SCRIPT ======================================================

# --- ARDUINO UNO MOTOR CONTROLLER CONFIGURATION ---
# UNO_SERIAL_PORT_NAME = 'COM5' # Emma
UNO_SERIAL_PORT_NAME = 'COM8' # Jules
# UNO_SERIAL_PORT_NAME = "/dev/ttyACM1" #pi
UNO_BAUD_RATE = 115200

# --- ARDUINO NANO I/O CONTROLLER CONFIGURATION ---
NANO_SERIAL_PORT_NAME = 'COM6'
NANO_BAUD_RATE = 9600

# --- SAND TABLE INFORMATION ---
TICKS_PER_MM_R = 40 # not used here, set in GRBL $100=40
TICKS_PER_DEG_THETA = 22.222 # not used here, set in GRBL $102=22.222
DISH_RADIUS_MM = 280
MARBLE_DIAMETER_MM = 14
MARBLE_WAKE_PITCH_MM = 14
R_MIN = 0
R_MAX = DISH_RADIUS_MM - MARBLE_DIAMETER_MM/2

# --- GRBL SETTINGS ---
grbl_settings = {
    0: 10,  # Step pulse, microseconds
    1: 25,  # Step idle delay, milliseconds
    2: 0,  # Step port invert, XYZmask*
    3: 4,  # Direction port invert, XYZmask*
    4: 0,  # Step enable invert, (0=Disable, 1=Invert)
    5: 0,  # Limit pins invert, (0=N-Open. 1=N-Close)
    6: 0,  # Probe pin invert, (0=N-Open. 1=N-Close)
    10: 255,  # Status report, '?' status contents
    11: 0.010,  # Junction deviation, mm
    12: 0.002,  # Arc tolerance, mm
    13: 0,  # Report in inches, (0=mm. 1=Inches)**
    20: 0,  # Soft limits, (0=Disable. 1=Enable, Homing must be enabled)
    21: 1,  # Hard limits, (0=Disable. 1=Enable)
    22: 1,  # Homing cycle, (0=Disable. 1=Enable)
    23: 3,  # Homing direction invert, XYZmask* Sets which corner it homes to.
    24: 100.000,  # Homing feed, mm/min
    25: 3000.000,  # Homing seek, mm/min
    26: 250,  # Homing debounce, milliseconds
    27: 8.000,  # Homing pull-off, mm
    30: 1000,  # Max spindle speed, RPM
    31: 0,  # Min spindle speed, RPM
    32: 0,  # Laser mode, (0=Off, 1=On)
    100: 40.000,  # Number of X steps to move 1mm
    101: 40.000,  # Number of Y steps to move 1mm
    102: 22.222,  # Number of Z steps to move 1mm (degree)
    110: 10000.000,  # X Max rate, mm/min
    111: 10000.000,  # Y Max rate, mm/min
    112: 10000.000,  # Z Max rate, mm/min
    120: 50.000,  # X Acceleration, mm/sec^2
    121: 50.000,  # Y Acceleration, mm/sec^2
    122: 10.000,  # Z Acceleration, mm/sec^2
    130: 550.000,  # X Max travel, mm Only for Homing and Soft Limits.
    131: 345.000,  # Y Max travel, mm Only for Homing and Soft Limits.
    132: 200.000  # Z Max travel, mm Only for Homing and Soft Limits.
}


state = State()
mode = Mode()

dw = debugger_window.DebuggerWindow()

def main():
    
    loop_count = 0
    
    # Start debug window
    dw.startup()
 
    # Set up debug window order, state last
    dw.show("msg", "")
    dw.show("loop_count",loop_count)
    dw.show("mode", "")
    dw.show("state","")
    
    
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
    state.flags_to_setup() # allow all types of actions
    state.flags.run_control_loop = True
    state.prev_limits_hit.soft_r_min = False
    state.prev_move = Move(r=0,t=0,t_grbl=0)
    state.next_move = Move(r=0,t=0,t_grbl=0)
    
    # modes = [SpiralMode(mode_name="spiral out"), SpiralMode(mode_name="spiral in", r_dir=-1)]
    # modes = [SVGMode(svg_file_path="youre_hot.svg")]
    modes = [SpiralMode(mode_name="spiral out"), 
             SVGMode(svg_file_path="hex_gosper_d3.svg")]
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
        
        if state.flags.send_grbl_settings:
            print("Sending GRBL settings...")
            send_grbl_settings(grbl_settings)

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
    
        dw.show("loop_count",loop_count)
        dw.show("mode", mode)
        dw.show("state", state)
        dw.next()
    
# --- MAIN CONTROL LOOP --------------------------------------------------------
    while state.flags.run_control_loop:
        loop_count += 1
        state.iterate()
        
        print(f"Loop Start --------------- moves sent: {state.moves_sent} | loop_count: {loop_count}")
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
            
            print(f">>>> mode.need_touch_sensors: {mode.need_touch_sensors}")
            if mode.need_touch_sensors:
                print("Checking touch sensors...")
                update_sensor_data(sensor_data_queue)
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
        
        # If the mode is done, cyle to next modeC
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
            state.prev_move = Move(r=state.grbl.mpos_r, t_grbl=state.grbl.mpos_t, s=0, t=state.grbl.mpos_t % 360)
            state.next_move = mode.next_move(state.prev_move)
            set_t_grbl()
            mode.set_next_speed()
            print(f"Next move: {state.next_move}")
        
        # If input has not changed and buffer is low, 
        # calc next move from last sent move. Otherwise, do nothing.
        elif state.flags.buffer_space:
            state.flags.need_send_next_move = True
            print("Calculating next move from last sent move...")
            state.next_move = mode.next_move(state.prev_move)
            set_t_grbl()
            mode.set_next_speed()
            print(f"Next move: {state.next_move}")
            
        mode.update()

# --- ACT ----------------------------------------------------------------------
        print("Acting...")
        state.flags_to_act_phase()
        if state.flags.run_control_loop and state.flags.connect_to_uno:
            if state.flags.need_send_next_move and state.flags.log_commands:
                state.grbl_command_log.append([time.time(), state.next_move.r, state.next_move.t, state.next_move.s])
            run_grbl_communicator()
        else:
            print("Not performing grbl tasks because run_control_loop is set to False.")
        
        dw.show("loop_count",loop_count)
        dw.show("mode", mode)
        dw.show("state", state)

        time.sleep(0.5)
        dw.next()
    # --- END OF MAIN CONTROL LOOP -------------------------------------------------

    print("Control loop exited. Performing safe stop...")
    state.flags_to_sense_phase()
    state.flags.need_reset = True
    state.flags.need_status = True
    run_grbl_communicator()

    if state.flags.log_path or state.flags.log_commands:
        print("Saving data...")
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"../logs/table_log_{timestamp}.json"

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