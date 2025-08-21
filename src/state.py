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

from utils import *

class Phase(Enum):
    SETUP = "setup"
    SENSE = "sense"
    THINK = "think"
    ACT = "act"

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
    # need_reset: bool = False
    # need_status: bool = False
    # need_unlock: bool = False
    # expecting_extra_msg = False
    need_homing: bool = False
    need_calc_next_move: bool = False
    # need_send_next_move: bool = False
    # need_send_setting: bool = False
    # need_get_settings: bool = False
    # grbl_com_done: bool = False
    
    # user set program settings flags
    # log_commands: bool = True
    # log_path: bool = True
    # grbl_homing_on: bool = True
    # connect_to_uno: bool = True
    # connect_to_nano: bool = False
    # send_grbl_settings: bool = True
    
    # status flags
    run_control_loop: bool = False

    # in_setup_phase: bool = False
    # in_sense_phase: bool = False
    # in_think_phase: bool = False
    # in_act_phase: bool = False

@dataclass
class State:
    phase: Phase = Phase.SETUP
    limits_hit: LimitsHit = field(default_factory=lambda: LimitsHit())
    control_panel: ControlPanel = field(default_factory=lambda: ControlPanel())
    touch_sensors: list = field(default_factory=list)
    grbl: Grbl = field(default_factory=lambda: Grbl())
    flags: Flags = field(default_factory=lambda: Flags())
    next_move: Move = field(default_factory=lambda: Move(r=0,t=0))

    prev_limits_hit: LimitsHit = field(default_factory=lambda: LimitsHit())
    prev_control_panel: ControlPanel = field(default_factory=lambda: ControlPanel())
    prev_touch_sensors: list = field(default_factory=list)
    prev_grbl: Grbl = field(default_factory=lambda: Grbl())
    prev_flags: Flags = field(default_factory=lambda: Flags())
    prev_move: Move = field(default_factory=lambda: Move(r=0,t=0))
    # prev_grbl_msg: GrblSendMsg = field(default_factory=lambda: GrblSendMsg())
    desired_linspeed: int = 3000 #mm/min
    moves_sent: int = 0

    # last_grbl_resp: GrblRespMsg = field(default_factory=lambda: GrblRespMsg())
    # next_grbl_msg: GrblSendMsg = field(default_factory=lambda: GrblSendMsg())

    theta_correction: float = 0 # TODO
    path_history: list = field(default_factory=list)
    grbl_command_log: list = field(default_factory=list)
    # curr_grbl_settings: dict = field(default_factory=dict)

        
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
            # f"  last_grbl_resp={self.last_grbl_resp},\n"
            # f"  next_grbl_msg={self.next_grbl_msg}\n"
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
        # self.flags.need_reset = False
        # self.flags.need_send_next_move = False

    def check_move(self, move):
        """Check move validity based on limits and grbl status."""
        # If r limit has been hit, next move needs to be in oppsite direction
        print(f"Checking move {move}...")
        if self.limits_hit.hard_r_min and move.r < self.grbl.mpos_r:
            print(f"Requested move {move} is into the hard_r_min limit switch.")
            return False
        if self.limits_hit.hard_r_max and move.r > self.grbl.mpos_r:
            print(f"Requested move {move} is into the hard_r_max limit switch.")
            return False

        if not self.flags.need_homing: # Assuming switches have not been hit and position can be trusted
            if self.limits_hit.soft_r_min and move.r < self.grbl.mpos_r:
                print(f"Requested move {move} is into the soft_r_min limit switch.")
                return False
            elif self.limits_hit.soft_r_max and move.r > self.grbl.mpos_r:
                print(f"Requested move {move} is into the soft_r_max limit switch.")
                return False
            elif move.r > R_MAX or move.r < R_MIN:
                print(f"Requested move {move} is out of bounds.")
                return False
        print(f"Checks passed.")
        return True