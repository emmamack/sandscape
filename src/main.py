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
from parse_svg import SVGParser, create_polar_plot, create_cartesian_plot

from local.local_constants import UNO_SERIAL_PORT_NAME, NANO_SERIAL_PORT_NAME, LOG_COMMANDS, LOG_PATH, GRBL_HOMING_ON, CONNECT_TO_UNO, CONNECT_TO_NANO, SYNC_GRBL_SETTINGS
from utils import *
from state import *
from grbl import *
from nano_comm import *
from mode import *

# --- PROGRAM OPTIONS --- # SET IN local_constants
# UNO_SERIAL_PORT_NAME = "/dev/ttyACM0" #pi
# NANO_SERIAL_PORT_NAME = 'COM6'
# LOG_COMMANDS = False
# LOG_PATH = False
# GRBL_HOMING_ON = True
# CONNECT_TO_UNO = True
# CONNECT_TO_NANO = False
# SYNC_GRBL_SETTINGS = True

@dataclass
class SandscapeController:
    state: State
    mode: Mode
    grbl_comm: GrblCommunicator
    nano_comm: NanoCommunicator



def main():
    global uno_serial_port
    # global grbl_data_queue
    signal.signal(signal.SIGINT, sig_handler)
    
    # --- SETUP ---
    state = State()
    mode = Mode(state=state)


    loop_count = 0
    state.phase = Phase.SETUP
    state.flags.run_control_loop = True
    state.prev_limits_hit.soft_r_min = False
    state.prev_move = Move(r=0,t=0,t_grbl=0)
    state.next_move = Move(r=0,t=0,t_grbl=0)
    
    modes = [
        SpiralMode(state, mode_name="spiral out"), 
        SVGMode(state, svg_file_name="pentagon_fractal"),
        SpiralMode(state, mode_name="spiral in", r_dir=-1),
        SpiralMode(state, mode_name="spiral out"), 
        SVGMode(state, svg_file_name="hex_gosper_d4"),
        SpiralMode(state, mode_name="spiral in", r_dir=-1),
        SVGMode(state, svg_file_name="dither_wormhole"),
        SpiralMode(state, mode_name="spiral in", r_dir=-1),
        SpiralMode(state, mode_name="spiral out"), 
        SVGMode(state, svg_file_name="hilbert_d5"),
        SpiralMode(state, mode_name="spiral in", r_dir=-1),
    ]
    mode_index = 0
    mode = modes[mode_index]

    stop_event = threading.Event()

    if CONNECT_TO_NANO:
        print("Establishing serial connection to Arduino Nano...")
        nano_serial_port = serial.Serial(NANO_SERIAL_PORT_NAME, NANO_BAUD_RATE, timeout=1)
        time.sleep(2) # Wait for connection to establish!
        
        # Set up sensor monitoring thread
        sensor_data_queue = queue.Queue()
        sensor_reader_thread = threading.Thread(target=read_from_port, args=(nano_serial_port, stop_event, sensor_data_queue, True, "NANO"))
        sensor_reader_thread.daemon = True
        sensor_reader_thread.start()
        time.sleep(2)
        nano_comm = NanoCommunicator(state=state, nano_serial_port=nano_serial_port, nano_data_queue=sensor_data_queue)
    else:
        print("Not connecting to Arduino Nano.")

    if CONNECT_TO_UNO:
        print("Establishing serial connection to Arduino Uno...")
        uno_serial_port = serial.Serial(UNO_SERIAL_PORT_NAME, UNO_BAUD_RATE, timeout=1)
        time.sleep(2) # Wait for connection to establish!
        
        # Set up grbl monitoring thread
        grbl_data_queue = queue.Queue()
        grbl_reader_thread = threading.Thread(target=read_from_port, args=(uno_serial_port, stop_event, grbl_data_queue, True, "UNO"))
        grbl_reader_thread.daemon = True
        grbl_reader_thread.start()
        time.sleep(2)
        grbl_comm = GrblCommunicator(state=state, uno_serial_port=uno_serial_port, grbl_data_queue=grbl_data_queue)
        
        print("Resetting and unlocking GRBL...")
        grbl_comm.reset()
        grbl_comm.unlock()

        if SYNC_GRBL_SETTINGS:
            print("Sending GRBL settings...")
            success = grbl_comm.sync_settings()
            if not success:
                print("ERROR: GRBL settings sync failed. Ending program.")
                return

        print("Checking GRBL status...")
        grbl_comm.status()

        if GRBL_HOMING_ON:
            print("Starting homing...")
            success = grbl_comm.home()
            if not success:
                print("ERROR: Homing sequence failed. Stopping GRBL and ending program.")
                uno_serial_port.write(bytes([0x18]))
                return

            print("Checking GRBL status...")
            grbl_comm.status()

    else:
        print("Not connecting to Arduino Uno.")
    
# --- MAIN CONTROL LOOP --------------------------------------------------------
    while state.flags.run_control_loop:
        loop_count += 1
        state.iterate()
        
        print(f"Loop Start --------------- moves sent: {state.moves_sent} | loop_count: {loop_count}")
        print(f"Current mode: {mode.mode_name}")
        print(mode)

# --- SENSE --------------------------------------------------------------------
        print("Sensing...")
        state.phase = Phase.SENSE
        if CONNECT_TO_NANO:
            if mode.need_control_panel:
                print("Checking control panel...")
            else:
                print("Ignoring control panel.")
            
            print(f">>>> mode.need_touch_sensors: {mode.need_touch_sensors}")
            if mode.need_touch_sensors:
                print("Checking touch sensors...")
                nano_comm.update_state()
            else:
                print("Ignoring touch sensors.")
        else:
            print("Not connected to Arduino Nano - ignoring control panel and touch sensors.")
            if mode.need_control_panel or mode.need_touch_sensors:
                print(f"Skipping mode: {mode.mode_name}")
                mode = modes[(mode_index + 1) % len(modes)]
        
        if CONNECT_TO_UNO:
            if mode.need_grbl:
                print("Checking GRBL status...")
                grbl_comm.status()
                
                if LOG_PATH:
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
        state.phase = Phase.THINK

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
            # print(mode)
            mode_index = (mode_index + 1) % len(modes)
            mode = modes[mode_index]
            mode.done = False
            mode.startup()
            state.flags.input_change = True
            print(f"Next mode: {mode.mode_name}")
            # print(mode)
        
        # If an input has changed, plan to send reset 
        # and calc next move from current position
        if state.flags.input_change:
            grbl_comm.need_reset = True
            grbl_comm.need_send_next_move = True
            print("Calculating next move from current position...")
            state.prev_move = Move(r=state.grbl.mpos_r, t_grbl=state.grbl.mpos_t, s=0, t=state.grbl.mpos_t % 360)
            state.next_move = mode.next_move(state.prev_move)
            # grbl_comm.set_t_grbl()
            mode.set_next_speed()
            print(f"Next move: {state.next_move}")
        
        # If input has not changed and buffer is low, 
        # calc next move from last sent move. Otherwise, do nothing.
        elif state.flags.buffer_space:
            grbl_comm.need_send_next_move = True
            print("Calculating next move from last sent move...")
            state.next_move = mode.next_move(state.prev_move)
            # grbl_comm.set_t_grbl()
            mode.set_next_speed()
            print(f"Next move: {state.next_move}")
            
        mode.update() # Allow mode to stay up to date even if next move is not needed this loop

# --- ACT ----------------------------------------------------------------------
        print("Acting...")
        state.phase = Phase.ACT
        if state.flags.run_control_loop and CONNECT_TO_UNO:
            if grbl_comm.need_send_next_move and LOG_COMMANDS:
                state.grbl_command_log.append([time.time(), state.next_move.r, state.next_move.t, state.next_move.s])
            grbl_comm.run()
        else:
            print("Not performing grbl tasks because run_control_loop is set to False.")
        
        time.sleep(0.1)
    
    # --- END OF MAIN CONTROL LOOP -------------------------------------------------

    print("Control loop exited. Performing safe stop...")
    state.phase = Phase.SETUP
    grbl_comm.need_reset = True
    grbl_comm.need_status= True
    grbl_comm.run()

    if LOG_PATH or LOG_COMMANDS:
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
    uno_serial_port.write(bytes([0x18]))
    print("Done.")
    exit(0)

if __name__ == "__main__":
    main()