import serial
import time
import math
import threading
import queue
import json
from datetime import datetime

from parse_grbl_status import *
from main import read_from_port, send_with_check


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
GRBL_UNLOCK = bytes("$X\n",  'utf-8')
GRBL_HOME = bytes("$H\n",  'utf-8')

program_start_time = time.time()

print("Establishing serial connection to Arduino Uno...")
uno_serial_port = serial.Serial(UNO_SERIAL_PORT_NAME, UNO_BAUD_RATE, timeout=1)
time.sleep(2) # Wait for connection to establish!

data_queue = queue.Queue()
stop_event = threading.Event()
reader_thread = threading.Thread(target=read_from_port, args=(uno_serial_port, stop_event, data_queue))
reader_thread.daemon = True
reader_thread.start()
time.sleep(2)
send_with_check(uno_serial_port, GRBL_STATUS, data_queue)
time.sleep(2)

send_with_check(uno_serial_port, GRBL_SOFT_RESET, data_queue)
time.sleep(.5)
send_with_check(uno_serial_port, GRBL_STATUS, data_queue)
time.sleep(.5)

send_with_check(uno_serial_port, GRBL_UNLOCK, data_queue)
time.sleep(.5)
send_with_check(uno_serial_port, GRBL_STATUS, data_queue)
time.sleep(.5)

send_with_check(uno_serial_port, "G01 X300 F3000\n", data_queue)
time.sleep(10)

send_with_check(uno_serial_port, GRBL_SOFT_RESET, data_queue)
time.sleep(.5)

send_with_check(uno_serial_port, GRBL_UNLOCK, data_queue)
time.sleep(.5)

send_with_check(uno_serial_port, GRBL_HOME, data_queue, q_timeout=100)
time.sleep(1)

send_with_check(uno_serial_port, GRBL_STATUS, data_queue)

send_with_check(uno_serial_port, GRBL_SOFT_RESET, data_queue)
time.sleep(.5)
send_with_check(uno_serial_port, GRBL_STATUS, data_queue)
time.sleep(.5)

send_with_check(uno_serial_port, GRBL_UNLOCK, data_queue)
time.sleep(.5)
send_with_check(uno_serial_port, GRBL_STATUS, data_queue)
time.sleep(.5)

send_with_check(uno_serial_port, "G01 X260 F10000\n", data_queue)
time.sleep(1)

send_with_check(uno_serial_port, GRBL_STATUS, data_queue)
send_with_check(uno_serial_port, GRBL_HOLD, data_queue)
time.sleep(1)
send_with_check(uno_serial_port, GRBL_STATUS, data_queue)

send_with_check(uno_serial_port, GRBL_SOFT_RESET, data_queue)
time.sleep(2)
send_with_check(uno_serial_port, GRBL_STATUS, data_queue)
time.sleep(2)

send_with_check(uno_serial_port, GRBL_UNLOCK, data_queue)
time.sleep(2)
send_with_check(uno_serial_port, GRBL_STATUS, data_queue)
time.sleep(2)

send_with_check(uno_serial_port, "G01 X1 F3000\n", data_queue)
send_with_check(uno_serial_port, "G01 X10 F3000\n", data_queue)
send_with_check(uno_serial_port, "G01 X1 F3000\n", data_queue)
time.sleep(1)
send_with_check(uno_serial_port, GRBL_STATUS, data_queue)


send_with_check(uno_serial_port, GRBL_SOFT_RESET, data_queue)
time.sleep(2)
send_with_check(uno_serial_port, GRBL_STATUS, data_queue)
time.sleep(2)

send_with_check(uno_serial_port, GRBL_UNLOCK, data_queue)
time.sleep(2)
send_with_check(uno_serial_port, GRBL_STATUS, data_queue)
time.sleep(2)

send_with_check(uno_serial_port, "G01 X260 F10000\n", data_queue)
time.sleep(1)
send_with_check(uno_serial_port, GRBL_STATUS, data_queue)

