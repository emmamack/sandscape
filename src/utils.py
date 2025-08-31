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

UNO_BAUD_RATE = 115200
NANO_BAUD_RATE = 9600

EXPECTED_SENSOR_BYTESTRING_LENGTH = 17

# --- SAND TABLE INFORMATION ---
TICKS_PER_MM_R = 40 # not used here, set in GRBL $100=40
TICKS_PER_DEG_THETA = 22.222 # not used here, set in GRBL $102=22.222
DISH_RADIUS_MM = 280
MARBLE_DIAMETER_MM = 14
MARBLE_WAKE_PITCH_MM = 14
R_MIN = 0
R_MAX = DISH_RADIUS_MM - MARBLE_DIAMETER_MM/2

def red(text):
    return f"\033[91m{text}\033[0m"

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

@dataclass
class SerialCommunicator:
    port_name: str
    baud_rate: int
    serial_port: serial.Serial = field(default_factory=lambda: serial.Serial())
    data_queue: queue.Queue = field(default_factory=lambda: queue.Queue())
    stop_event: threading.Event = field(default_factory=lambda: threading.Event())
    reader_thread: threading.Thread = field(default_factory=lambda: threading.Thread())
    connected: bool = False
    display_name: str = "<untitled device>"
    print_header: bool = True

    def serial_connect(self, do_ping=True):
        print(f"Establishing serial connection to {self.display_name}...")
        try:
            self.serial_port = serial.Serial(self.port_name, self.baud_rate, timeout=1)
            time.sleep(2) # Wait for connection to establish!
            self.data_queue = queue.Queue()
            self.reader_thread = threading.Thread(target=self.read_from_port)
            self.reader_thread.daemon = True
            self.reader_thread.start()
            time.sleep(2)
            if do_ping:
                success = self.ping()
                if success == None:
                    print(f"Ping is not implemented for {self.display_name}. Connection assumed successful.")
                    self.connected = True
                elif success == True:
                    print(f"Ping successful for {self.display_name}.")
                    self.connected = True
                else:
                    print(f"{red('ERROR')}: Ping failed for {self.display_name}.")
                    self.connected = False
                    return False
                print(f"Serial connection to {self.display_name} established.")
                return True
            else:
                print(f"Skipping ping for {self.display_name}.")
        except Exception as e:
            print(f"{red('ERROR')}: Failed to establish serial connection to {self.display_name}. {e}")
            return False

    def serial_disconnect(self):
        print(f"Ending serial connection to {self.display_name}.")
        self.serial_port.close()
        self.stop_event.set()

    def ping(self) -> bool: # pyright: ignore[reportReturnType]
        pass

    def read_from_port(self):
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
        while not self.stop_event.is_set():
            try:
                # The readline() function will block until a newline character
                # is received, or until the timeout (set during port initialization)
                # is reached.
                line = self.serial_port.readline()

                # If a line was actually read (i.e., not a timeout)
                if line:
                    # Decode the bytes into a string, using UTF-8 encoding.
                    # 'errors='ignore'' will prevent crashes on decoding errors.
                    # .strip() removes leading/trailing whitespace, including the newline.
                    time_since_last_msg = time.time() - last_msg_timestamp
                    last_msg_timestamp = time.time()
                    decoded_line = line.decode('utf-8', errors='ignore').strip()
                    print_str = ""
                    if self.print_header:
                        print_str += f"{time.time():.5f} | {time_since_last_msg:.5f}s | Received"
                        if self.display_name != "":
                            print_str += f" from {self.display_name}: "
                        else:
                            print_str += ": "
                    print_str += decoded_line
                    print(print_str)
                    if len(decoded_line) > 0:
                        self.data_queue.put(decoded_line)

            except serial.SerialException as e:
                # Handle cases where the serial port is disconnected or an error occurs
                print(f"{red('ERROR')}: Serial port error: {e}. Stopping thread.")
                break
            except Exception as e:
                # Handle other potential exceptions
                print(f"{red('ERROR')}: An unexpected error occurred: {e}")
                if not self.stop_event.is_set():
                    # Avoid flooding the console with error messages
                    time.sleep(1)

        print("Reader thread finished.")