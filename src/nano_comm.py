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
from state import *

@dataclass
class NanoCommunicator:
    state: State
    nano_serial_port: serial.Serial
    nano_data_queue: queue.Queue

    def update_state(self):
        if not self.nano_data_queue.empty():
            # Get the most recent item
            raw = None
            while not self.nano_data_queue.empty():
                raw = self.nano_data_queue.get()
                
            if raw and len(raw) == EXPECTED_SENSOR_BYTESTRING_LENGTH:
                touch_sensors = [int(char) for char in raw[0:16]]
                self.state.touch_sensors = touch_sensors
                print(f">>>>>> state.touch_sensors {self.state.touch_sensors}")
                prox_status = int(raw[16])
                self.state.limits_hit.theta_zero = bool(prox_status)