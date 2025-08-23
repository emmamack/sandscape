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

from local.local_constants import UNO_SERIAL_PORT_NAME, NANO_SERIAL_PORT_NAME
from utils import *
from state import *

@dataclass
class NanoCommunicator(SerialCommunicator):
    state: State = field(default_factory=lambda: State())
    port_name: str = NANO_SERIAL_PORT_NAME
    baud_rate: int = NANO_BAUD_RATE
    display_name: str = "Nano"

    def update_state(self):
        if not self.data_queue.empty():
            # Get the most recent item
            raw = None
            while not self.data_queue.empty():
                raw = self.data_queue.get()
                
            if raw and len(raw) == EXPECTED_SENSOR_BYTESTRING_LENGTH:
                touch_sensors = [int(char) for char in raw[0:16]]
                self.state.touch_sensors = touch_sensors
                print(f">>>>>> state.touch_sensors {self.state.touch_sensors}")
                prox_status = int(raw[16])
                self.state.limits_hit.theta_zero = bool(prox_status)