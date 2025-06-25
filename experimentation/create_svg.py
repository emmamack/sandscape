import xml.etree.ElementTree as et
from dataclasses import dataclass
from typing import List
import matplotlib.pyplot as plt
import re
import bezier
import numpy as np
import math

from parse_svg import Curve, CartesianPt

def tangent_offset_curve(curve, offset_dist):
    """_summary_

    Args:
        curve (_type_): _description_
        offset_dist (_type_): _description_
    """

