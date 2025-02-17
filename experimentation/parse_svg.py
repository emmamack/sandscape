import xml.etree.ElementTree as et
from dataclasses import dataclass
from typing import List

@dataclass
class Curve:
    marker: str
    body: List[int]

@dataclass
class Point:
    x: int
    y: int


def parse_multiple_curves(curves_raw):
    # subcommands = split_command(command)
    curves = [Curve(marker='m', body=[19, 275]),
              Curve(marker='h', body=[612])]
    
    pts = [Point(x=0, y=0)]
    for curve in curves:
        prev_pt = pts[-1]
        if curve.marker=='m':
            pts.append(Point(x=curve.body[0], y=curve.body[1]))
        if curve.marker=='h':
            pts.append(Point(x=prev_pt.x+curve.body[0], y=prev_pt.y))
    
    return pts
        

tree = et.parse("..\svg_examples\Archimedean_spiral.svg")
root = tree.getroot()

if root.attrib['version'] != '1.1':
    print(f"Warning! Unsupported svg version: {root['version']}")

#TODO: handle variable numbers of layers before meat appears

for child in root:
    curves_raw = child.attrib['d']
    pts = parse_multiple_curves(curves_raw)
    print(pts)