import xml.etree.ElementTree as et
from dataclasses import dataclass
from typing import List
import matplotlib.pyplot as plt
import re

@dataclass
class Curve:
    marker: str
    body: List[int]

    def __post_init__(self):
        if not len(self.body) == curve_types_to_expected_length[self.marker]:
            raise ValueError(f"Unexpected curve length for type {self.marker}: {len(self.body)}!")

@dataclass
class Point:
    x: int
    y: int

    def to_tuple(self):
        return (self.x, self.y)

#TODO: maybe encoded curve types and metadata in a better object than this - include description of what the curve actually is/does
curve_types_to_expected_length = {
    'm':2,
    'h':1,
    'v':1,
    'c':6
}

def split_raw_curves(raw):
    split_at_letter = re.findall(r'[a-zA-Z][^a-zA-Z]*', raw)
    curves = []
    for curve_block in split_at_letter:
        marker = curve_block[0]
        body = [float(x) for x in re.findall(r'-?\d+\.?\d*', curve_block[1:])]
        curves.append(Curve(
            marker=marker,
            body=body
        ))
    return curves


def parse_multiple_curves(curves_raw):
    curves = split_raw_curves(curves_raw)
    
    pts = [Point(x=0, y=0)]
    for curve in curves:
        prev_pt = pts[-1]
        # if curve.marker=='M':
        #   capital letter means absolutely positioned
        if curve.marker=='m':
            pts.append(Point(x=prev_pt.x+curve.body[0], y=prev_pt.y+curve.body[1]))
        if curve.marker=='h':
            pts.append(Point(x=prev_pt.x+curve.body[0], y=prev_pt.y))
        if curve.marker=='v':
            pts.append(Point(x=prev_pt.x, y=prev_pt.y+curve.body[0]))
        # if curve.marker=='c':
        #     TODO
    
    return pts
        

tree = et.parse("..\svg_examples\Archimedean_spiral.svg")
root = tree.getroot()

if root.attrib['version'] != '1.1':
    print(f"Warning! Unsupported svg version: {root['version']}")

#TODO: handle variable numbers of layers before meat appears

for child in [root[0]]:
    curves_raw = child.attrib['d']
    pts = parse_multiple_curves(curves_raw)
    
    pts_decoded = [pt.to_tuple() for pt in pts]
    print(pts_decoded)
    x_coords = [pt[0] for pt in pts_decoded]
    y_coords = [pt[1] for pt in pts_decoded]
    plt.figure()
    plt.plot(x_coords, y_coords, 'b-')  # 'b-' means blue line
    plt.scatter(x_coords, y_coords, c='red', s=50)  # Add points as red dots
    plt.grid(True)
    plt.axis('equal')  # Make the scale equal on both axes
    plt.title('SVG Path Visualization')
    plt.show()