import xml.etree.ElementTree as et
from dataclasses import dataclass
from typing import List
import matplotlib.pyplot as plt
import re
import bezier
import numpy as np

@dataclass
class Curve:
    marker: str
    body: List[int]

    # def __post_init__(self):
    #     if not len(self.body) == curve_types_to_expected_length[self.marker]:
    #         raise ValueError(f"Unexpected curve length for type {self.marker}: {len(self.body)}!")

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

def discretize_bezier(raw, prev_pt):
    node_sets = [raw[i:i + 6] for i in range(0, len(raw), 6)]

    pts = []

    for node_set in node_sets:
        nodes = np.asfortranarray([
            [prev_pt.x, prev_pt.x+node_set[0], prev_pt.x+node_set[2], prev_pt.x+node_set[4]],
            [prev_pt.y, prev_pt.y+node_set[1], prev_pt.y+node_set[3], prev_pt.y+node_set[5]],
        ])
        print(nodes)
        bezier_curve_obj = bezier.Curve(nodes, degree=3)

        s_vals = np.linspace(0.0, 1.0, 10)
        evaluator_output = bezier_curve_obj.evaluate_multi(s_vals)
        for x, y in zip(evaluator_output[0], evaluator_output[1]):
            pts.append(Point(x=float(x), y=float(y)))
        
        prev_pt = Point(x=prev_pt.x+node_set[4], y=prev_pt.y+node_set[5])
    
    return pts



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
        if curve.marker=='c':
            pts.extend(discretize_bezier(curve.body, prev_pt))
        else:
            print(f"Encountered unexpected curve marker: {curve.marker}")
    
    return pts
        

tree = et.parse("..\svg_examples\Archimedean_spiral.svg")
root = tree.getroot()

if root.attrib['version'] != '1.1':
    print(f"Warning! Unsupported svg version: {root['version']}")

#TODO: handle variable numbers of layers before meat appears

all_pts = []
for child in root:
    curves_raw = child.attrib['d']
    all_pts.extend(parse_multiple_curves(curves_raw))
    
pts_decoded = [pt.to_tuple() for pt in all_pts]
x_coords = [pt[0] for pt in pts_decoded]
y_coords = [pt[1] for pt in pts_decoded]
plt.figure()
plt.plot(x_coords, y_coords, 'b-')  # 'b-' means blue line
plt.scatter(x_coords, y_coords, c='red', s=50)  # Add points as red dots
plt.grid(True)
plt.axis('equal')  # Make the scale equal on both axes
plt.title('SVG Path Visualization')
plt.show()