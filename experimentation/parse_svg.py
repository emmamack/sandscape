import xml.etree.ElementTree as et
from dataclasses import dataclass
from typing import List
import matplotlib.pyplot as plt
import re
import bezier
import numpy as np
import math

SEG_LENGTH = 5

@dataclass
class Curve:
    marker: str
    body: List[int]

    def __post_init__(self):
        expected_length = curve_types_to_expected_length[self.marker][0]
        multiples_expected = curve_types_to_expected_length[self.marker][1]

        if multiples_expected:
            if not (len(self.body)%expected_length) == 0:
                raise ValueError(f"Unexpected curve length for type {self.marker}: {len(self.body)}!")
        else: 
            if not len(self.body) == expected_length:
                raise ValueError(f"Unexpected curve length for type {self.marker}: {len(self.body)}!")

@dataclass
class Point:
    x: int
    y: int
    
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def to_tuple(self):
        return (self.x, self.y)

#TODO: maybe encoded curve types and metadata in a better object than this - include description of what the curve actually is/does
# tuple: (expected_length, multiples_expected)
curve_types_to_expected_length = {
    'm': (2, False),
    'M': (2, False),
    'l': (2, True),
    'L': (2, False),
    'h': (1, False),
    'v': (1, False),
    'c': (6, True)
}

def split_raw_curves(raw):
    split_at_letter = re.findall(r'[a-zA-Z][^a-zA-Z]*', raw)
    curves = []

    for curve_block in split_at_letter:
        marker = curve_block[0]

        if marker == 'z': #discard end markers
            continue

        body = [float(x) for x in re.findall(r'-?\d+\.?\d*', curve_block[1:])]
        curve_length = curve_types_to_expected_length[marker][0]
        single_curves = [body[i:i + curve_length] for i in range(0, len(body), curve_length)]
        for single_curve in single_curves:
            curves.append(Curve(
                marker=marker,
                body=single_curve
            ))
    return curves

def discretize_bezier(node_set, prev_pt):
    nodes = np.asfortranarray([
        [prev_pt.x, prev_pt.x+node_set[0], prev_pt.x+node_set[2], prev_pt.x+node_set[4]],
        [prev_pt.y, prev_pt.y+node_set[1], prev_pt.y+node_set[3], prev_pt.y+node_set[5]],
    ])
    # print(nodes)
    bezier_curve_obj = bezier.Curve(nodes, degree=3)

    num_pts_in_curve = bezier_curve_obj.length / SEG_LENGTH
    s_vals = np.linspace(0.0, 1.0, math.ceil(num_pts_in_curve))
    evaluator_output = bezier_curve_obj.evaluate_multi(s_vals)
    
    pts = [Point(x=float(x), y=float(y)) for x, y in zip(evaluator_output[0], evaluator_output[1])]
    
    return pts



def parse_multiple_curves(curves_raw):
    curves = split_raw_curves(curves_raw)
    
    pts = [Point(x=0, y=0)]
    for curve in curves:
        prev_pt = pts[-1]
        if curve.marker=='M' or curve.marker=="L": # moveto, lineto (absolute)
            # svg defines M and L as different, but since the ball can't float, to us they are the same
            pts.append(Point(x=curve.body[0], y=curve.body[1]))
        elif curve.marker=='m' or curve.marker=="l": # moveto, lineto (relative)
            pts.append(Point(x=prev_pt.x+curve.body[0], y=prev_pt.y+curve.body[1]))
        elif curve.marker=='h': # horizontal line
            pts.append(Point(x=prev_pt.x+curve.body[0], y=prev_pt.y))
        elif curve.marker=='v': # vertical line
            pts.append(Point(x=prev_pt.x, y=prev_pt.y+curve.body[0]))
        elif curve.marker=='c': # bezier curve
            pts.extend(discretize_bezier(curve.body, prev_pt))
        elif curve.marker=='z': # end of curve
            pass
        else:
            print(f"Encountered unexpected curve marker: {curve.marker}")
    
    return pts[1:] # remove artificial (0,0) point
        

def get_layer_above_meat(layer):

    print(f"current layer.attrib: {layer.attrib}")
    if layer.attrib == {}:
        return None

    for child in layer:
        print(f"evaluating child: {child.attrib}")
        if child.attrib.get('d', None):
            print("it's this one!")
            return layer

    for child in layer:
        layer_below_evaluation = get_layer_above_meat(child)
        if layer_below_evaluation:
            return layer_below_evaluation
    
    raise RuntimeError("oops, shouldn't have gotten here")

def get_pts_from_file(file):
    tree = et.parse(file)
    root = tree.getroot()
    print(f"root: {root.attrib}")
    if root.attrib['version'] != '1.1':
        print(f"Warning! Unsupported svg version: {root.attrib['version']}")

    layer_above_meat = get_layer_above_meat(root)
    print(layer_above_meat.attrib)

    all_pts = []
    for child in layer_above_meat:
        curves_raw = child.attrib['d']
        all_pts.extend(parse_multiple_curves(curves_raw))
    
    return all_pts

def plot_pts(pts): 
    pts_decoded = [pt.to_tuple() for pt in pts]
    print(pts_decoded)
    x_coords = [pt[0] for pt in pts_decoded]
    y_coords = [pt[1] for pt in pts_decoded]
    plt.figure()
    plt.plot(x_coords, y_coords, 'k-')  # 'b-' means blue line
    # plt.scatter(x_coords, y_coords, c='red', s=5)  # Add points as red dots
    # plt.plot(x_coords, y_coords, ".-", c='red', markersize=10)
    plt.scatter(x_coords, y_coords, c=np.linspace(0,1,len(x_coords)), s=20)
    plt.set_cmap("gist_rainbow") 
    plt.gca().yaxis.set_inverted(True)
    plt.grid(True)
    plt.axis('equal')
    plt.title('SVG Path Visualization')
    plt.show()

if __name__ == "__main__":
    svg_file = "..\svg_examples\cabin.svg"
    # svg_file = "..\svg_examples\Archimedean_spiral.svg"
    # svg_file = "..\svg_examples\spiral.svg"
    # svg_file = "..\svg_examples\eye-drops-svgrepo-com.svg"
    # svg_file = "..\svg_examples\chef-man-cap-svgrepo-com.svg"
    
    pts = get_pts_from_file(svg_file)
    plot_pts(pts)