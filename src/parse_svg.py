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
class CartesianPt:
    x: int
    y: int

    def to_tuple(self):
        return (self.x, self.y)

@dataclass
class PolarPt:
    r: float
    t: float

class SVGParser:
    # Class attribute for curve types configuration
    curve_types_to_expected_length = {
        'm': (2, False),
        'M': (2, False),
        'l': (2, True),
        'L': (2, False),
        'h': (1, False),
        'v': (1, False),
        'c': (6, True)
    }

    @dataclass
    class Curve:
        marker: str
        body: List[int]

        def __post_init__(self):
            # Access the parent class's curve_types_to_expected_length
            expected_length = SVGParser.curve_types_to_expected_length[self.marker][0]
            multiples_expected = SVGParser.curve_types_to_expected_length[self.marker][1]

            if multiples_expected:
                if not (len(self.body)%expected_length) == 0:
                    raise ValueError(f"Unexpected curve length for type {self.marker}: {len(self.body)}!")
            else: 
                if not len(self.body) == expected_length:
                    raise ValueError(f"Unexpected curve length for type {self.marker}: {len(self.body)}!")

    def __init__(self):
        #TODO: maybe encoded curve types and metadata in a better object than this - include description of what the curve actually is/does
        # tuple: (expected_length, multiples_expected)
        self.parsing_inkscape_file = True

    def split_raw_curves(self, raw):
        split_at_letter = re.findall(r'[a-zA-Z][^a-zA-Z]*', raw)
        curves = []

        for curve_block in split_at_letter:
            marker = curve_block[0]

            if marker == 'z': #discard end markers
                continue

            body = [float(x) for x in re.findall(r'-?\d+\.?\d*', curve_block[1:])]
            curve_length = self.curve_types_to_expected_length[marker][0]
            single_curves = [body[i:i + curve_length] for i in range(0, len(body), curve_length)]
            for single_curve in single_curves:
                curves.append(self.Curve(
                    marker=marker,
                    body=single_curve
                ))
        return curves

    def discretize_bezier(self, node_set, prev_pt):
        nodes = np.asfortranarray([
            [prev_pt.x, prev_pt.x+node_set[0], prev_pt.x+node_set[2], prev_pt.x+node_set[4]],
            [prev_pt.y, prev_pt.y+node_set[1], prev_pt.y+node_set[3], prev_pt.y+node_set[5]],
        ])
        # print(nodes)
        bezier_curve_obj = bezier.Curve(nodes, degree=3)

        num_pts_in_curve = bezier_curve_obj.length / SEG_LENGTH
        s_vals = np.linspace(0.0, 1.0, math.ceil(num_pts_in_curve))
        evaluator_output = bezier_curve_obj.evaluate_multi(s_vals)
        
        pts = [CartesianPt(x=float(x), y=float(y)) for x, y in zip(evaluator_output[0], evaluator_output[1])]
        
        return pts



    def parse_multiple_curves(self, curves_raw):
        curves = self.split_raw_curves(curves_raw)
        
        pts = [CartesianPt(x=0, y=0)]
        for curve in curves:
            prev_pt = pts[-1]
            if curve.marker=='M' or curve.marker=="L": # moveto, lineto (absolute)
                # svg defines M and L as different, but since the ball can't float, to us they are the same
                pts.append(CartesianPt(x=curve.body[0], y=curve.body[1]))
            elif curve.marker=='m' or curve.marker=="l": # moveto, lineto (relative)
                pts.append(CartesianPt(x=prev_pt.x+curve.body[0], y=prev_pt.y+curve.body[1]))
            elif curve.marker=='h': # horizontal line
                pts.append(CartesianPt(x=prev_pt.x+curve.body[0], y=prev_pt.y))
            elif curve.marker=='v': # vertical line
                pts.append(CartesianPt(x=prev_pt.x, y=prev_pt.y+curve.body[0]))
            elif curve.marker=='c': # bezier curve
                pts.extend(self.discretize_bezier(curve.body, prev_pt))
            elif curve.marker=='z': # end of curve
                pass
            else:
                print(f"Encountered unexpected curve marker: {curve.marker}")
        
        return pts[1:] # remove artificial (0,0) point
            

    def get_layer_above_meat(self,layer):

        if self.parsing_inkscape_file:
            if layer.tag == "{http://www.w3.org/2000/svg}svg":
                for child in layer:
                    if child.tag == "{http://www.w3.org/2000/svg}g":
                        return child
            raise RuntimeError("Inkscape file not in expected format")
        
        else: 
            print(f"current layer.attrib: {layer.attrib}")
            if layer.attrib == {}:
                return None

            for child in layer:
                print(f"evaluating child: {child.attrib}")
                if child.attrib.get('d', None):
                    print("it's this one!")
                    return layer

            for child in layer:
                layer_below_evaluation = self.get_layer_above_meat(child)
                if layer_below_evaluation:
                    return layer_below_evaluation
            
            raise RuntimeError("oops, shouldn't have gotten here")

    def get_pts_from_file(self, file):
        tree = et.parse(file)
        root = tree.getroot()
        print(f"root: {root.attrib}")
        if root.attrib['version'] != '1.1':
            print(f"Warning! Unsupported svg version: {root.attrib['version']}")

        layer_above_meat = self.get_layer_above_meat(root)
        print(layer_above_meat.attrib)

        all_pts = []
        for child in layer_above_meat:
            curves_raw = child.attrib['d']
            all_pts.extend(self.parse_multiple_curves(curves_raw))
        
        return all_pts
    
    def convert_to_table_axes(self, pts):
        '''3 things that need to happen:
        - flip y axis
        - move origin to middle
        - convert to polar
        '''
        converted_pts = []
        for pt in pts:
            x = pt.x - 273
            y = 273 - pt.y
            polar_pt = cartesian_to_polar(CartesianPt(x=x, y=y))
            converted_pts.append(polar_pt)
        return converted_pts

def plot_pts(pts): 
    pts_decoded = [pt.to_tuple() for pt in pts]
    # print(pts_decoded)
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

def create_polar_plot(pts: List[PolarPt]):
    # Convert theta from degrees to radians for plotting
    ts_rad = [p.t * math.pi/180 for p in pts]
    rs = [p.r for p in pts]

    # Create polar plot
    plt.figure(figsize=(8, 8))
    ax = plt.subplot(111, projection='polar')
    ax.plot(ts_rad, rs, 'b.-', label='Path')
    ax.set_rmax(300)  # Set maximum radius to 300
    ax.set_rticks([0, 100, 200, 300])  # Set radius ticks
    ax.set_thetagrids(np.arange(0, 360, 45))  # Set theta grid lines every 45 degrees
    ax.grid(True)
    ax.set_title('Path in Polar Coordinates')
    plt.show()

def cartesian_to_polar(pt: CartesianPt) -> PolarPt:
    r = math.sqrt(pt.x**2 + pt.y**2)
    t = math.atan2(pt.y, pt.x)*180/math.pi
    return PolarPt(float(r), float(t))

if __name__ == "__main__":
    svg_file = "..\svg_examples\inkscape_hi.svg"
    # svg_file = "..\svg_examples\cabin.svg"
    # svg_file = "..\svg_examples\Archimedean_spiral.svg"
    # svg_file = "..\svg_examples\inkscape_spiral.svg"
    # svg_file = "..\svg_examples\eye-drops-svgrepo-com.svg"
    # svg_file = "..\svg_examples\chef-man-cap-svgrepo-com.svg"
    
    svg_parser = SVGParser()
    pts = svg_parser.get_pts_from_file(svg_file)
    polar_pts = svg_parser.convert_to_table_axes(pts)
    # plot_pts(pts)
    create_polar_plot(polar_pts)