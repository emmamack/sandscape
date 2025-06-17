import math
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass
from typing import List

@dataclass
class PolarPt:
    r: float
    t: float

@dataclass
class CartesianPt:
    x: float
    y: float

def cartesian_to_polar(pt: CartesianPt) -> PolarPt:
    r = math.sqrt(pt.x**2 + pt.y**2)
    t = math.atan2(pt.y, pt.x)*180/math.pi
    return PolarPt(float(r), float(t))

def polar_to_cartesian(pt: PolarPt) -> CartesianPt:
    t = pt.t*math.pi/180
    x = pt.r * math.cos(t)
    y = pt.r * math.sin(t)
    return CartesianPt(float(x), float(y))

def create_polar_plot(points: List[PolarPt]):
    # Convert theta from degrees to radians for plotting
    ts_rad = [p.t * math.pi/180 for p in points]
    rs = [p.r for p in points]

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

def create_cartesian_plot(points: List[CartesianPt]):
    plt.figure(figsize=(8, 8))
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    plt.plot(xs, ys, 'b.-', label='Path')
    plt.grid(True)
    plt.axis('equal')  # This ensures the plot is circular
    plt.title('Path in Cartesian Coordinates')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

def create_random_initial_condition_polar():
    # Generate 10 random radii between 0 and 50
    rs = np.random.uniform(0, 50, 10)
    # Generate 10 random angles between 0 and 360 degrees
    ts = np.random.uniform(0, 360, 10)
    return rs, ts

def create_easy_initial_condition_polar():
    rs = np.random.uniform(50, 70, 10)
    ts = list(range(0, 359, 39))
    return rs, ts

def create_easy_initial_condition_cartesian() -> List[CartesianPt]:
    points = []
    for i in range(11):
        x = math.cos(i*math.pi*2/10)*30
        y = math.sin(i*math.pi*2/10)*30
        points.append(CartesianPt(x, y))
    points.append(CartesianPt(33, 0))
    return points

def interpolate(points: List[CartesianPt], distance_between_points: float) -> List[CartesianPt]:
    interpolated_points = []
    
    for i in range(len(points)-1):
        # Add the current point
        interpolated_points.append(points[i])
        
        # Calculate the total distance between points
        dx = points[i+1].x - points[i].x
        dy = points[i+1].y - points[i].y
        total_distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate how many points we need to add
        num_points = int(total_distance / distance_between_points)
        
        # Add the interpolated points
        for j in range(1, num_points + 1):
            t = j * distance_between_points / total_distance
            x = points[i].x + t * dx
            y = points[i].y + t * dy
            interpolated_points.append(CartesianPt(x, y))
    
    # Add the final point
    interpolated_points.append(points[-1])
    
    return interpolated_points

def get_first_pt(points: List[CartesianPt]) -> CartesianPt:
    pass
    '''Ideas:
    can go anywhere that's one radius away from starting point
    go "out" from middle by one radius amount, check that not radius hit
    "out" can be in reference to the actual origin or the shape in general.
    can probably get away with actual origin since the shape must necessarily
    include the origin.
    need to be able to kinda sense what's nearby and what direction will get you
    on a nice tangent path.
    we vaguely want to dictate which way the spiral goes, let's say counterclockwise
    for now, so bias towards counterclockwise from current point
    '''

# Example usage
points = create_easy_initial_condition_cartesian()
points = interpolate(points, 2)  # Points will be 2 units apart
create_cartesian_plot(points)
