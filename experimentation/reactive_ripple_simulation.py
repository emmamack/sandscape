import math
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass
from typing import List

LINE_SPACING = 10

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
    
    # Add arrows to show direction
    for i in range(len(points)-1):
        # Calculate the midpoint for the arrow
        mid_x = (points[i].x + points[i+1].x) / 2
        mid_y = (points[i].y + points[i+1].y) / 2
        
        # Calculate the direction vector
        dx = points[i+1].x - points[i].x
        dy = points[i+1].y - points[i].y
        
        # Normalize the direction vector
        length = math.sqrt(dx*dx + dy*dy)
        dx = dx/length
        dy = dy/length
        
        # Plot the arrow
        plt.arrow(mid_x - dx*2, mid_y - dy*2, 
                 dx*4, dy*4,
                 head_width=1, head_length=2, fc='red', ec='red')
    
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
    pts = [PolarPt(r=r, t=t) for r, t in zip(rs, ts)]
    return pts

def create_easy_initial_condition_polar():
    # rs = np.random.uniform(50, 70, 10)
    rs = [50]*10
    ts = list(range(0, 360, 36))
    pts = [PolarPt(r=r, t=t) for r, t in zip(rs, ts)]
    # pts.extend([PolarPt(r=50, t=0), PolarPt(r=90, t=0)])
    pts.extend([PolarPt(r=50, t=0)])
    print(pts)
    return pts

def create_easy_initial_condition_cartesian() -> List[CartesianPt]:
    points = []
    for i in range(11):
        x = math.cos(i*math.pi*2/10)*30
        y = math.sin(i*math.pi*2/10)*30
        points.append(CartesianPt(x, y))
    points.append(CartesianPt(33, 0))
    return points

def interpolate_cartesian(points: List[CartesianPt], distance_between_points: float) -> List[CartesianPt]:
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

def interpolate_polar(points: List[PolarPt], distance_between_points: float) -> List[PolarPt]:
    interpolated_points = []
    
    for i in range(len(points)-1):
        # Add the current point
        interpolated_points.append(points[i])
        
        # Convert current and next points to Cartesian for distance calculation
        curr_cart = polar_to_cartesian(points[i])
        next_cart = polar_to_cartesian(points[i+1])
        
        # Calculate the total distance between points
        dx = next_cart.x - curr_cart.x
        dy = next_cart.y - curr_cart.y
        total_distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate how many points we need to add
        num_points = int(total_distance / distance_between_points)
        
        # Add the interpolated points
        for j in range(1, num_points + 1):
            t = j * distance_between_points / total_distance
            # Interpolate in Cartesian coordinates
            x = curr_cart.x + t * dx
            y = curr_cart.y + t * dy
            # Convert back to polar
            interpolated_points.append(cartesian_to_polar(CartesianPt(x, y)))
    
    # Add the final point
    interpolated_points.append(points[-1])
    
    return interpolated_points

def get_first_pt_polar(points: List[CartesianPt]) -> CartesianPt:
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

def get_tangent_slope(p0, p1):
    return (p1.y - p0.y)/(p1.x-p0.x)

def is_direction_towards_origin(point, dir):
    # Dot product between position vector and direction vector
    dot_product = point.x * dir.x + point.y * dir.y
    
    if dot_product > 0:
        return 1  # Away from origin
    elif dot_product < 0:
        return -1  # Towards origin
    else:
        return 0  # Perpendicular

def get_position_from_target_pt(points, target_index):
    # Get the target point and its neighbors
    target_pt = points[target_index]
    # TODO: handle edge case where there's a line crossing
    # or maybe just transform set of points to not have line crossing as pre-processing
    prev_pt = points[target_index - 1]
    next_pt = points[target_index + 1]
    
    # Average the direction vectors to get the tangent direction
    avg_dir_x = (next_pt.x - prev_pt.x) / 2
    avg_dir_y = (next_pt.y - prev_pt.y) / 2
    
    # Calculate the perpendicular direction (rotate 90 degrees)
    perp_dir_x = -avg_dir_y
    perp_dir_y = avg_dir_x

    if is_direction_towards_origin(target_pt, CartesianPt(perp_dir_x, perp_dir_y)):
        perp_dir_x = -perp_dir_x
        perp_dir_y = -perp_dir_y
    
    # Normalize the perpendicular direction to get a unit vector
    magnitude = math.sqrt(perp_dir_x*perp_dir_x + perp_dir_y*perp_dir_y)
    unit_perp_x = perp_dir_x / magnitude
    unit_perp_y = perp_dir_y / magnitude
    
    points.append(
        CartesianPt(
            x = target_pt.x + unit_perp_x * LINE_SPACING, 
            y = target_pt.y + unit_perp_y * LINE_SPACING
        )
    )

    return points

points = create_easy_initial_condition_cartesian()
points = interpolate_cartesian(points, 2) 
for ind in range(1, 100):
    points = get_position_from_target_pt(points, ind)
create_cartesian_plot(points)

# points = create_easy_initial_condition_polar()
# points = interpolate_polar(points, 10)
# create_polar_plot(points)