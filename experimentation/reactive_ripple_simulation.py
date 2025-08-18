import math
import matplotlib.pyplot as plt
import numpy as np
from dataclasses import dataclass
from typing import List

LINE_SPACING = 10
INTERP_SPACING = 2

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

def create_cartesian_plot(pts: List[CartesianPt], highlight_pt: CartesianPt):
    plt.figure(figsize=(8, 8))
    
    # Create rainbow color gradient
    num_pts = len(pts)
    colors = plt.cm.rainbow(np.linspace(1, 0, num_pts))
    
    # Plot each segment with its own color
    for i in range(len(pts)-1):
        xs = [pts[i].x, pts[i+1].x]
        ys = [pts[i].y, pts[i+1].y]
        plt.plot(xs, ys, color=colors[i], linewidth=2)
    
    # Plot individual pts with their colors
    xs = [p.x for p in pts]
    ys = [p.y for p in pts]
    plt.scatter(xs, ys, c=colors, s=30, zorder=5)
    
    # Highlight the specified point
    plt.scatter(highlight_pt.x, highlight_pt.y, c='red', s=100, zorder=10, 
                edgecolors='black', linewidth=2, marker='o')
    
    # Add arrows to show direction
    # for i in range(len(pts)-1):
    #     mid_x = (pts[i].x + pts[i+1].x) / 2
    #     mid_y = (pts[i].y + pts[i+1].y) / 2
        
    #     # Calculate the direction vector
    #     dx = pts[i+1].x - pts[i].x
    #     dy = pts[i+1].y - pts[i].y
        
    #     # Normalize the direction vector
    #     length = math.sqrt(dx*dx + dy*dy)
    #     dx = dx/length
    #     dy = dy/length
        
    #     # Plot the arrow
    #     plt.arrow(mid_x - dx*2, mid_y - dy*2, 
    #              dx*4, dy*4,
    #              head_width=1, head_length=2, fc='black', ec='black')
    
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
    return pts

def create_easy_initial_condition_cartesian() -> List[CartesianPt]:
    pts = []
    for i in range(11):
        x = math.cos(i*math.pi*2/10)*30
        y = math.sin(i*math.pi*2/10)*30
        pts.append(CartesianPt(x, y))
    pts.append(CartesianPt(33, 0))
    return pts

def create_random_initial_condition_cartesian() -> List[CartesianPt]:
    xs = np.random.uniform(-10, 10, 10)
    ys = np.random.uniform(-10, 10, 10)
    pts = [CartesianPt(x=x, y=y) for x, y in zip (xs, ys)]
    return pts

def interpolate_batch(pts: List[CartesianPt], start_ind, end_ind):
    interpolated_pts = [pts[start_ind]]
    
    for i in range(start_ind, end_ind-1):
        interp_line = interpolate_single(pts[i], pts[i+1])
        interpolated_pts.extend(interp_line)
    
    return interpolated_pts

def interpolate_polar(pts: List[PolarPt], distance_between_pts: float) -> List[PolarPt]:
    interpolated_pts = []
    
    for i in range(len(pts)-1):
        # Add the current pt
        interpolated_pts.append(pts[i])
        
        # Convert current and next pts to Cartesian for distance calculation
        curr_cart = polar_to_cartesian(pts[i])
        next_cart = polar_to_cartesian(pts[i+1])
        
        # Calculate the total distance between pts
        dx = next_cart.x - curr_cart.x
        dy = next_cart.y - curr_cart.y
        total_distance = math.sqrt(dx*dx + dy*dy)
        
        # Calculate how many pts we need to add
        num_pts = int(total_distance / distance_between_pts)
        
        # Add the interpolated pts
        for j in range(1, num_pts + 1):
            t = j * distance_between_pts / total_distance
            # Interpolate in Cartesian coordinates
            x = curr_cart.x + t * dx
            y = curr_cart.y + t * dy
            # Convert back to polar
            interpolated_pts.append(cartesian_to_polar(CartesianPt(x, y)))
    
    # Add the final pt
    interpolated_pts.append(pts[-1])
    
    return interpolated_pts

def get_first_pt_polar(pts: List[CartesianPt]) -> CartesianPt:
    pass
    '''Ideas:
    can go anywhere that's one radius away from starting pt
    go "out" from middle by one radius amount, check that not radius hit
    "out" can be in reference to the actual origin or the shape in general.
    can probably get away with actual origin since the shape must necessarily
    include the origin.
    need to be able to kinda sense what's nearby and what direction will get you
    on a nice tangent path.
    we vaguely want to dictate which way the spiral goes, let's say counterclockwise
    for now, so bias towards counterclockwise from current pt
    '''

def get_tangent_slope(p0, p1):
    return (p1.y - p0.y)/(p1.x-p0.x)

def is_direction_towards_origin(pt, dir):
    # Dot product between position vector and direction vector
    dot_product = pt.x * dir.x + pt.y * dir.y
    
    if dot_product > 0:
        return 1  # Away from origin
    elif dot_product < 0:
        return -1  # Towards origin
    else:
        return 0  # Perpendicular

def remove_duplicates(pts):
    duplicates_removed = [pts[0]]
    for i in range(1, len(pts)):
        pt = pts[i]
        prev_pt = pts[i-1]
        if not (pt.x == prev_pt.x and pt.y == prev_pt.y):
            duplicates_removed.append(pt)
    return duplicates_removed

def interpolate_single(p0, p1):
    total_distance = get_dist(p0, p1)
    
    if total_distance < INTERP_SPACING*1.5: # make sure we don't add points which are basically duplicates
        return [p1]
    
    num_pts = math.floor(total_distance / INTERP_SPACING)
    
    pts_to_add = []
    # Add the interpolated pts
    for j in range(1, num_pts + 1):
        t = j * INTERP_SPACING / total_distance
        x = p0.x + t * (p1.x - p0.x)
        y = p0.y + t * (p1.y - p0.y)
        pts_to_add.append(CartesianPt(x, y))
    
    pts_to_add.append(p1)
    return pts_to_add

def get_position_from_target_pt(pts, target_index):
    # Get the target pt and its neighbors
    target_pt = pts[target_index]
    # TODO: handle edge case where there's a line crossing
    # or maybe just transform set of pts to not have line crossing as pre-processing
    prev_pt = pts[target_index - 1]
    next_pt = pts[target_index + 1]
    
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
    
    target_pos = CartesianPt(
            x = target_pt.x + unit_perp_x * LINE_SPACING, 
            y = target_pt.y + unit_perp_y * LINE_SPACING
    )

    return target_pos

def get_dist(p0, p1):
    return math.sqrt((p1.x - p0.x)**2 + (p1.y - p0.y)**2)

def check_for_collision_efficient(pts, proposed_pt):
    """
    Efficiently check if any point in pts is within LINE_SPACING of proposed_pt.
    Uses a grid-based spatial data structure for O(1) average case lookup.
    """
    # Create a grid where each cell is LINE_SPACING x LINE_SPACING
    # This ensures points in the same cell are within LINE_SPACING of each other
    grid = {}
    
    # Build the spatial index for existing points
    for pt in pts:
        # Calculate grid coordinates
        grid_x = int(pt.x / LINE_SPACING)
        grid_y = int(pt.y / LINE_SPACING)
        
        # Add point to all neighboring cells (including diagonal neighbors)
        # This handles edge cases where points are near cell boundaries
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                cell_key = (grid_x + dx, grid_y + dy)
                if cell_key not in grid:
                    grid[cell_key] = []
                grid[cell_key].append(pt)
    
    # Check if proposed point collides with any existing points
    proposed_grid_x = int(proposed_pt.x / LINE_SPACING)
    proposed_grid_y = int(proposed_pt.y / LINE_SPACING)
    
    # Check the cell where proposed point would be placed and neighboring cells
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            cell_key = (proposed_grid_x + dx, proposed_grid_y + dy)
            if cell_key in grid:
                for existing_pt in grid[cell_key]:
                    if get_dist(proposed_pt, existing_pt) < LINE_SPACING:
                        return True  # Collision detected
    
    return False  # No collision

def check_for_collision(pts, proposed_pt):
    trailing_grace = 2*LINE_SPACING/INTERP_SPACING #TODO: this needs work
    for pt in pts[:-1*int(trailing_grace)]:
        if get_dist(pt, proposed_pt) < LINE_SPACING-.001: #deal with floating point errors
            return True
    return False

def get_next_legal_target(pts, test_ind):
    current_pt = pts[-1]
    
    found_target = False
    while not found_target:
        test_position = get_position_from_target_pt(pts, test_ind)
        if not check_for_collision(pts, test_position):
            return test_ind, test_position
        test_ind += 1



pts = create_easy_initial_condition_cartesian()
# pts = create_random_initial_condition_cartesian()
pts = interpolate_batch(pts, 0, len(pts)) 
target_pt_ind = 1
while target_pt_ind <500:
    try:
        print("---------")
        next_pt = get_position_from_target_pt(pts, target_pt_ind)
        if check_for_collision(pts, next_pt):
            print("collision detected")
            target_pt_ind, next_pt = get_next_legal_target(pts, target_pt_ind)
        else:
            target_pt_ind += 1
        next_pts_interp = interpolate_single(pts[-1], next_pt)
        print(f"target_pt {target_pt_ind}: {pts[target_pt_ind]}")
        print(f"base pt: {pts[-1]} | going to pt {next_pt}")
        print(f"next_pts_interp: {next_pts_interp}")
        pts.extend(next_pts_interp)
    except Exception as e:
        print(f"Failed on target pt {pts[target_pt_ind]} with error {str(e)}")
        break

create_cartesian_plot(pts, pts[target_pt_ind])
# TODO: next up is handling the crossings that come with harder initial condition