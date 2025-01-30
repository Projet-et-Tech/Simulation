import time
import numpy as np

def distance_3d(point1, point2):
    return np.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2 + (point2[2] - point1[2])**2)

def distance_2d(point1, point2):
    return np.hypot(point1, point2)

def interpolate_position(start, end, steps):
    """
    Crée une interpolation linéaire entre deux points pour un déplacement fluide.

    Args:
        start (tuple): Coordonnées (x, y) du point de départ.
        end (tuple): Coordonnées (x, y) du point d'arrivée.
        steps (int): Nombre d'étapes pour l'interpolation.

    Returns:
        list: Liste des coordonnées interpolées.
    """
    return [(start[0] + (end[0] - start[0]) * t / steps,
             start[1] + (end[1] - start[1]) * t / steps) for t in range(steps + 1)]

def position_to_grid_index(position, x_dim, y_dim, cell_size):
    x_idx = int((position[0] / cell_size) + (x_dim // 2))
    y_idx = int((position[1] / cell_size) + (y_dim // 2))
    return y_idx, x_idx

def grid_index_to_position(grid_index, x_dim, y_dim, cell_size):
    x_idx, y_idx = grid_index
    # Calculate the position based on the grid index
    x_position = (x_idx - (x_dim // 2)) * cell_size
    y_position = (y_idx - (y_dim // 2)) * cell_size
    return (x_position, y_position)


def mark_can_on_grid(center, radius, x_dim, y_dim, cell_size):
    y_idx, x_idx = position_to_grid_index(center, x_dim, y_dim, cell_size)
    radius_cells = int(radius / cell_size)

    ox, oy = [], []
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            ny, nx = y_idx + dy, x_idx + dx
            if 0 <= ny < y_dim and 0 <= nx < x_dim:
                if np.sqrt(dx**2 + dy**2) <= radius_cells:
                    ox.append(nx)
                    oy.append(ny)
    return oy, ox

def in_bound(pos, x_dim, y_dim):
    x, y = pos
    return 0 <= x < x_dim and 0 <= y < y_dim

def determine_direction(x1, y1, x2, y2):
    """Determine the direction of movement between two points."""
    if y1 == y2:  # Horizontal
        return "horizontal"
    elif x1 == x2:  # Vertical
        return "vertical"
    elif x2 > x1 and y2 > y1:  # Top-Right Diagonal
        return "top-right"
    elif x2 < x1 and y2 > y1:  # Top-Left Diagonal
        return "top-left"
    elif x2 > x1 and y2 < y1:  # Bottom-Right Diagonal
        return "bottom-right"
    elif x2 < x1 and y2 < y1:  # Bottom-Left Diagonal
        return "bottom-left"
    else:
        return "none"

def extract_major_positions(path_x, path_y):
    """Extract major positions from separate lists of x and y coordinates."""
    if not path_x or not path_y or len(path_x) != len(path_y):
        return [], []  # Return empty lists if input is invalid

    len_x = len(path_x)
    major_positions = []
    current_start = 0
    current_direction = determine_direction(path_x[0], path_y[0], path_x[1], path_y[1])

    for i in range(2, len_x):
        direction = determine_direction(path_x[i-1], path_y[i-1], path_x[i], path_y[i])
        if direction != current_direction:
            major_positions.append(current_start)
            # Add the current position as a new major position
            current_start = i-1
            current_direction = direction

    # Add the last line's start point if it's not already added
    if current_start != len_x-1:
        major_positions.append(current_start)
        major_positions.append(len_x-1)

    return list(set(major_positions))

def get_distances(list_x, list_y):
    len_y = len(list_y)
    print(len_y)
    assert len(list_x) == len_y
    return [np.hypot(list_x[i]-list_x[i+1], list_y[i]-list_y[i+1]) for i in range(len_y-1)]

def cluster_major_positions(major_positions, px, py, threshold):
    """Cluster major positions based on distance threshold."""
    clusters = []
    current_cluster = []

    # Iterate through the major positions
    for i in range(len(major_positions)-1):
        mp1 = major_positions[i]
        mp2 = major_positions[i+1]
        point_a = (px[mp1], py[mp1])
        point_b = (px[mp2], py[mp2])
        
        # Compute the distance between consecutive points
        distance = np.hypot(point_a[0]-point_b[0], point_a[1]-point_b[1])

        if distance == 0:
            continue
        elif distance <= threshold:
            # If the distance is below the threshold, add both points to the current cluster
            if mp1 not in current_cluster:
                current_cluster.append(mp1)
            if mp2 not in current_cluster:
                current_cluster.append(mp2)
        else:
            # If the distance exceeds the threshold, save the current cluster if it has points
            if current_cluster:
                clusters.append(current_cluster)
                current_cluster = []  # Reset the current cluster

    # Add the last cluster if it has points
    if current_cluster:
        clusters.append(current_cluster)

    return clusters

def extract_path(px, py, threshold):
    print("Extracting important coordinates...")
    time_start = time.time()
    major_positions = extract_major_positions(px, py)
    clusters = cluster_major_positions(major_positions, px, py, threshold)

    mp_in_cluster = [mp for c in clusters for mp in c]
    barycenters = []
    for c in clusters:
        list_pos_x =[px[mp] for mp in c]
        list_pos_y =[py[mp] for mp in c]
        len_pos = len(c)
        barycenter = (int(sum(list_pos_x)/len_pos), int(sum(list_pos_y)/len_pos))
        barycenters.append(barycenter)
    
    cpx, cpy = [], []
    i, id_cluster = 0, 0
    while i < len(major_positions):
        mp = major_positions[i]
        if mp not in mp_in_cluster:
            cpx.append(px[mp])
            cpy.append(py[mp])
            i+=1
        else:
            cpx.append(barycenters[id_cluster][0])
            cpy.append(barycenters[id_cluster][1])
            i+=len(clusters[id_cluster])
            id_cluster += 1
    extraction_time = time.time() - time_start
    return cpx, cpy, extraction_time