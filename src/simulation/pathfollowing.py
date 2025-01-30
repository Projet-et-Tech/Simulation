import time
import numpy as np

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

    return list(major_positions)

def get_distances(list_x, list_y):
    len_y = len(list_y)
    assert len(list_x) == len_y
    return [np.hypot(list_x[i]-list_x[i+1], list_y[i]-list_y[i+1]) for i in range(len_y-1)]

def cluster_major_positions(major_positions, px, py, threshold):
    """Cluster major positions based on distance threshold."""
    clusters = []
    current_cluster = []

    # Iterate through the major positions
    for i in range(len(major_positions)-1):
        # mp sont des indices !!!
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

def extract_waypoints(px, py, threshold):
    print("Extracting waypoints...")
    time_start = time.time()
    major_positions = extract_major_positions(px, py)
    clusters = cluster_major_positions(major_positions, px, py, threshold)

    # mp sont des indices !!!
    mp_in_cluster = [mp for c in clusters for mp in c]
    barycenters = []
    for c in clusters:
        if 0 in c:
            barycenters.append((px[0], py[0]))
        elif (len(px)-1) in c:
            barycenters.append((px[-1], py[-1]))
        else:
            id_bary = c[len(c)//2]
            barycenters.append((px[id_bary], py[id_bary]))
    
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