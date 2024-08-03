import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import directed_hausdorff

def hausdorff_distance(A, B):
    forward = directed_hausdorff(A, B)[0]
    backward = directed_hausdorff(B, A)[0]
    return max(forward, backward)

def plot_hausdorff_distance(curve1, curve2):
    curve1 = np.array(curve1)
    curve2 = np.array(curve2)
    
    # Compute Hausdorff distance
    forward_distance = directed_hausdorff(curve1, curve2)[0]
    backward_distance = directed_hausdorff(curve2, curve1)[0]
    
    if forward_distance > backward_distance:
        distance = forward_distance
        max_curve = curve1
        other_curve = curve2
    else:
        distance = backward_distance
        max_curve = curve2
        other_curve = curve1
    
    # Find the points corresponding to the maximum Hausdorff distance
    max_distance_points = []
    max_distance = 0
    for point in max_curve:
        nearest_point = other_curve[np.argmin(np.linalg.norm(other_curve - point, axis=1))]
        d = np.linalg.norm(point - nearest_point)
        if d > max_distance:
            max_distance = d
            max_distance_points = [(point, nearest_point)]
        elif d == max_distance:
            max_distance_points.append((point, nearest_point))
    
    # Print the coordinates of the points between which the maximum Hausdorff distance occurs
    print("Coordinates of points between which maximum Hausdorff distance occurs:")
    for point_pair in max_distance_points:
        max_point, nearest_max_point = point_pair
        print(f"Point 1: {max_point}, Point 2: {nearest_max_point}")
    
    # Plot the curves
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))
    
    ax1.plot(curve1[:, 0], curve1[:, 1], 'bo-', label='EKF position', markersize=1)
    ax1.plot(curve2[:, 0], curve2[:, 1], 'rs-', label='AMCL position', markersize=1)
    ax1.set_title('Curves')
    ax1.legend()
    
    # Plot the distances
    for point in curve1:
        nearest_point = curve2[np.argmin(np.linalg.norm(curve2 - point, axis=1))]
        ax2.plot([point[0], nearest_point[0]], [point[1], nearest_point[1]], 'g--', lw=1)
    
    for point in curve2:
        nearest_point = curve1[np.argmin(np.linalg.norm(curve1 - point, axis=1))]
        ax2.plot([point[0], nearest_point[0]], [point[1], nearest_point[1]], 'm--', lw=1)
    
    # Highlight the maximum Hausdorff distance
    for point_pair in max_distance_points:
        max_point, nearest_max_point = point_pair
        ax2.plot([max_point[0], nearest_max_point[0]], [max_point[1], nearest_max_point[1]], 'r--', lw=2, label='Max Hausdorff Distance')
    
    ax2.set_title(f'Hausdorff Distance: {distance:f}')
    ax2.set_xlim(ax1.get_xlim())
    ax2.set_ylim(ax1.get_ylim())  # Ensure the same y-axis limits
    
    # Calculate and set the aspect ratio for ax2 to match ax1
    ax1_aspect = np.diff(ax1.get_xlim())[0] / np.diff(ax1.get_ylim())[0]
    ax2.set_aspect(ax1_aspect)

    zoom_margin = 0.2  # Zoom-in margin as a fraction of the distance
    for point_pair in max_distance_points:
        max_point, nearest_max_point = point_pair
        ax3.plot([max_point[0], nearest_max_point[0]], [max_point[1], nearest_max_point[1]], 'r--', lw=2)
        ax3.plot(max_curve[:, 0], max_curve[:, 1], 'bo-', markersize=8)
        ax3.plot(other_curve[:, 0], other_curve[:, 1], 'rs-', markersize=8)
        
        x_min = min(max_point[0], nearest_max_point[0]) - zoom_margin * distance
        x_max = max(max_point[0], nearest_max_point[0]) + zoom_margin * distance
        y_min = min(max_point[1], nearest_max_point[1]) - zoom_margin* distance
        y_max = max(max_point[1], nearest_max_point[1]) +zoom_margin  * distance
        
        ax3.set_xlim(x_min, x_max)
        ax3.set_ylim(y_min, y_max)
        ax3.set_title('Zoomed-in View Near Max Distance')
    
    ax3.set_aspect('equal')
    
    
    ax2.legend()
    plt.tight_layout()
    plt.show()





def read_points_from_file(filename):
    curves = []
    with open(filename, 'r') as file:
        for line in file:
            points = line.strip().split()
            curve = [tuple(map(float, point.strip('()').split(','))) for point in points]
            curves.append(curve)
    return curves

filename = 'a=100.txt'  # Replace with your file path
curves = read_points_from_file(filename)

if len(curves) < 2:
    raise ValueError("The file must contain at least two curves.")

curve1, curve2 = curves[0], curves[1]



plot_hausdorff_distance(curve1, curve2)