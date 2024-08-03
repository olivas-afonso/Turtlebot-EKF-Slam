import numpy as np
import matplotlib.pyplot as plt
import statistics as stats
import math

def read_points_from_file(filename):
    curves = []
    with open(filename, 'r') as file:
        for line in file:
            points = line.strip().split()
            curve = [list(map(float, point.strip('()').split(','))) for point in points]
            curves.append(curve)
    return curves

filename = 'position_check/a=100.txt'  # Replace with your file path
curves = read_points_from_file(filename)

if len(curves) < 2:
    raise ValueError("The file must contain at least two curves.")


curve1, curve2 = curves[0], curves[1]

distances = []
max_dis = -1
for i in range(len(curve2)):
    pos = curve2[i]
    min_dist = -1

    for j in range(len(curve1)):
        pos2 = curve1[j]
        dis = math.sqrt((pos[0]-pos2[0])**2 + (pos[1]-pos2[1])**2)
        if dis < min_dist or min_dist < 0:
            auxj = j
            min_dist = dis
    distances.append(min_dist)
    if min_dist > max_dis:
        max_dis = min_dist
        imax = i
        jmax = auxj

    
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))

curve1 = np.array(curve1)
curve2 = np.array(curve2)


max_point = curve2[imax]
nearest_max_point = curve1[jmax]
ax3.plot([max_point[0], nearest_max_point[0]], [max_point[1], nearest_max_point[1]], 'r--', lw=2)
ax3.plot(curve1[:, 0], curve1[:, 1], 'bo-', markersize=8)
ax3.plot(curve2[:, 0], curve2[:, 1], 'rs-', markersize=8)
        
zoom_margin = 3

x_min = min(max_point[0], nearest_max_point[0]) - zoom_margin * max_dis
x_max = max(max_point[0], nearest_max_point[0]) + zoom_margin * max_dis
y_min = min(max_point[1], nearest_max_point[1]) - zoom_margin* max_dis
y_max = max(max_point[1], nearest_max_point[1]) + zoom_margin  * max_dis
        
ax3.set_xlim(x_min, x_max)
ax3.set_ylim(y_min, y_max)
ax3.set_title('Zoomed-in View Near Max Distance')
    
ax1.plot(curve1[:, 0], curve1[:, 1], 'bo-', label='EKF position', markersize=1)
ax1.plot(curve2[:, 0], curve2[:, 1], 'rs-', label='AMCL position', markersize=1)
ax1.set_title('Curves')
ax1.legend()

ax2.plot(range(0, len(curve2)), distances, label='Distance Error', markersize=1)
ax2.set_title('Distance between closest EKF point for each AMCL point')

plt.tight_layout()
plt.show()
