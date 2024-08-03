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

counter = 0
distances = []
for i in range(len(curve2)-1):
    pos = curve2[i]
    next_pos = curve2[i+1]
    point_x = []
    point_y = []
    point = curve1[counter]
    while((point[0]-pos[0])**2 + (point[1]-pos[1])**2 < (point[0]-next_pos[0])**2 + (point[1]-next_pos[1])**2):
        point_x.append(point[0])
        point_y.append(point[1])
        counter += 1
        point = curve1[counter]
    
    mean_x = stats.mean(point_x)
    mean_y = stats.mean(point_y)

    S = np.array([[stats.variance(point_x), stats.covariance(point_x, point_y)], [stats.covariance(point_x, point_y), stats.variance(point_y)]])
    
    dis = np.array([pos[0]-mean_x, pos[1]-mean_y]) @ np.linalg.inv(S) @ np.array([[pos[0]-mean_x],[pos[1]-mean_y]])
    distances.append(math.sqrt(dis))

point_x = []
point_y = []
while counter < len(curve1):
    point_x.append(curve1[counter][0])
    point_y.append(curve1[counter][1])
    counter += 1

mean_x = stats.mean(point_x)
mean_y = stats.mean(point_y)
S = np.array([[stats.variance(point_x), stats.covariance(point_x, point_y)], [stats.covariance(point_x, point_y), stats.variance(point_y)]])
pos = curve2[len(curve2) - 1]

dis = np.array([pos[0]-mean_x, pos[1]-mean_y]) @ np.linalg.inv(S) @ np.array([[pos[0]-mean_x],[pos[1]-mean_y]])
distances.append(math.sqrt(dis))

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 6))

curve1 = np.array(curve1)
curve2 = np.array(curve2)
    
ax1.plot(curve1[:, 0], curve1[:, 1], 'bo-', label='EKF position', markersize=1)
ax1.plot(curve2[:, 0], curve2[:, 1], 'rs-', label='AMCL position', markersize=1)
ax1.set_title('Curves')
ax1.legend()

ax2.plot(range(0, len(curve2)), distances)

plt.tight_layout()
plt.show()
