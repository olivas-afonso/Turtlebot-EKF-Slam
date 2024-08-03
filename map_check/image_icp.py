
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from icp import icp

points = [0,0]
def read_points(file_path):
    offset_x=154
    offset_y=255
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            # Remove parentheses, strip any extra whitespace, and split by comma
            x, y = line.strip('()\n').split(',')
            x=int(x)+offset_x 
            y=int(y)+offset_y 
            # Convert to integers and append as a tuple to the points list
            points.append([int(x), int(y)])
    return np.array(points)

def plot_map_and_points(map_name, original_points, transformed_points):
    im = Image.open(map_name)
    plt.imshow(im)
    original_points = np.array(original_points)
    transformed_points = np.array(transformed_points)
    plt.scatter(original_points[:, 0], original_points[:, 1], c='blue', label='Original Points', s=1)  
    plt.scatter(transformed_points[:, 0], transformed_points[:, 1], c='red', label='Transformed Points', s=1)  
    plt.legend()
    plt.show()




file_path = 'div_landmark_pos_1000.txt'
points = read_points(file_path)


map_image_path = 'gmapping_big.png'
if map_image_path is None:
    raise ValueError("Map image could not be loaded.")

# Load map image using PIL
map_image = Image.open(map_image_path)

# Convert map image to grayscale
map_image_gray = map_image.convert('L')

# Convert image to numpy array
map_image_array = np.array(map_image_gray)

# Find all points in the map where the value is less than 10
aux = np.column_stack(np.where(map_image_array < 10))

map_points = []
for i in range(len(aux)):
     map_points.append([aux[i][1], aux[i][0]])

T, transformed_points = icp(map_points, points, verbose=True, distance_threshold=3)

print(T)

plot_map_and_points(map_image_path, points, transformed_points)
        