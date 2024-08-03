import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from scipy.optimize import minimize

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
    return points

# Example usage
file_path = 'landmark_pos_100.txt'
points_array = read_points(file_path)




def pixel_match(map_name, landmarks):
    im = Image.open(map_name)

    original_map = im.load()

    num_of_differences = 0
    for pos in landmarks:
        x = pos[0]
        y = pos[1]
        #print(pos)
        #print(original_map[x,y])
        if original_map[x,y] != (255,255,255,255):
            num_of_differences += 1

   
    return num_of_differences


def apply_transformation(landmarks, translation, rotation):
    rotation_rad = np.deg2rad(rotation)
    rotation_matrix = np.array([
        [np.cos(rotation_rad), -np.sin(rotation_rad)],
        [np.sin(rotation_rad), np.cos(rotation_rad)]
    ])
    transformed_landmarks = np.dot(landmarks, rotation_matrix.T)
    transformed_landmarks += translation
    return transformed_landmarks

def find_best_transformation(map_name, landmarks, translation_range, rotation_range):
    best_translation = None
    best_rotation = None
    max_differences = -1

    for tx in translation_range:
        print(tx)
        for ty in translation_range:
            for rot in rotation_range:
                transformed_landmarks = apply_transformation(landmarks, [tx, ty], rot)
                num_differences = pixel_match(map_name, transformed_landmarks)
                if num_differences > max_differences:
                    max_differences = num_differences
                    best_translation = [tx, ty]
                    best_rotation = rot

    return best_translation, best_rotation

def plot_map_and_points(map_name, original_points, transformed_points):
    im = Image.open(map_name)
    plt.imshow(im)
    original_points = np.array(original_points)
    transformed_points = np.array(transformed_points)
    plt.scatter(original_points[:, 0], original_points[:, 1], c='blue', label='Original Points', s=1)  
    plt.scatter(transformed_points[:, 0], transformed_points[:, 1], c='red', label='Transformed Points', s=1)  
    plt.legend()
    plt.show()

map_name = 'gmapping_big.png'
print(len(points_array))
print(pixel_match(map_name, points_array))

translation_range = range(-20, 20)  # From -10 to 10
rotation_range = range(-10, 10, 1)  # From -30 to 30 degrees

best_translation, best_rotation = find_best_transformation(map_name, points_array, translation_range, rotation_range)
print("Best Translation:", best_translation)
print("Best Rotation:", best_rotation)



transformed_points = apply_transformation(points_array, best_translation, best_rotation)

print(pixel_match(map_name, transformed_points))

while True:  
    continue


plot_map_and_points(map_name, points_array, transformed_points)

