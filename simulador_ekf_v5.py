import cv2
import numpy as np
import math
import random
from numpy.linalg import inv
import copy
from PIL import Image

import time

start_time = time.time()

timestamps = []
positions = []
quaternions = []
linear_velocities = []
angular_velocities = []
lidar_ranges_list = []
lidar_ranges = []
lidar_intensities =[]

def file_to_arrays(q):
    # Initialize arrays to store Euler angles
    xy_yaw_positions = []
    linear_angular_velocities = []
    
    # Iterate over each quaternion
    for quaternion, position, linear_velocity, angular_velocity, lidar_range in zip(q, positions, linear_velocities, angular_velocities, lidar_ranges):
        # Extract quaternion elements
        q_w, q_x, q_y, q_z = quaternion['w'], quaternion['x'], quaternion['y'], quaternion['z']
        
        # Yaw (psi)
        yaw = math.atan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y**2 + q_z**2))
        if yaw < 0:
            yaw += 2 * np.pi

        # Extract x and y position values
        x = position['x']
        y = position['y']

        linear_velocity_x = linear_velocity['x']
        angular_velocity_z = angular_velocity['z']

        xy_yaw_positions.append([x, y, yaw])
        linear_angular_velocities.append((linear_velocity_x, angular_velocity_z))
        lidar_ranges_list.append(lidar_range)
    
    return  xy_yaw_positions, linear_angular_velocities, lidar_ranges_list

with open('corredor_bag.txt', 'r') as f:
    lines = f.readlines()
    i = 0
    save_next_odom = False
    while i < len(lines):
            
        if lines[i].startswith('Topic'):
            topic = lines[i].split(':')[1].strip()

            if topic == '/scan':

                timestamp = float(lines[i+1].split(':')[1].strip())
                timestamps.append(timestamp)
                # Extract Lidar information
                ranges_line = lines[i + 3].strip().split(':')[1].strip()
                ranges = [float(val) for val in ranges_line[1:-1].split(',')]
                lidar_ranges.append(ranges)

                intensities_line = lines[i + 4].strip().split(':')[1].strip()
                intensities = [float(val) for val in intensities_line[1:-1].split(',')]
                lidar_intensities.append(intensities)
                
                # Set flag to save next odom message
                save_next_odom = True

            elif topic == '/odom' and save_next_odom:
                # Extract Odometry information
                position_line = lines[i + 3].strip().split(':')[1].strip().split(',')
                position = {'x': float(position_line[0].split('=')[1]),
                            'y': float(position_line[1].split('=')[1]),
                            'z': float(position_line[2].split('=')[1])}
                positions.append(position)

                quaternion_line = lines[i + 4].strip().split(':')[1].strip().split(',')
                quaternion = {'x': float(quaternion_line[0].split('=')[1]),
                              'y': float(quaternion_line[1].split('=')[1]),
                              'z': float(quaternion_line[2].split('=')[1]),
                              'w': float(quaternion_line[3].split('=')[1])}
                quaternions.append(quaternion)

                linear_velocity_line = lines[i + 5].strip().split(':')[1].strip().split(',')
                linear_velocity = {'x': float(linear_velocity_line[0].split('=')[1]),
                                   'y': float(linear_velocity_line[1].split('=')[1]),
                                   'z': float(linear_velocity_line[2].split('=')[1])}
                linear_velocities.append(linear_velocity)

                angular_velocity_line = lines[i + 6].strip().split(':')[1].strip().split(',')
                angular_velocity = {'x': float(angular_velocity_line[0].split('=')[1]),
                                    'y': float(angular_velocity_line[1].split('=')[1]),
                                    'z': float(angular_velocity_line[2].split('=')[1])}
                angular_velocities.append(angular_velocity)

                # Reset flag
                save_next_odom = False

            # Move to the next topic
            i += 6
        else:
            i += 1
xy_positions, velocity, ranges = file_to_arrays(quaternions)

class EKF:
     
    def __init__(self, pos):

        self.x0 = 1.10652899742
        self.y0 = 0.000232384933042
        self.tt0 = 3.1174481532182856

        self.mu = pos # Estado atual
        self.sigma = np.array([[0, 0, 0],
                               [0, 0, 0],
                               [0, 0, 0]]) # Covariancia atual
        self.Q = np.array([[0.001, 0.0],
                           [0.0, 0.001]]) # Ruido dos sensores <- ALTERAR ISTO PARA OS VALORES REAIS
        self.n = 0 # N de objetos no mapa
        self.f = np.identity(3) # Matriz auxiliar. Tem sempre tres linhas e e a identidade nas primeiras 3 colunas. Por cada objeto no mapa, adicionar 2 colunas 
                                # de zeros a essa matriz
        self.map_index = (0,0)
        self.minimap_length = float('inf')

    def set_square(self, index, minimap_length):
        self.map_index = index
        self.minimap_length = minimap_length

    def predict_state(self,u,dt):
        
        x = self.mu[0]
        y = self.mu[1]
        theta = self.mu[2]
        v = u[0]
        w = u[1]
        if w == 0:
            x = x + v*math.cos(theta)*dt #se w=0, deslocamento linha reta
            y = y + v*math.sin(theta)*dt
        else:
            x = x + v/w*(-math.sin(theta) + math.sin(theta + w*dt))
            y = y + v/w*(math.cos(theta) - math.cos(theta + w*dt))   # Ver Probabilistic Robotics, pag. 101
            theta = theta + w *dt 
        
        mu_bar = [x, y, theta]
        return mu_bar

    def predict_cov(self,u,dt):
            
        x = self.mu[0]
        y = self.mu[1]
        theta = self.mu[2]
        v = u[0]
        w = u[1]
        if w==0:
            aux_matrix = np.array([[0, 0, -v*(math.sin(theta)*dt)],
                                   [0, 0, v*(math.cos(theta)*dt)],
                                   [0, 0, 0]])
        else:
            aux_matrix = np.array([[0, 0, v/w * (-math.cos(theta)+math.cos(theta + w*dt))],
                                [0, 0, v/w * (-math.sin(theta)+math.sin(theta+w*dt))],
                                [0, 0, 0]])
            

        G = np.identity(2*self.n + 3) + (np.transpose(self.f)) @ aux_matrix @ self.f # Derivada do modelo dinamico

        R = np.array([[(0.001095*np.cos(theta)+0.0001)**2, 0, 0], 
                      [0,(0.001095*np.sin(theta)+0.0001)**2,0], 
                      [0,0,0.0010128**2]]) # Variancias do processo no referencial do robo



        sigma_bar = G @ self.sigma @ np.transpose(G) + (np.transpose(self.f)) @ R @ self.f # Assumir que nao existe ruido no processo
        return sigma_bar

    def predict_obs(self, mu_bar, sigma_bar):
        
        z_bar_list = []
        H_list = []
        K_list = []
        Psi_list = []

        for k in range(self.n):

            px = self.mu[2*k+3]
            py = self.mu[2*k+4]
            dx = px - mu_bar[0]
            dy = py - mu_bar[1]
            q = dx**2 + dy**2
            
            phi = math.atan2(dy,dx)
            if phi < 0:
                phi += math.pi*2
            z_bar = np.array([math.sqrt(q), phi - mu_bar[2]])
            
            if z_bar[1] < 0:
                z_bar[1] += 2 * math.pi
            z_bar_list.append(z_bar) 

            # Calculo da matriz H
            aux = np.zeros((2, 2*self.n+3))
            aux[0][2*k + 3] = 1
            aux[1][2*k + 4] = 1
            Fk = np.vstack((self.f, aux))

            aux = np.array([[-math.sqrt(q)*dx, -math.sqrt(q)*dy, 0, math.sqrt(q)*dx, math.sqrt(q)*dy],
                            [dy, -dx, -q, -dy, dx]])
            H = 1/q * aux @ Fk
            H_list.append(H)

            Q = self.get_noise_matrix(z_bar)
            # Calculo da matriz Psi
            Psi = H @ sigma_bar @ np.transpose(H) + Q
             # so se usa o inverso
            Psi_list.append(inv(Psi))


            # Calculo da matriz K
            
            K = sigma_bar @ np.transpose(H) @ inv(Psi)
            K_list.append(K)

        return z_bar_list, H_list, Psi_list, K_list
        
    
    def append_state(self, sigma_bar, state_to_append):
        New_state = (state_to_append[0], state_to_append[1])

        self.mu.extend(New_state)
            
        sigma_bar = np.hstack((sigma_bar, np.zeros((self.n*2 + 3, 2))))
        sigma_bar = np.vstack((sigma_bar, np.zeros((2, self.n*2 + 5))))

     
            
        sigma_bar[(2*self.n + 3)][(2*self.n + 3)] = 1
        sigma_bar[(2*self.n + 4)][(2*self.n + 4)] = 1

        self.f = np.hstack((self.f, np.zeros((3,2))))
        return sigma_bar

    def matching(self, r: list, mu_bar):
        minimum_range = 0.12 
        maximum_range = 1
        n = len(r)
        i = 0
        while i < n:
            curr_r = r[i]

            if curr_r[0] > maximum_range or curr_r[0] < minimum_range:
                r.pop(i)
                n -= 1
                continue

            px = mu_bar[0] + r[i][0]*math.cos(r[i][1] + mu_bar[2])
            py = mu_bar[1] + r[i][0]*math.sin(r[i][1] + mu_bar[2]) 

            aux1 = -(px - self.x0) + self.minimap_length/2 - minimap_length*self.map_index[0]
            aux2 = (py - self.y0 + self.minimap_length/2) - minimap_length*self.map_index[1]

            if aux1 < 0 or aux1 > self.minimap_length or aux2 < 0 or aux2 > self.minimap_length:
                r.pop(i)
                n -= 1
                continue
            i += 1
        return r

    def new_state(self,u,r, odom, dt):

        state_to_append =[]

        mu_bar = copy.deepcopy(self.mu)
        
        # Comecar por prever o nosso estado seguinte
        mu_bar[0:3] = self.predict_state(u, dt)
        sigma_bar = self.predict_cov(u, dt)
        r = self.matching(r, mu_bar)
        
        # O parametro a e o que ira ditar se estamos a ver um novo objeto ou nao. Quanto menor o a, maior a resolucao do nosso mapa, mas tambem mais tempo
        # demorara o nosso algoritmo a fazer os calculos
        a = 100 
        # Lista das correspondencias entre cada medicao e cada objeto conhecido no nosso mapa
        N = len(r) # Numero de medicoes de range
        #z = self.form_measurements(r) # Adicionar as medicoes de range as medicoes de phi
       
         # Estados que temos de adicionar ao nosso vetor de estados, caso encontremos um novo objeto
            # N de objetos a adicionar ao mapa
        for i in range(N):
            pi_min = -1
            if self.n == 0:
                # Caso nao hajam objetos no mapa, entao tudo o que o robo esta a ver e novo.

                ##### ISTO E ALGO A REVER PORQUE PODE nao ESTAR CORRETO. NUMA PRIMEIRA ITERACAO, ESTAREMOS SO A ADICIONAR ESTADOS AO ALGORITMO SEM TER EM CONTA
                ##### O RESTO DO FILTRO DE KALMAN   
                                                                                    
                px = mu_bar[0] + r[0][0]*math.cos(r[0][1] + mu_bar[2])
                py = mu_bar[1] + r[0][0]*math.sin(r[0][1] + mu_bar[2])
                state_to_append = (px, py)
                sigma_bar = self.append_state(sigma_bar, state_to_append)
                mu_bar.append(px)
                mu_bar.append(py)

                
                k_min = self.n
                self.n += 1

                continue
            
            z_bar_list, H_list, Psi_list, K_list = self.predict_obs(mu_bar, sigma_bar)

            for k in range(self.n):
             
                #print(k)
                # Para cada objeto k no nosso mapa, calcular a nossa funcao de custo e registar o objeto que a minimiza.
                
                pi = (r[i] - z_bar_list[k]) @ Psi_list[k] @ np.transpose(r[i] - z_bar_list[k])
                
                if pi < pi_min or pi_min < 0:
                    pi_min = pi
                    
                    k_min=k
                
                
            if pi_min > a:
                #print("OI")
                # Se o custo minimo for maior que a, entao estamos a ver uma coisa nova. Devemos adicionar isto aos estados
                
                px = mu_bar[0] + r[i][0]*math.cos(r[i][1] + mu_bar[2])
                py = mu_bar[1] + r[i][0]*math.sin(r[i][1] + mu_bar[2])
                state_to_append = (px, py)
                sigma_bar = self.append_state(sigma_bar, state_to_append)
                mu_bar = np.append(mu_bar, px)
                mu_bar = np.append(mu_bar, py)

                k_min = self.n
                self.n += 1

                z_bar_list, H_list, Psi_list, K_list = self.predict_obs(mu_bar, sigma_bar)
                
            
            mu_bar += K_list[k_min] @ (r[i] - z_bar_list[k_min])
            sigma_bar = (np.identity(self.n*2 + 3) - K_list[k_min] @ H_list[k_min]) @ sigma_bar
            
        
        

        #####################################################################################
        # Nesta parte incorporamos os dados de odometria na estimacao do estado. De notar que a odometria nao qualquer dependencia na posicao dos objetos,
        # pelo que nao os devera afetar nesta fase.
        z_bar_odom = np.array([[mu_bar[0]],
                                [mu_bar[1]],
                                [mu_bar[2]]])
        teta=self.mu[2]
       
        Q = np.array([[(0.001095*np.cos(teta) + 0.0001)**2, 0, 0],
                      [0, (0.001095*np.sin(teta) + 0.0001)**2, 0],
                      [0, 0, 0.001095**2]])
        
        H = np.identity(3)                     #       [1 0 0 0 0 ...]
        for i in range(self.n):                #   H = [0 1 0 0 0 ...]
            H = np.hstack((H, np.zeros((3, 2))))      #       [0 0 1 0 0 ...]
            #print(H)
        

        Psi = H @ sigma_bar @ np.transpose(H) + Q
        K = sigma_bar @ np.transpose(H) @ inv(Psi)
    
        
        aux =  mu_bar + np.transpose(K @ (odom - z_bar_odom))
        self.mu = aux[0].tolist()
        self.sigma = (np.identity(self.n*2+3) - K @ H) @ sigma_bar
        
        
        #####################################################################################
        return self.mu

    def form_measurements(self,r):
        phi = 0
        dphi = 2*math.pi/len(r)
        z = []
        for i in range(len(r)):
            z.append(np.array([r[i], phi]))
            phi += dphi
        z=np.array(z)

        return z
    
    def get_noise_matrix(self,z):
       
        var_phi = (0.5 * math.pi /180)**2
        if z[0] >= 0.5:
            var_r = (z[0] * 0.035)**2
        else:
            var_r = (0.01)**2
        
        return np.array([[var_r, 0], [0, var_phi]])
    
    def get_new_object_covariance(self, x, y, theta, px, py, sigma_bar):

        dx = px - x
        dy = py - y
        q = dx**2 + dy**2


    
        
        z_bar = np.array([math.sqrt(q), math.atan2(dy,dx) - theta])
        aux = np.array([[-math.sqrt(q)*dx, -math.sqrt(q)*dy, 0, math.sqrt(q)*dx, math.sqrt(q)*dy], 
                        [dy, -dx, -q, -dy, dx]])
        H = 1/q * aux 
        Q = self.get_noise_matrix(z_bar)
        Psi = H @ sigma_bar @ np.transpose(H) + Q

        K = sigma_bar @ np.transpose(H) @ inv(Psi)
        
        Sigma = (np.identity(5) - K @ H) @ sigma_bar
        return Sigma

x0 = 1.10652899742
y0 = 0.000232384933042
tt0 = 3.1174481532182856

minimap_length = 2
obj_EKF = EKF([x0, y0, tt0])
obj_EKF.set_square((0,0), minimap_length)
EKF_list = {(0,0): obj_EKF}


# Load the map image


# Define LiDAR sensor position
lidar_position = [200, 150, 0]  # Example coordinates, replace with actual position

freq_counter = 0
intersection_points = []
distances = []
#phis = []

u = [0, 0] # Initial velocities

du = 0.03 # Time step

# Define parameters
num_rays = 360  # Number of rays to cast
max_range = 70  # Maximum range of LiDAR sensor
error_percentage = 5  # Error percentage

# Initialize list to store LiDAR positions
robot_positions = []

ekf_positions=[]

# Initialize OpenCV window
cv2.namedWindow('Map and LiDAR Readings')



def calculate_triangle_points(lidar_position):
    
    theta = lidar_position[2]
    # Define the length of the triangle sides
    side_length = 3
    
    # Define the angles for the three sides of the triangle
    angles = [theta - np.pi / 2, theta, theta + np.pi / 2]
    
    # Calculate the coordinates of the triangle vertices
    triangle_points = np.array([
        [round(lidar_position[0] + side_length * np.cos(angle)), round(lidar_position[1] + side_length * np.sin(angle))]
        for angle in angles
    ])
    
    return triangle_points


# Define the size of the display
width, height = 1000, 1000
background_color = (255, 255, 255)  # white background
global image

# Create a blank image
image = cv2.imread('map_gmapping.png')
#image = np.ones((height, width, 3), dtype=np.uint8) * 255

# Calculate the center of the image
global origin
#origin = (500,500)
origin = (570,212)

global count_landmarks
count_landmarks =0


# Function to update LiDAR readings
def update_lidar(obj_ekf, EKF_list):
    global robot_position
    global ekf_position
    global lidar_range
    global curr_vel
    global distances
    global landmark_positions
    global time_diff
    global image
    global origin
    global count
    global time_diff
    
    global count_landmarks
    
    distances = []
    #scale=30
    scale = 40

    
    for i_aux,distance in enumerate(lidar_range):
        distances.append(np.array([distance, i_aux *(2*math.pi / 360)]))

    
    odom_predict = np.array(robot_position)
    odom_predict = odom_predict.reshape(-1,1)

    ekf_vector= obj_ekf.new_state(curr_vel, distances, odom_predict, time_diff)
    ekf_position = ekf_vector[:3]

    landmark_positions_aux = []
    for key in EKF_list:
        landmark_positions_aux.extend(EKF_list[key].mu[3:])
    
    landmark_positions_aux = ekf_vector[(3+count_landmarks*2):]
    landmark_positions = [landmark_positions_aux[i:i+2] for i in range(0, len(landmark_positions_aux), 2)]
    count_landmarks += len(landmark_positions)

    # Draw red points for every position the sensor has been
    for pos in robot_positions:
        x = round(origin[0] + pos[0]*scale)
        y = round(origin[1] - pos[1]*scale)
        cv2.circle(image, (x, y), 1, (0, 0, 255), -1)

    # Draw red points for every position the sensor has been
    

        # Draw red points for every position the sensor has been
    for pos in ekf_positions:
        x = round(origin[0] + pos[0]*scale)
        y = round(origin[1] - pos[1]*scale)
        cv2.circle(image, (x, y), 1, (255, 0, 0), -1)

    for pos in landmark_positions:
        
        
        lx = round(origin[0] + pos[0]*scale )
        ly = round(origin[1] - pos[1]*scale )
        
        #with open('div_landmark_pos_1.txt', 'a') as file:
            #file.write(f"({lx},{ly})\n")
            
        cv2.circle(image, (lx, ly), 1, (0, 0, 255), -1)
    

    # Display the map and LiDAR readings side by side
    cv2.imshow('Map and LiDAR Readings', image)
    cv2.waitKey(1)
    #print(obj_ekf.n)


def draw_square (image, center_x,center_y, half_size):

    top_left = (center_x - half_size*40, center_y - half_size*40)
    bottom_right = (center_x + half_size*40, center_y + half_size*40)
    cv2.rectangle(image, top_left, bottom_right, (0,0,0), 1)
    

    

lidar_range=[]

# Main loop
global count
count = 0
global time_diff

obj_ekf = EKF_list[(0,0)]
square_init_x= round(origin[0] + 1.10652899742*40)
square_init_y= round(origin[1] - 0.000232384933042*40 )

prev_aux = (0,0)
draw_square(image, square_init_x, square_init_y, 1)
pos_square_x =square_init_x
pos_square_y=square_init_y
while True:
    for index, (robot_position,lidar_range, curr_vel ) in enumerate(zip(xy_positions, ranges, velocity)):
        count += 1
        if index == 0:
            continue
        
        else:
            time_diff=abs(timestamps[index]/ 1e9 - timestamps[index-1]/ 1e9)

        robot_positions.append(robot_position[:])
 
        update_lidar(obj_ekf, EKF_list)
        #with open('div_a=100000.txt', 'a') as file:
        # Write the new values to the file
            #file.write(f"({ekf_position[0]},{ekf_position[1]}) ")
        ekf_positions.append(ekf_position[:])
        aux1 = math.floor((-(ekf_position[0] - 1.10652899742) + minimap_length/2) / minimap_length)
        aux2 = math.floor((ekf_position[1] - 0.000232384933042 + minimap_length/2) / minimap_length)

        aux = (aux1, aux2)
        

        if aux not in EKF_list:
            
            if aux[0] > prev_aux[0]:
                pos_square_x-=minimap_length*40
            elif aux[0] > prev_aux[0]:
                pos_square_x+=minimap_length*40

            if aux[1] > prev_aux[1]:
                pos_square_y-=minimap_length*40
            elif aux[1] > prev_aux[1]:
                pos_square_y+=minimap_length*40

            
            
            draw_square(image, pos_square_x, pos_square_y, (int)(minimap_length/2))

            
            
            new_ekf = EKF([ekf_position[0], ekf_position[1], ekf_position[2]])
            new_ekf.set_square(aux, minimap_length)
            EKF_list.update({aux: new_ekf})
            obj_ekf = new_ekf
            count_landmarks=0
            prev_aux = aux
            print(robot_position)
        else:
            obj_ekf = EKF_list[aux]

        if aux != prev_aux:
            
            obj_ekf.mu[:3] = ekf_position
            obj_ekf.sigma[:3][:3] = [[0,0,0], [0, 0, 0], [0,0,0]]

    end_time = time.time()
    break

print(f"Runtime: {end_time - start_time} seconds")       
print("COUNT")
print(count)    

print("COUNT")
print(count)


cv2.waitKey(0)
# Close OpenCV windows
cv2.destroyAllWindows()