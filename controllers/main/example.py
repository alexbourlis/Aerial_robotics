# Examples of basic methods for simulation competition
import numpy as np
import matplotlib.pyplot as plt
import math

# Global variables
on_ground = True
height_desired = 0.5

# Obstacle avoidance with range sensors
def obstacle_avoidance(sensor_data):
    global on_ground, height_desired

    # Take off
    if on_ground and sensor_data['range_down'] < 0.49:
        control_command = [0.0, 0.0, 0.0, height_desired]
        return control_command
    else:
        on_ground = False

    # Obstacle avoidance with distance sensors
    if sensor_data['range_front'] < 0.2:
        if sensor_data['range_left'] > sensor_data['range_right']:
            control_command = [0.0, 0.2, 0.0, height_desired]
        else:
            control_command = [0.0, -0.2, 0.0, height_desired]
    else:
        control_command = [0.2, 0.0, 0.0, height_desired] #speed was 0.2 originaly

    return control_command

# Coverage path planning
setpoints = [[-0.0, -0.0], [0.0, -2.0], [2.0, -2.0], [2.0,  -4.0],[-1.0, -2.0],[-1.0, -3.0]]
delta = 0.3
setpoints = [[3.5+delta,delta],[3.5+delta, 3-delta], [3.5+3*delta, 3-delta], [3.5+3*delta, delta]]
index_current_setpoint = 0
print_flag = True
take_off_counter = 0

def path_planning(sensor_data):
    def vector_orientation(u,v):
        seuil = 0.02
        u = np.array(u).reshape((1,2))
        v = np.array(v).reshape((1,2))
        det_uv = np.linalg.det(np.concatenate([u,v],axis=0))
        scal_prod = u.dot(v.T)

        if scal_prod < 0 and det_uv == 0:
            angle = np.pi
        else:
            angle = np.sign(det_uv)*np.arccos(scal_prod/np.linalg.norm(u)/np.linalg.norm(v))

        if print_flag == True: print("vector orientation and angle_uv = ",angle)

        return np.squeeze(angle).tolist()

    global on_ground, height_desired, index_current_setpoint, setpoints, take_off_counter
    seuil = 0.02
    # Take off
    if on_ground and sensor_data['range_down'] < 0.49:
        if take_off_counter > 2:
            v_goal = np.array(setpoints[index_current_setpoint])                            #goal position vector
            v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
            dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector
            dir_goal = v_goal-v_drone                                                       #drone->goal vector
            angle = vector_orientation(dir_goal,dir_drone)
            if abs(angle) >= seuil: omega = -np.sign(angle)
            else: omega = 0
        else:
            omega = 0
        take_off_counter += 1
        control_command = [0.0, 0.0, omega, height_desired]
        return control_command
    else:
        on_ground = False

    # Hover at the final setpoint
    if index_current_setpoint == len(setpoints):
        control_command = [0.0, 0.0, 0.0, height_desired]
        return control_command

    # Get the goal position and drone position
    v_goal = np.array(setpoints[index_current_setpoint])                            #goal position vector            
    v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
    dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector
    dir_goal = v_goal-v_drone                                                       #drone->goal vector
    distance_drone_to_goal = np.linalg.norm(dir_goal)

    try:
        v_next_goal = np.array(setpoints[index_current_setpoint+1])                     #next goal position vector
        dir_next_goal = v_next_goal-v_drone                                             #drone->nextGoal vector
        angle = vector_orientation(dir_next_goal,dir_drone)
        if print_flag == True: print("drone orientation:", dir_drone," | drone->nextGoal vec:",dir_next_goal," | current goal: ", v_goal ," | next goal:", v_next_goal)
    except IndexError:
        angle = 0
   
    
    # When the drone reaches the goal setpoint, e.g., distance < 0.1m
    if distance_drone_to_goal < 0.1:
        # Select the next setpoint as the goal position
       
        if abs(angle) >= seuil:
            
            omega = -np.sign(angle)*2 if abs(angle)>0.4 else -np.sign(angle)*(abs(angle)**0.25)
            control_command = [0.0, 0.0, omega, height_desired]
            if print_flag == True: print("distance drone goal:", distance_drone_to_goal) 
            return control_command
        
        if print_flag == True: print("alligned")
        index_current_setpoint += 1
        # Hover at the final setpoint
        if index_current_setpoint == len(setpoints):
            control_command = [0.0, 0.0, 0.0, height_desired]
            return control_command

    # Calculate the control command based on current goal setpoint
    theta = sensor_data['yaw']
    M = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]]) #MB2I
    v_goal = np.array(setpoints[index_current_setpoint])
    v_x, v_y = np.clip(np.linalg.inv(M).dot(v_goal-v_drone),-0.3,0.3)
    control_command = [v_x, v_y, 0.0, height_desired]
    if print_flag == True: print("distance drone goal (2):", np.linalg.norm(v_goal-v_drone)) 
    return control_command
        
    
# Occupancy map based on distance sensor
min_x, max_x = 0, 5.0 # meter
min_y, max_y = 0, 5.0 # meter
range_max = 2.0 # meter, maximum range of distance sensor
res_pos = 0.2 # meter
conf = 0.2 # certainty given by each measurement
t = 0 # only for plotting

map = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))) # 0 = unknown, 1 = free, -1 = occupied

def occupancy_map(sensor_data):
    global map, t
    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']
    yaw = sensor_data['yaw']
    
    for j in range(4): # 4 sensors
        yaw_sensor = yaw + j*np.pi/2 #yaw positive is counter clockwise
        if j == 0:
            measurement = sensor_data['range_front']
        elif j == 1:
            measurement = sensor_data['range_left']
        elif j == 2:
            measurement = sensor_data['range_back']
        elif j == 3:
            measurement = sensor_data['range_right']
        
        for i in range(int(range_max/res_pos)): # range is 2 meters
            dist = i*res_pos
            idx_x = int(np.round((pos_x - min_x + dist*np.cos(yaw_sensor))/res_pos,0))
            idx_y = int(np.round((pos_y - min_y + dist*np.sin(yaw_sensor))/res_pos,0))

            # make sure the point is within the map
            if idx_x < 0 or idx_x >= map.shape[0] or idx_y < 0 or idx_y >= map.shape[1] or dist > range_max:
                break

            # update the map
            if dist < measurement:
                map[idx_x, idx_y] += conf
            else:
                map[idx_x, idx_y] -= conf
                break
    
    map = np.clip(map, -1, 1) # certainty can never be more than 100%

    # only plot every Nth time step (comment out if not needed)
    if t % 50 == 0:
        plt.imshow(np.flip(map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
        plt.pause(0.01) #added
        plt.savefig("map.png")
    t +=1

    return map