import numpy as np
import matplotlib.pyplot as plt
from work_file import vector_orientation
from numpy import pi

on_ground = True
height_desired = 0.5
setpoints = [[-0.0, -0.0], [0.0, -2.0], [2.0, -2.0], [2.0,  -4.0],[-1.0, -2.0],[-1.0, -3.0]]
delta = 0.3
setpoints = [[3.5+delta,delta],[3.5+delta, 3-delta], [3.5+3*delta, 3-delta], [3.5+3*delta, delta]]
index_current_setpoint = 0
print_flag = True
take_off_counter = 0
obstacle_info = [0,0]
scanning_state = 0    #0 is the scanning start condition


def path_planning(sensor_data):
    
    global on_ground, height_desired, index_current_setpoint, setpoints, take_off_counter
    #TAKE OFF
    if on_ground and sensor_data['range_down'] < 0.49:
    	return take_off(sensor_data)
    else:
        on_ground = False
        angle = scanning(pi/4,drone_goal_orientation(sensor_data))
        #return hover(omega_func2(angle))
        return command_to_goal(sensor_data,omega_func2(angle))
	
def take_off(sensor_data):
    # Take off
    global on_ground, height_desired, index_current_setpoint, setpoints, take_off_counter
    seuil = 0.02
    if take_off_counter > 2:
        #angle = drone_goal_orientation(sensor_data)
        #if abs(angle) >= seuil: omega = np.sign(-angle)#-np.sign(angle)
        #else: omega = 0
        omega = omega_func2(scanning(pi/4,sensor_data['yaw']))
    else:
        omega = 0
    take_off_counter += 1
    return hover(omega)

# Calculate the control command based on current goal setpoint
def command_to_goal(sensor_data,omega):
    global height_desired, index_current_setpoint, setpoints
    
    theta = sensor_data['yaw']
    M = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])    #MB2I
    v_goal = np.array(setpoints[index_current_setpoint])                            #goal position vector 
    v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
    dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector v
    dir_goal = v_goal-v_drone                                                       #drone->goal vector       u
    v_x, v_y = np.linalg.inv(M).dot(dir_goal)                                       #velocity expressed in the body frame
    #preserving proportionality even when the values are clipped
    if v_x > v_y:                                                                           
        ratio = v_y/v_x
        v_x = np.clip(v_x,-0.5,0.5)
        v_y = v_x*ratio
    elif v_y >= v_x and v_y != 0:
        ratio = v_x/v_y
        v_y = np.clip(v_y,-0.5,0.5)
        v_x = ratio*v_y

    #alligne drone with drone->goal direction
    #angle = vector_orientation(dir_goal,dir_drone)                                  #how well the drone is alligned with the target
    #omega = -2*np.sign(angle)*abs(angle)**0.5                                       #square root the angle to have a faster speed for small angles(better reactivity)
    control_command = [v_x, v_y, omega, height_desired]
    return control_command

def hover(omega):
    return [0.0, 0.0, omega, height_desired]

def scanning(theta,alpha):
	global scanning_state
	seuil = 0.02
	scan_states = {'start':0,'goal_+':1,'goal_-':2,'goal_0':3,'end':4}
	phi = theta
	if scanning_state == scan_states['goal_-']: phi = -theta
	if scanning_state == scan_states['goal_0']: phi = 0

	if scanning_state == scan_states['start']:
		if abs(theta-alpha) < seuil:
			scanning_state = scan_states['goal_-']
			phi = -theta
		else:
			scanning_state = scan_states['goal_+']
	elif scanning_state == scan_states['end']:
		phi = 0
		scanning_state = scan_states['start']
	else:
		if abs(phi-alpha) < seuil:
			scanning_state +=1
	print("scanning_state: ",scanning_state," | phi:",phi,"phi-alpha:",phi-alpha)
	return phi-alpha

def drone_goal_orientation(sensor_data):#positive angle means dir drone is left from dir goal
    global index_current_setpoint, setpoints
    v_goal = np.array(setpoints[index_current_setpoint])                            #goal position vector
    v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
    dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector
    dir_goal = v_goal-v_drone                                                       #drone->goal vector

    angle = vector_orientation(dir_goal,dir_drone)
    return angle

def omega_func(angle):
	return np.sign(angle)*2 if abs(angle)>0.4 else np.sign(angle)*(abs(angle)**0.25)

def omega_func2(angle):
    return 2*np.sign(angle)*abs(angle)**0.5

min_x, max_x = 0, 5.0 # meter
min_y, max_y = 0, 3.0 # meter
range_max = 2.0 # meter, maximum range of distance sensor
res_pos = 0.1 # meter (step)
conf = 0.5 # certainty given by each measurement
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
    if t % 25 == 0:
        plt.imshow(np.flip(map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
        #plt.imshow(map, vmin=-1, vmax=1, cmap='gray', origin='lower')
        #plt.pause(0.01) #added
        plt.savefig("map_scan.png")
    t +=1

    return map