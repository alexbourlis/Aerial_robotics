import numpy as np

on_ground = True
height_desired = 0.5
setpoints = [[-0.0, -0.0], [0.0, -2.0], [2.0, -2.0], [2.0,  -4.0],[-1.0, -2.0],[-1.0, -3.0]]
delta = 0.3
setpoints = [[3.5+delta,delta],[3.5+delta, 3-delta], [3.5+3*delta, 3-delta], [3.5+3*delta, delta]]
index_current_setpoint = 0
print_flag = True
take_off_counter = 0
obstacle_info = [0,0]

def path_planning(sensor_data):
    
    global on_ground, height_desired, index_current_setpoint, setpoints, take_off_counter
    #TAKE OFF
    if on_ground and sensor_data['range_down'] < 0.49:
    	return take_off(sensor_data)
    else:
        on_ground = False
    # Hover at the final setpoint
    if index_current_setpoint == len(setpoints):
        control_command = [0.0, 0.0, 0.0, height_desired]
        return control_command

    # Get the drone->goal distance
    distance_drone_to_goal = get_dist_drone_goal(sensor_data)
    # When the drone reaches the goal setpoint, we oriente it towards the next goal
    if distance_drone_to_goal < 0.1:
        return adjust_drone_orientation(sensor_data)

    #advance to goal taking into account obstacles
    control_command = get_command_to_goal(sensor_data)
    control_command = obstacle_avoidance(sensor_data,control_command)
    return control_command

def take_off(sensor_data):
	# Take off
	global on_ground, height_desired, index_current_setpoint, setpoints, take_off_counter
	seuil = 0.02
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

def get_command_to_goal(sensor_data):
	global on_ground, height_desired, index_current_setpoint, setpoints
	# Calculate the control command based on current goal setpoint
	theta = sensor_data['yaw']
	M = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]]) 	#MB2I
	v_goal = np.array(setpoints[index_current_setpoint])							#goal position vector 
	v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
	dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector v
	dir_goal = v_goal-v_drone 														#drone->goal vector       u
	v_x, v_y = np.clip(np.linalg.inv(M).dot(dir_goal),-0.5,0.5)						#velocity expressed in the body frame
	angle = vector_orientation(dir_goal,dir_drone)									#how well the drone is alligned with the target
	omega = -2*np.sign(angle)*abs(angle)**0.5										#square root the angle to have a faster speed for small angles(better reactivity)
	control_command = [v_x, v_y, omega, height_desired]
	if print_flag == True: print("distance drone goal (2):", np.linalg.norm(dir_goal))
	return control_command

def adjust_drone_orientation(sensor_data):
    global height_desired, index_current_setpoint, setpoints
    seuil = 0.02
    # adjust the orientation of the drone before going to the next goal
    try:
        v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
        v_next_goal = np.array(setpoints[index_current_setpoint+1])                     #next goal position vector
        dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector v
        dir_next_goal = v_next_goal-v_drone                                             #drone->nextGoal vector   u'
        angle = vector_orientation(dir_next_goal,dir_drone)
    except IndexError:
        angle = 0
    if abs(angle) >= seuil:
    	omega = -np.sign(angle)*2 if abs(angle)>0.4 else -np.sign(angle)*(abs(angle)**0.25)
    	control_command = [0.0, 0.0, omega, height_desired]
    	return control_command

    if print_flag == True: print("alligned")
    index_current_setpoint += 1										# Select the next setpoint as the goal position
    control_command = [0.0, 0.0, 0.0, height_desired]
    return control_command


def get_dist_drone_goal(sensor_data):
	global index_current_setpoint, setpoints
	v_goal = np.array(setpoints[index_current_setpoint])                            #goal position vector            
	v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
	dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector v
	dir_goal = v_goal-v_drone                                                       #drone->goal vector       u
	return np.linalg.norm(dir_goal)

def vector_orientation(u,v):
        u = np.array(u).reshape((1,2))
        v = np.array(v).reshape((1,2))
        det_uv = np.linalg.det(np.concatenate([u,v],axis=0))
        scal_prod = u.dot(v.T)

        if scal_prod < 0 and det_uv == 0:
            angle = np.pi
        else:
            angle = np.sign(det_uv)*np.arccos(scal_prod/np.linalg.norm(u)/np.linalg.norm(v))

        return np.squeeze(angle).tolist()

# Obstacle avoidance
def obstacle_avoidance(sensor_data,control_command):
        # Obstacle avoidance with distance sensors
        obst_type = {"No_obstacle": 0,"front_obstacle": 1,"left_obstacle": 2,"right_obstacle": 3}
        #Direction to follow when getting around an obstacle 
        around_dir = {"go_straight": 0,"go_left": 1,"go_right": 2}
        O_TYPE,A_DIR = 0,1 #indices for the obstacle_info array,O_TYPE = obstacle type index | A_DIR = get around direction index

        if sensor_data['range_front'] < 0.3:
            obstacle_info[O_TYPE] = obst_type['front_obstacle']
            control_command[0] = 0.0
            if (sensor_data['range_left'] > sensor_data['range_right'] and obstacle_info[A_DIR] == around_dir['go_straight']
                ) or obstacle_info[A_DIR] == around_dir['go_left']:
                obstacle_info[A_DIR] = around_dir['go_left']
                control_command[1] = 0.7

            elif obstacle_info[A_DIR] == around_dir['go_straight'] or obstacle_info[A_DIR] == around_dir['go_right']:
                obstacle_info[A_DIR] = around_dir['go_right']
                control_command[1] = -0.7

        elif sensor_data['range_right'] < 0.3:# and sensor_data['v_left'] < -0:
            obstacle_info[O_TYPE] = obst_type['right_obstacle']
            control_command[1] = min(0.01/(sensor_data['range_right'])**2,0.5) if sensor_data['range_right'] < 0.2 else 0

        elif sensor_data['range_left'] < 0.3:# and sensor_data['v_left'] > 0: 
            obstacle_info[O_TYPE] = obst_type['left_obstacle']
            control_command[1] = -0.2 if sensor_data['range_left'] < 0.14 else 0
        else:
            obstacle_info[O_TYPE] = obst_type['No_obstacle']
            obstacle_info[A_DIR] = around_dir['go_straight']

        return control_command

def new_obstacle_avoidance(sensor_data,control_command):
    obst_type = {"No_obstacle": 0,"front_obstacle": 1,"left_obstacle": 2,"right_obstacle": 3}
    around_dir = {"go_straight": 0,"go_left": 1,"go_right": 2}  #Direction to follow when getting around an obstacle
    O_TYPE,A_DIR = 0,1  #indices for the obstacle_info array,O_TYPE = obstacle type index | A_DIR = get around direction index
    OBS_THRESHOLD = 0.3
    MARGIN = 40+10+5
    left_dist, right_dist = get_dist_edges(sensor_data)
    left_range, right_range = sensor_data['range_left'],sensor_data['range_right']
    if sensor_data['range_front'] < OBS_THRESHOLD or obstacle_info[O_TYPE] == obst_type['front_obstacle']:
        if (left_dist > MARGIN and left_range > MARGIN) or obstacle_info[A_DIR] == around_dir['go_left']:
            obstacle_info[A_DIR] = around_dir['go_left']
            #contourner à gauche
        if (right_dist > MARGIN and right_range > MARGIN) or obstacle_info[A_DIR] == around_dir['go_right']:
            obstacle_info[A_DIR] = around_dir['go_right']
            #contourner à droite
        else:
            print("ne pas bouger")
            # ne pas bouger

def get_dist_edges(sensor_data):
    pos_x,pos_y = sensor_data['x_global'],sensor_data['y_global']
    yaw_angle = sensor_data['yaw']
    pi = np.pi
    phi = pi/8

    if yaw_angle > -phi or yaw_angle <= phi:                    #Nord
        left_dist = 3-pos_y
        right_dist = pos_y
    elif yaw_angle > -pi/2+phi or yaw_angle <= -phi:            #Nord-Est
        left_dist = min(3-pos_y,5-pos_x)
        right_dist = min(pos_y,pos_x)
    elif yaw_angle > -pi/2-phi or yaw_angle <= -pi/2+phi:       #Est
        left_dist = 5-pos_x
        right_dist = pos_x
    elif yaw_angle > phi or yaw_angle <= pi/2-phi:              #Nord-Ouest
        left_dist = min(3-pos_y,pos_x)
        right_dist = min(pos_y,5-pos_x)
    elif yaw_angle > pi/2-phi or yaw_angle <= pi/2+phi:         #Ouest
        left_dist = pos_x
        right_dist = 5-pos_x
    elif yaw_angle > pi/2+phi or yaw_angle <= pi-phi:           #Sud-Ouest
        left_dist = min(pos_x,pos_y)
        right_dist = min(5-pos_x,3-pos_y)
    elif yaw_angle > -pi+phi or yaw_angle <= -pi/2-phi:         #Sud-Est
        left_dist = min(5-pos_x,pos_y)
        right_dist = min(pos_x,3-pos_y)
    else:                                                       #Sud
        left_dist = pos_y
        right_dist = 3-pos_y
    return left_dist, right_dist