import numpy as np
from numpy import pi
from numpy.linalg import norm

on_ground = True
height_desired = 0.5
setpoints = [[-0.0, -0.0], [0.0, -2.0], [2.0, -2.0], [2.0,  -4.0],[-1.0, -2.0],[-1.0, -3.0]]
delta = 0.3
setpoints = [[3.5+delta,delta],[3.5+delta, 3-delta], [3.5+2*delta, 3-delta], [3.5+2*delta, delta],[3.5+3*delta,delta],[3.5+3*delta, 3-delta],
[3.5+4*delta, 3-delta], [3.5+4*delta, delta],[3.5+5*delta,delta],[3.5+5*delta, 3-delta]]
index_current_setpoint = 0
print_flag = True
take_off_counter = 0
obstacle_info = [0,0]
past_height = 0
land_drone = False
start_pos = [-1.0,-1.0]
scanning_state = 0    #0 is the scanning start condition
propagation_counter = 0
#testing
period = 0

obstacle_flag = False
around = False

machin = 0 #another counter

def path_planning(sensor_data):
    
    global on_ground, height_desired, index_current_setpoint, setpoints, take_off_counter,land_drone,past_height,around
    #TAKE OFF
    if on_ground and sensor_data['range_down'] < 0.49:
    	return take_off(sensor_data)
    else:
        on_ground = False

    #land the drone
    if (past_height-sensor_data['range_down']>0.05 or land_drone) and around == False:
        print("land_drone around:", around)
        land_drone = True
        past_height = sensor_data['range_down']
        return landing_drone(sensor_data)
 
    past_height = sensor_data['range_down']    
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
    #control_command = obstacle_avoidance(sensor_data,control_command)
    #control_command = old_obstacle_avoidance(sensor_data,control_command)
    obstacle_scanning(sensor_data)
    return control_command

def landing_drone(sensor_data):
    global height_desired, land_drone, on_ground, index_current_setpoint, setpoints, start_pos
    #if self.get_distance_to_goal(sensor_data)>0.07: 
    #   return self.path_planning(sensor_data)
    height_desired -= 0.005
    control_command = [0.0, 0.0, 0.0, height_desired]
    if sensor_data['range_down']<0.015:
    		land_drone = False
    		on_ground = True
    		index_current_setpoint = 0
    		setpoints[index_current_setpoint] = start_pos
    		height_desired = 0.5
    		if np.linalg.norm(np.array(start_pos)-np.array([sensor_data['x_global'], sensor_data['y_global']])) < 0.15:
    			height_desired = 0.009
    			index_current_setpoint = len(setpoints)
    			on_ground = False
    #on_ground = False
    #past_height = sensor_data['range_down']
    return control_command

def take_off(sensor_data):
	# Take off
	global on_ground, height_desired, index_current_setpoint, setpoints, take_off_counter, start_pos
	seuil = 0.02
	if take_off_counter > 2:
	    if start_pos == [-1.0,-1.0]: start_pos = [sensor_data['x_global'], sensor_data['y_global']]
	    past_height = sensor_data['range_down']
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
	v_x, v_y = np.linalg.inv(M).dot(dir_goal)										#velocity expressed in the body frame
	if v_x > v_y:                                                                           
	    ratio = v_y/v_x
	    v_x = np.clip(v_x,-0.5,0.5)
	    v_y = v_x*ratio
	elif v_y >= v_x and v_y != 0:
	    ratio = v_x/v_y
	    v_y = np.clip(v_y,-0.5,0.5)
	    v_x = ratio*v_y

	angle = vector_orientation(dir_goal,dir_drone)									#how well the drone is alligned with the target
	angle = scanning(pi/30,angle)
	omega = 2*np.sign(angle)*abs(angle)**0.5		#i put a minus without scanning #square root the angle to have a faster speed for small angles(better reactivity)
	control_command = [v_x, v_y, omega, height_desired]
	if print_flag == True: print("distance drone goal (2):", np.linalg.norm(dir_goal))
	return control_command

furthest_point = 0

def obstacle_scanning(sensor_data):
    global machin, index_current_setpoint, setpoints,furthest_point

    front_dist = sensor_data['range_front']
    left_dist = sensor_data ['range_left']
    yaw = sensor_data['yaw']
    v_goal = np.array(setpoints[index_current_setpoint])
    print("machin: ", machin)
    if machin == 21: 
        machin = 0
        print("furthest_point after a scanning period: ", furthest_point)
    if machin > 0: machin+=1
    if front_dist< 1:
        if machin == 0: machin+=1
        v_drone = np.array([sensor_data ['x_global'],sensor_data['y_global']])
        v_obst = front_dist*np.array([np.cos(yaw),np.sin(yaw)])
        vec_path = v_goal-v_drone
        v_dist = v_obst-(v_obst.dot(vec_path)/vec_path.dot(vec_path))*vec_path
        angle = vector_orientation(vec_path,v_obst)
        if norm(v_dist)>abs(furthest_point): furthest_point = np.sign(angle)*norm(v_dist)
        print("distance: ", np.sign(angle)*norm(v_dist),"furthest_point: ", furthest_point)

def dist_point_to_path(pro_point,point):
    point = np.array(point)
    global index_current_setpoint, setpoints

    v_goal = np.array(setoints[index_current_setpoint])                            #goal position vector
    v_pro_point = np.array(pro_point)                           #propagation point position vector
    v_point = np.array(point)
    vec_path = v_goal-v_pro_point                                                      #drone->goal vector
    dir_point = v_point-v_pro_point
    v_dist = dir_point-(dir_point.dot(vec_path)/vec_path.dot(vec_path))*vec_path
    return v_dist

def scanning(theta,alpha):
    global scanning_state,period
    seuil = 0.02
    scan_states = {'start':0,'goal_+':1,'goal_-':2,'end':3} #'goal_0':3,
    phi = theta
    if scanning_state == scan_states['goal_-']: phi = -theta
    #if scanning_state == scan_states['goal_0']: phi = 0
    if scanning_state == scan_states['start']:
        period = 0
        if abs(theta-alpha) < seuil:
        	scanning_state = scan_states['goal_-']
        	phi = -theta
        else:
        	scanning_state = scan_states['goal_+']
    elif scanning_state == scan_states['end']:
        print("periode:", period)
        phi = 0
        scanning_state = scan_states['start']
    else:
        if abs(phi-alpha) < seuil:
        	scanning_state +=1
    #print("scanning_state: ",scanning_state," | phi:",phi," | phi-alpha:",phi-alpha)
    period +=1
    return phi-alpha

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
    	#omega = -np.sign(angle)*2 if abs(angle)>0.4 else -np.sign(angle)*(abs(angle)**0.25)
    	omega = -2*np.sign(angle)*abs(angle)**0.5
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


def old_obstacle_avoidance(sensor_data,control_command):
    global height_desired, obstacle_flag,propagation_counter, around 

    dt = 0.032
    obstacle_flag = False
    front_dist = sensor_data['range_front']
    left_dist = sensor_data	['range_left']
    speed_front = max(0.1,sensor_data['v_forward'])

    cycles = 20
    dist = 0.2
    speed = 2*dist/cycles/dt
    acc = speed/cycles/dt
    #speed = 0.5
    #acc = speed**2/2/dist	

    if front_dist < 0.8 or propagation_counter > 0:
        around = True
        if propagation_counter == 0: propagation_counter = 1000
        #v_x = np.sign(front_dist-0.2)*(abs(speed_front*(front_dist-0.2)))**0.5
        cycles_prime = 75
        dist_prime = 0.6
        speed_prime = 2*dist_prime/cycles_prime/dt  #speed of 0.5
        acc_prime = speed_prime/cycles_prime/dt
        N = 1000-propagation_counter
        v_x = max(0,(speed_prime-acc_prime*N*dt))
        v_y = 0 if propagation_counter > 200 else -speed
        omega = 0
        if front_dist < 0.3 and propagation_counter>200:#around == False:
            propagation_counter = 1000  
            v_y = -speed
            v_x = 0

        if front_dist > 0.8 or propagation_counter <= 200:#around == True:
            if propagation_counter>200: propagation_counter = 200
            #around = True
            if left_dist < 0.5: propagation_counter	= 100
            #v_y = -speed
            #v_x = 0 
            if propagation_counter > 200-cycles:
                N = 200-propagation_counter
                v_x = 0
                v_y = -(speed-acc*N*dt)
            elif propagation_counter > 100-cycles:
                N = 100-propagation_counter
                v_x = min(speed,(speed-acc*N*dt))
                v_y = 0#(left_dist-0.2) if left_dist < 0.3 else 0
            else:
                N = 60-propagation_counter
                if propagation_counter == 40: propagation_counter = 1
                v_x = 0
                v_y = min(speed,(speed-acc*N*dt))

        propagation_counter -= 1

        print("propagation counter:", propagation_counter)
        #obstacle_flag = True
        control_command = [v_x,v_y,omega,height_desired]

    if propagation_counter == 0: around = False
    print("around:", around)

    return control_command

def speed_func1(x,centre):
	speed = 0.5-(0.4/centre**2)*((x-centre)**2)
	return speed
def speed_func2(x,centre):
	speed = 0.5-(0.5/centre**2)*((x-centre)**2)
	return speed
def omega_func(angle):
    return np.sign(angle)*2 if abs(angle)>0.4 else np.sign(angle)*(abs(angle)**0.25)

def omega_func2(angle):
    return 2*np.sign(angle)*abs(angle)**0.5

# Obstacle avoidance with distance sensors
#if sensor_data['range_front'] < 0.4:
#    if sensor_data['range_left'] > 1:
#        control_command[1] = 0.5
#    else:
#        control_command[1] = -0.5
#    control_command[0] = control_command[0]/2
#
#if sensor_data['range_left'] < 0.3:
#   control_command[1] = -0.5
#if sensor_data['range_right'] < 0.3:
#   control_command[1] = 0.5
#

def obstacle_avoidance(sensor_data,control_command):
        # Obstacle avoidance with distance sensors
        obst_type = {"No_obstacle": 0,"front_obstacle": 1,"left_obstacle": 2,"right_obstacle": 3}
        #Direction to follow when getting around an obstacle 
        around_dir = {"go_straight": 0,"go_left": 1,"go_right": 2}
        O_TYPE,A_DIR = 0,1 #indices for the obstacle_info array,O_TYPE = obstacle type index | A_DIR = get around direction index

        if sensor_data['range_front'] < 0.3:
            obstacle_info[O_TYPE] = obst_type['front_obstacle']
            control_command[0] = -0.7 if sensor_data['range_front']<0.2 else 0.0
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
    OBS_THRESHOLD = 0.5
    MARGIN = (40+10+5)/100
    left_dist, right_dist = get_dist_edges(sensor_data)
    left_range, right_range = sensor_data['range_left'],sensor_data['range_right']
    print("here")
    if sensor_data['range_front'] < OBS_THRESHOLD or obstacle_info[O_TYPE] == obst_type['front_obstacle']:
        print("ici: left dist et left range:", left_dist, left_range)
        if (left_dist > MARGIN and left_range > MARGIN) or obstacle_info[A_DIR] == around_dir['go_left']:
            print("coucou")
            obstacle_info[A_DIR] = around_dir['go_left']
            control_command[1] = 0.7
            return control_command
        if (right_dist > MARGIN and right_range > MARGIN) or obstacle_info[A_DIR] == around_dir['go_right']:
            obstacle_info[A_DIR] = around_dir['go_right']
            control_command[1] = -0.7
            return control_command
        else:
            print("ne pas bouger")
            control_command[1] = 0

    return control_command


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