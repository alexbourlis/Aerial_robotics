import numpy as np
from numpy.linalg import norm

# Global variables
on_ground = True
land_flag = False
height_desired = 0.5
past_height = 0
take_off_counter = 0
initial_pos = [0,0]
scanning_state = 0
delta = 0.3
setpoints = [[3.5+delta,delta],[3.5+delta, 3-delta], [3.5+2*delta, 3-delta], [3.5+2*delta, delta],[3.5+3*delta,delta]\
			,[3.5+3*delta, 3-delta],[3.5+4*delta, 3-delta], [3.5+4*delta, delta],[3.5+5*delta,delta],[3.5+5*delta, 3-delta]]
index_current_setpoint = 0
obstacle_counter = 0
DT = 0.032
N = 0
clearing_distance = 0
old_vec_per = None
last_front_range = 0
last_angle_goal_drone = 0
closest_point = np.inf
largest_front_range = 0
largest_front_range_angle = 0
direction = 0 #1 left,-1 right
first_point = [0,0]
computing_flag = False

# Obstacle avoidance with range sensors
def main(sensor_data):
    global on_ground, height_desired, obstacle_counter, DT, N, old_vec_per, last_front_range , N2\
           , last_angle_goal_drone, direction, closest_point, setpoints, index_current_setpoint\
           ,take_off_counter, initial_pos, past_height, land_flag, first_point, computing_flag
    side_speed = 0.2
    # Take off
    if on_ground and sensor_data['range_down'] < 0.49:
        last_front_range = sensor_data['range_front']
        _,last_angle_goal_drone = drone_goal_orientation(sensor_data,index_current_setpoint)
        omega = omega_func2(-last_angle_goal_drone)
        control_command = [0.0, 0.0, omega, height_desired]
        if take_off_counter == 2:
        	initial_pos = [sensor_data['x_global'],sensor_data['y_global']]
        if take_off_counter > 2:
        	past_height = sensor_data['range_down']
        take_off_counter += 1
        return control_command
    else:
        on_ground = False
    
    # Landing the Drone
    if (sensor_data['t']> 4 and (past_height-sensor_data['range_down']) > 0.05) or land_flag:
    	
    	land_flag = True
    	height_desired -= 0.005
    	if sensor_data['range_down']<0.014:
    		print("LANDED")
    		if setpoints[index_current_setpoint] != initial_pos:
    			land_flag = False
    			on_ground = True
    			index_current_setpoint = 0
    			setpoints[index_current_setpoint] = initial_pos
    			height_desired = 0.5

    	control_command = [0.0, 0.0, 0.0, height_desired]
    	return control_command


    #goals update
    if sensor_data['range_front'] < 2:
    	k = 0
    	vec_drone_obstacle = sensor_data['range_front']*np.array([np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])])
    	for i in range(len(setpoints)-index_current_setpoint):
    		vec_drone_goal,_ = drone_goal_orientation(sensor_data,index_current_setpoint+i-k)
    		dist = norm(vec_drone_goal - vec_drone_obstacle)
    		if dist < 0.5:
    			print("GOAL DELETED")
    			setpoints.pop(index_current_setpoint+i-k)
    			k+=1


    vec_goal_drone,angle_goal_drone = drone_goal_orientation(sensor_data,index_current_setpoint)
    desired_velocity = velocity_clipper(vec_goal_drone)
    velocity = progressiv_start(sensor_data,norm(desired_velocity))*desired_velocity/norm(desired_velocity)

    # obstacle_avoidance
    if obstacle_counter > 0: obstacle_counter -= 1
    if sensor_data['range_front'] < 0.7 or obstacle_counter>0:
    	if abs(last_front_range-sensor_data['range_front']) > 0.6 and direction == 0:
    		#print("last_angle_goal_drone: ", last_angle_goal_drone," and angle_goal_drone: ", angle_goal_drone)
    		direction = np.sign(last_angle_goal_drone-angle_goal_drone)*np.sign(last_front_range-sensor_data['range_front'])
    	
    	if obstacle_counter == 0: 
    		obstacle_counter = 1000
    		old_vec_per = unit_vec_per(sensor_data)
    		#velocity = B2I(sensor_data,[sensor_data['v_forward'],sensor_data['v_left']])
    		N = int((((sensor_data['range_front'])*np.cos(angle_goal_drone))-0.4)/norm(velocity)/DT)
    		print("N1: ", N)
    		closest_point = sensor_data['range_front']
    	if sensor_data['range_front'] < closest_point: closest_point = sensor_data['range_front']
    	velocity = desired_velocity*((abs(closest_point-0.2))**0.5)#*np.sign(sensor_data['range_front']-0.2)

    	if direction == 0:
    		omega = omega_func2(scanning(np.pi/16,angle_goal_drone))
    		if obstacle_counter == 1000-N:
    			direction = -np.sign(curve_orientation(sensor_data))
    	else:
    		omega = omega_func2(-angle_goal_drone)
    	print("we want to go in the direction: ", direction," and velocity: ", norm(velocity))
    	if obstacle_counter <= 1000-N:
    		omega = omega_func2(-angle_goal_drone)
    		if sensor_data['range_front'] > 0.5 or obstacle_counter <= 500:
    			if obstacle_counter > 500: 
    				clearing_distance = 0.1*norm(vec_goal_drone)/(norm(vec_goal_drone)-last_front_range)#normalement norm(vec_goal_drone)
    				N2 = int(clearing_distance/side_speed/DT)
    				if obstacle_counter == 1000-N:
    					N2 = 10
    				print("N2: ", N2," last_front_range: ", last_front_range, " norm: ", norm(vec_goal_drone),"clear:"\
    					, clearing_distance)
    				old_vec_per = unit_vec_per(sensor_data)
    				obstacle_counter = 500
    			velocity = progressiv_start(sensor_data,norm(desired_velocity))*desired_velocity/norm(desired_velocity)
    			print("we want to go in the direction: ", direction," and velocity: ", norm(velocity))
    			if obstacle_counter <= 500-N2:
    				if obstacle_counter <= 500-N2-8: 
    					obstacle_counter = 0
    					closest_point = np.inf
    					direction = 0
    				velocity -= direction*side_speed*old_vec_per
    		velocity += direction*side_speed*old_vec_per
    else:
    	# close to goal
    	distance_drone_to_goal = norm(vec_goal_drone)
    	if distance_drone_to_goal < 0.3:
    		print("HERE WE ARE")
    		if setpoints[index_current_setpoint]!=initial_pos:
    			_,angle_goal_drone = drone_goal_orientation(sensor_data,index_current_setpoint+1)
    			omega = omega_func2(-angle_goal_drone)
    		else:
    			omega = 0
    	else:
    		omega = omega_func2(scanning(np.pi/16,angle_goal_drone))
    	if distance_drone_to_goal < 0.1:
    		if setpoints[index_current_setpoint]!=initial_pos:
    			index_current_setpoint+=1


    print("obstacle_counter:", obstacle_counter)
    v_x_b,v_y_b = I2B(sensor_data,velocity)
    control_command = [v_x_b,v_y_b, omega, height_desired]

    last_front_range = sensor_data['range_front']
    last_angle_goal_drone = angle_goal_drone
    past_height = sensor_data['range_down']
    return control_command

def velocity_clipper(velocity):
    CLIP_VALUE = 0.5
    velocity = np.array(velocity)
    new_norm = min(norm(velocity),CLIP_VALUE)
    clipped_velocity = (velocity/norm(velocity))*new_norm if new_norm != 0 else velocity
    return clipped_velocity

def B2I(sensor_data,vec_b):
	theta = sensor_data['yaw']
	M = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
	vec_i = M.dot(np.array(vec_b))
	return vec_i

def I2B(sensor_data,vec_i):  #vec_i = vector in the inertial frame
	theta = sensor_data['yaw']
	M = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
	vec_b = np.linalg.inv(M).dot(np.array(vec_i))
	return vec_b			 #vec_b = vector in the body frame

def scanning(theta,alpha):			# theta = scanning angle, alpha = drone orientation
	global scanning_state
	seuil = 0.02
	scan_states = {'start':0,'goal_+':1,'goal_-':2,'end':3}
	phi = theta
	if scanning_state == scan_states['goal_-']: phi = -theta

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
	#print("scanning_state: ",scanning_state," | phi:",phi," | phi-alpha:",phi-alpha)
	return phi-alpha

def omega_func2(angle):
    return 2*np.sign(angle)*abs(angle)**0.5

def dist_point_to_path(point):
    global index_current_setpoint, setpoints

    v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])
    v_point = np.array(point)
    vec_path,_ = drone_goal_orientation(sensor_data)                                                     
    dir_point = v_point-v_drone
    v_dist = dir_point-(dir_point.dot(vec_path)/vec_path.dot(vec_path))*vec_path
    return v_dist

def drone_goal_orientation(sensor_data,index_current_setpoint):#positive angle means dir drone is left from dir goal
    global setpoints
    v_goal = np.array(setpoints[index_current_setpoint])                            #goal position vector
    v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
    dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector
    dist_goal = v_goal-v_drone                                                       #drone->goal vector

    angle = vector_orientation(dist_goal,dir_drone)
    return dist_goal,angle   #dist_goal is a vector

def unit_vec_per(sensor_data):
    global index_current_setpoint, setpoints
    v_goal = np.array(setpoints[index_current_setpoint])                            #goal position vector
    v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
    dir_goal = v_goal-v_drone                                                       #drone->goal vector
    v_per_unit = np.array([-dir_goal[1],dir_goal[0]])/np.linalg.norm(dir_goal)
    return v_per_unit

def curve_orientation(sensor_data):
    v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
    dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector
    v_center = np.array([2.5,1.5])
    v_drone_center = v_center-v_drone
    angle = vector_orientation(v_drone_center,dir_drone)
    return angle

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




def progressiv_start(sensor_data,desired_speed):
	acc = 0.5
	current_speed = norm(np.array([sensor_data['v_forward'],sensor_data['v_left']]))
	if desired_speed - current_speed > 0.1:
		speed = current_speed + (desired_speed-current_speed)/5
	else:
		speed = desired_speed
	return speed

