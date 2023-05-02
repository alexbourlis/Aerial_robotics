import numpy as np

on_ground = True
height_desired = 0.5
#setpoints = [[-0.0, -0.0], [1.0, 0.0], [1.0, -4.0], [2.0,  -4.0],[-1.0, -2.0],[-1.0, -3.0]]
setpoints = [[-0.0, -0.0], [1.0, 0.0], [1.2, -4.0]]
delta = 0.3
#setpoints = [[3.5+delta,delta],[3.5+delta, 3-delta], [3.5+3*delta, 3-delta], [3.5+3*delta, delta]]
index_current_setpoint = 0
print_flag = True
take_off_counter = 0
obstacle_info = [0,0]
center = [0,0]
delta_t = 0.032
time = 0
y = 0
x = 0

def path_planning(sensor_data):
    
    global on_ground, height_desired, index_current_setpoint, setpoints, take_off_counter, time, delta_t
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
    	distance_drone_to_goal = 10
    	setpoints[index_current_setpoint] = gamma(time,sensor_data)
    	#if np.linalg.norm(np.array(setpoints[index_current_setpoint])-v_drone) < 0.8: 
    	print("timeIE: ",time)
    	time += delta_t
    	angle_goal = vector_orientation([1,0],setpoints[index_current_setpoint])
    	angle_drone = vector_orientation([1,0],v_drone)
    	if print_flag == True: print("angle goal: ", angle_goal," | angle drone: ", angle_drone)
    	if print_flag == True: print("goal pos: ", setpoints[index_current_setpoint]," | drone pos: ", v_drone)
   
    
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
    M = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]]) 	#MB2I
    v_goal = np.array(setpoints[index_current_setpoint])							#goal position vector 
    dir_goal = v_goal-v_drone 														#drone->goal vector
    angle = vector_orientation(dir_goal,dir_drone)
    omega = -2*np.sign(angle)*abs(angle)**0.5
    #omega=0
    v_x, v_y = np.clip(np.linalg.inv(M).dot(dir_goal),-5,5)
    control_command = [v_x, v_y, omega, height_desired]
    if print_flag == True: print("distance drone goal (2):", np.linalg.norm(dir_goal))
    return control_command

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

    return np.squeeze(angle).tolist()

def gamma(t,sensor_data):
	global y,x
	T = 6
	w = 2*np.pi/T
	A = 1.3
	#if t<1: 
	#	y = sensor_data['y_global']
	#	x = sensor_data['x_global']
	#if t<50:
	#	return [x,y-t]
	if t < 6:#6.88:
		return [A*np.cos(w*t),-A*np.sin(w*t)]
	if t < 9:
		return [1,6-t]
	
	return [2-A*np.cos(w*(t-9)),-2-A*np.sin(w*(t-9))] #center vector + cos,sin rebased at time t and set at the good position