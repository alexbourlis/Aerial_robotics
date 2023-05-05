import numpy as np
from numpy import pi
from numpy.linalg import norm
import matplotlib.pyplot as plt
from work_file import vector_orientation
from math import floor

on_ground = True
height_desired = 0.5
setpoints = [[-0.0, -0.0], [0.0, -2.0], [2.0, -2.0], [2.0,  -4.0],[-1.0, -2.0],[-1.0, -3.0]]
delta = 0.3
setpoints = [[3.5+delta,delta],[3.5+delta, 3-delta], [3.5+3*delta, 3-delta], [3.5+3*delta, delta]]
goals = []
perm_list = []
pro_point = [0,0]
ind_px,ind_py = 0,0
index_current_setpoint = 0
print_flag = True
take_off_counter = 0
obstacle_info = [0,0]
scanning_state = 0    #0 is the scanning start condition

min_x, max_x = 0, 5.0 # meter
min_y, max_y = 0, 3.0 # meter
range_max = 2.0 # meter, maximum range of distance sensor
res_pos = 0.05 # meter (step)
conf = 1 # certainty given by each measurement
t = 0 # only for plotting
map = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))) # 0 = unknown, 1 = free, -1 = occupied
map_coord = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos),2)) # 0 = unknown, 1 = free, -1 = occupied


def path_planning(sensor_data):
    
    global on_ground, height_desired, index_current_setpoint, setpoints, take_off_counter, goals
    #TAKE OFF
    if on_ground and sensor_data['range_down'] < 0.49:
        occupancy_map(sensor_data)
        return take_off(sensor_data)
    else:
        on_ground = False
   
    goals = create_path(sensor_data)
    print("goals in path planning: ", goals)
    occupancy_map(sensor_data)
    angle = scanning(pi/4,drone_goal_orientation(sensor_data))
    #return hover(omega_func2(angle))
    control_command = command_to_goal(sensor_data,omega_func2(angle), np.array(setpoints[index_current_setpoint]))
    #control_command = obstacle_avoidance(sensor_data,control_command)
    return control_command
	
def take_off(sensor_data):
    # Take off
    global on_ground, height_desired, index_current_setpoint, setpoints, take_off_counter
    seuil = 0.02
    if take_off_counter > 2: #for the first cycle the sensor data is wrong
        #angle = drone_goal_orientation(sensor_data)
        #if abs(angle) >= seuil: omega = np.sign(-angle)#-np.sign(angle)
        #else: omega = 0
        omega = omega_func2(scanning(pi/4,sensor_data['yaw']))
    else:
        omega = 0
    take_off_counter += 1
    return hover(omega)

# Calculate the control command based on current goal setpoint
def command_to_goal(sensor_data,omega,goal):
    global height_desired, index_current_setpoint, setpoints
    
    theta = sensor_data['yaw']
    M = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])    #MB2I
    v_goal = goal#np.array(setpoints[index_current_setpoint])                            #goal position vector 
    v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
    dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector v
    dir_goal = v_goal-v_drone                                                       #drone->goal vector       u
    print("distance drone goal:", np.linalg.norm(dir_goal))
    v_x, v_y = np.linalg.inv(M).dot(dir_goal)                #velocity expressed in the body frame
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
	#print("scanning_state: ",scanning_state," | phi:",phi," | phi-alpha:",phi-alpha)
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

def occupancy_map(sensor_data):
    global map, t
    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']

    #my stuff
    goal_x = 3.8
    goal_y = 0.3
    ind_dx = int(np.round((pos_x - min_x)/res_pos,0))
    ind_dy = int(np.round((pos_y - min_y)/res_pos,0))
    ind_gx = int(np.round((goal_x - min_x)/res_pos,0))
    ind_gy = int(np.round((goal_y - min_y)/res_pos,0))

    #p_map = path_map(ind_dx, ind_dy, ind_gx, ind_gy, np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))))
    p_map = path_map2(np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))))
    mask = p_map == 0

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
                map_coord[idx_x, idx_y] = [(pos_x - min_x + measurement*np.cos(yaw_sensor)),(pos_y - min_y + measurement*np.sin(yaw_sensor))]
                break

    map = np.clip(map, -1, 1) # certainty can never be more than 100%
    show_map = mask*map+p_map
    #map[ind_dx][ind_dy]=10
    #map.T[20] = 5*np.ones((1,int((max_x-min_x)/res_pos)))
    # only plot every Nth time step (comment out if not needed)
    if t % 25 == 0:
        plt.imshow(np.flip(show_map,1),origin='lower') # flip the map to match the coordinate system (cmap='gray', vmin=-1, vmax=1)
        #plt.imshow(map, vmin=-1, vmax=1, cmap='gray', origin='lower')
        #plt.pause(0.01) #added
        plt.savefig("map_scan.png")
    t +=1

def create_path(sensor_data):
    global map, res_pos, perm_list, pro_point, ind_px, ind_py
    dist_min = 0.2
    s = res_pos*0.5
    N_prime = 11
    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']
    ind_dx = floor((pos_x - min_x)/res_pos)
    ind_dy = floor((pos_y - min_y)/res_pos)
    if len(perm_list)==0:
        pro_point = [pos_x,pos_y]
        ind_px, ind_py = ind_dx, ind_dy
    vec_path = np.array([3.5+0.3,0.3]) - np.array(pro_point)
    
    compteur = 0
    N = N_prime - max(abs(ind_px-ind_dx),abs(ind_py-ind_dy))

    obstacle = False
    var_list = []

    while(compteur < N): # this loop repeats itself for every new propagation point
        var_list = []
        for n in range(N-compteur): # for every perimeter
            print("propagation point:", pro_point,"in perimeter: ", n+1)
            obstacle = False
            new_goal = None
            ind_new_goal = [0,0]
            compteur += 1
            k = 2*(n+1)+1               # length of the square perimeter 
            best_score = np.inf
            absolut_best_score = np.inf
            #for every square in the perimeter
            for i in range(k):
                for j in range(k):
                    if j == 0 or j == k-1 or i == 0 or i == k-1: # perimeter points
                        ind_x,ind_y = ind_px-(n+1)+i,ind_py-(n+1)+j
                        #print("square: ",[ind_x,ind_y]," in perimeter:", n+1)
                        point = [ind_x*res_pos+s,ind_y*res_pos+s] #center of the square
                        if vec_path.dot(np.array(point)-np.array(pro_point)) >= 0: 
                            score = norm(dist_point_to_path(pro_point,point))
                            if field_state(ind_x,ind_y) == True:
                                #print("valid point")
                                if score < best_score:
                                    new_goal = point
                                    ind_new_goal = [ind_x,ind_y]
                                    best_score = score
                                    if best_score < absolut_best_score:
                                        absolut_best_score = best_score
                                        obstacle = False
                            else:
                                #print("obstacle")
                                if score < absolut_best_score:
                                    absolut_best_score = score
                                    obstacle = True
            if new_goal == None:
                print("dead end")
            if obstacle == False:
                new_goal = np.array(new_goal)-dist_point_to_path(pro_point,new_goal)
                var_list.append(new_goal.tolist())
            else:
                var_list.append(new_goal)
                var_list = adjust_goals(var_list,pro_point)
                pro_point = new_goal
                ind_px,ind_py = ind_new_goal
                if len(perm_list)>0:
                    perm_list = (np.concatenate([np.array(perm_list),np.array(var_list)], axis=0)).tolist()
                else:
                    perm_list = var_list
                break
        #print("var_list: ", var_list,"perm_list:", perm_list)

    #print("OUT: var_list: ", var_list,"perm_list:", perm_list)
    if len(perm_list)>0 and len(var_list)>0: 
        goals = (np.concatenate([np.array(perm_list),np.array(var_list)], axis=0)).tolist()
    elif len(var_list)>0: 
        goals = var_list
    else:
        goals = perm_list


    for i in range(len(goals)):
        if norm(np.array(goals[0])-np.array([pos_x,pos_y]))<dist_min:
            goals.pop(0)
            if len(perm_list)>0: perm_list.pop(0)
        else:
            break

    return goals

def adjust_goals(var_list,pro_point):
    Margin = 7
    n_points = len(var_list)-7  #number of points/goals to be changed
    if n_points > 0:
        for i in range(n_points):
            var_list[-i-1] = (np.array(var_list[-i-1])+i/n_points*(dist_point_to_path(pro_point,var_list[-1]))).tolist()
    return var_list

def field_state(ind_x,ind_y):
    global map
    k = 7
    counter = 0
    for i in range(k):
        for j in range(k):
            if ind_x-floor(k/2)+i >= 0 and ind_x-floor(k/2)+i < 100 \
            and ind_y-floor(k/2)+j >= 0 and ind_y-floor(k/2)+j < 60:
                if map[ind_x-floor(k/2)+i,ind_y-floor(k/2)+j] != -1:
                    counter += 1
    if counter == k**2:
        return True
    return False

def dist_point_to_path(pro_point,point):
    point = np.array(point)
    global index_current_setpoint, setpoints

    v_goal = np.array([3.5+0.3,0.3])                            #goal position vector
    v_pro_point = np.array(pro_point)                           #propagation point position vector
    v_point = np.array(point)
    vec_path = v_goal-v_pro_point                                                      #drone->goal vector
    dir_point = v_point-v_pro_point
    v_dist = dir_point-(dir_point.dot(vec_path)/vec_path.dot(vec_path))*vec_path
    return v_dist

def search_obstacles(sensor_data):
    global map, index_current_setpoint, setpoints
    old_goal = np.array(setpoints[index_current_setpoint])
    margin = 0.2
    N = 21
    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']
    ind_dx = int(np.round((pos_x - min_x)/res_pos,0))
    ind_dy = int(np.round((pos_y - min_y)/res_pos,0))
    shortest_dist = np.inf
    for i in range(N):
        for j in range(N):
            if ind_dx-floor(N/2)+i>=0 and ind_dx-floor(N/2)+i<100 and ind_dy-floor(N/2)+j>=0 and ind_dy-floor(N/2)+j<60:
                if map[ind_dx-floor(N/2)+i,ind_dy-floor(N/2)+j] == -1:
                    #print("indexes:", [ind_dx-floor(N/2)+i,ind_dy-floor(N/2)+j]," | map value:",map[ind_dx-floor(N/2)+i,ind_dy-floor(N/2)+j])
                    #print("drone indexes:",[ind_dx,ind_dy])
                    point = map_coord[ind_dx-floor(N/2)+i,ind_dy-floor(N/2)+j]
                    #x = dist_point_to_path(sensor_data,point)
                    x = np.linalg.norm(point-np.array([sensor_data['x_global'], sensor_data['y_global']])) #distance drone to obstacle
                    if abs(x) < abs(shortest_dist): 
                        shortest_dist = x
                        closest_point = point
                    #print("point coord:",point," | distance to path:",x)

    if abs(shortest_dist) < margin:
        if shortest_dist!=0:
            new_goal = closest_point - np.sign(dist_point_to_path(sensor_data,closest_point))*1.5*margin*unit_vec_per(sensor_data)
        else:
            new_goal = closest_point - margin*unit_vec_per(sensor_data)
    else:
        new_goal = old_goal
    print("current goal:", new_goal)
    return new_goal


#get the left-oriented unity vector perpendicular to the drone->goal vector
def unit_vec_per(sensor_data):
    global index_current_setpoint, setpoints
    v_goal = np.array(setpoints[index_current_setpoint])                            #goal position vector
    v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
    dir_goal = v_goal-v_drone                                                       #drone->goal vector
    v_per_unit = np.array([-dir_goal[1],dir_goal[0]])/np.linalg.norm(dir_goal)
    return v_per_unit

def path_map2(map):
    global goals
    for i in range(len(goals)):
        ind_x = floor((goals[i][0] - min_x)/res_pos)
        ind_y = floor((goals[i][1] - min_y)/res_pos)
        map[ind_x,ind_y] = 5 
    return map


def path_map(pos_x,pos_y,goal_x,goal_y,map):
    delta_y,delta_x = goal_y-pos_y,goal_x-pos_x
    sy = np.sign(delta_y)
    sx = np.sign(delta_x)
    delta_y += sy if sy != 0 else 1
    delta_x += sx if sx != 0 else 1
    inc = 0 if delta_y%2==0 else 1
    
    for i in range(floor(sy*delta_y/2)+inc):
        for k in range(floor(sx*delta_x/delta_y*sy)):
            map[pos_x][pos_y] = 5
            map[goal_x][goal_y] = 5
            pos_x += 1
            goal_x -= 1
        pos_y+=sy
        goal_y+=sx
        delta_x = delta_x-2*floor(sx*delta_x/delta_y*sy)
        delta_y -= sy*2

    return map 

