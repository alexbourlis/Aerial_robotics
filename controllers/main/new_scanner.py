import numpy as np
from numpy import pi
from numpy.linalg import norm
import matplotlib.pyplot as plt
from math import floor

on_ground = True
height_desired = 0.5
setpoints = [[-0.0, -0.0], [0.0, -2.0], [2.0, -2.0], [2.0,  -4.0],[-1.0, -2.0],[-1.0, -3.0]]
delta = 0.3
setpoints = \
            [[3.8,0.1],[3.8,0.45],[3.8,0.8],[3.8,1.15],[3.8,1.5],[3.8,1.85],[3.8,2.2],[3.8,2.55],[3.8,2.9]
            ,[4.1,2.9],[4.1,2.55],[4.1,2.2],[4.1,1.85],[4.1,1.5],[4.1,1.15],[4.1,0.8],[4.1,0.45],[4.1,0.1]
            ,[4.4,0.1],[4.4,0.45],[4.4,0.8],[4.4,1.15],[4.4,1.5],[4.4,1.85],[4.4,2.2],[4.4,2.55],[4.4,2.9]
            ,[4.7,2.9],[4.7,2.55],[4.7,2.2],[4.7,1.85],[4.7,1.5],[4.7,1.15],[4.7,0.8],[4.7,0.45],[4.7,0.1]]
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
map_past_pos = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos)))
map_coord = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos),2)) # 0 = unknown, 1 = free, -1 = occupied


def path_planning(sensor_data):
    
    global on_ground, height_desired, index_current_setpoint, setpoints, take_off_counter, goals
    #TAKE OFF
    if on_ground and sensor_data['range_down'] < 0.49:
        occupancy_map(sensor_data)
        return take_off(sensor_data)
    else:
        on_ground = False
   
    occupancy_map(sensor_data)
    new_goals,actual_map = create_path2(sensor_data)
    #new_goals = [setpoints[index_current_setpoint]]
    vec_goal_drone,angle_goal_drone = drone_goal_orientation(sensor_data,index_current_setpoint)
    omega = omega_func2(scanning(pi/4,angle_goal_drone))
    speed = min(0.2,norm(vec_goal_drone))
    orientation = np.array(new_goals[0])-np.array([sensor_data['x_global'],sensor_data['y_global']])
    #print("orientation:", orientation,"speed:", speed)
    #print("new_goals: ", new_goals)
    velocity_i = speed*orientation/norm(orientation)

    #return hover(omega_func2(angle))
    #velocity_i = velocity_clipper(vec_goal_drone)
    v_x,v_y = I2B(sensor_data,velocity_i)
    control_command = [v_x,v_y,omega,height_desired]
    if(norm(vec_goal_drone)<0.6):
        index_current_setpoint += 1
    #control_command = obstacle_avoidance(sensor_data,control_command)
    print("distance to goal: ", norm(vec_goal_drone))

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
    return [0.0, 0.0, omega, height_desired]

def velocity_clipper(velocity):
    CLIP_VALUE = 0.2
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
    return vec_b             #vec_b = vector in the body frame

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

def create_path2(sensor_data):
    global map, map_past_pos,t,setpoints,index_current_setpoint
    list1 = map.copy()
    mask = map_past_pos == 10
    list1[mask] = 10

    #starting position
    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']
    ind_dx = floor((pos_x - min_x)/res_pos)
    ind_dy = floor((pos_y - min_y)/res_pos)
    start = [ind_dx,ind_dy]
    list1[start[0],start[1]] = 10

    #goal
    g_x,g_y = setpoints[index_current_setpoint]
    ind_gx = floor((g_x - min_x)/res_pos)
    ind_gy = floor((g_y - min_y)/res_pos)
    goal = [ind_gx,ind_gy]
    list1[goal[0],goal[1]] = 11

    #Radius of research and length of zone of reasearch
    R = 1
    L = 50 

    #creating the path

    # list of the indexes for points with value -1 (list of obstacles indexes)
    u = np.where(list1==-1)
    v = np.vstack((u[0],u[1])).T    
    # labeling the non-allowed blocks
    r1 = 1 #radius of non allowed blocks
    r2 = 2
    r3 = 3
    for i,p in enumerate(v):
        #premiere couche
        mask0 = list1[max(p[0]-r1,0):p[0]+r1+1,max(p[1]-r1,0):p[1]+r1+1] > -1
        mask1 = list1[max(p[0]-r1,0):p[0]+r1+1,max(p[1]-r1,0):p[1]+r1+1] < 10
        mask1 = mask1 & mask0
        list1[max(p[0]-r1,0):p[0]+r1+1,max(p[1]-r1,0):p[1]+r1+1] = mask1*4\
         + (~mask1)*list1[max(p[0]-r1,0):p[0]+r1+1,max(p[1]-r1,0):p[1]+r1+1]

        #2eme couche
        mask0 = list1[max(p[0]-r2,0):p[0]+r2+1,max(p[1]-r2,0):p[1]+r2+1] > -1
        mask1 = list1[max(p[0]-r2,0):p[0]+r2+1,max(p[1]-r2,0):p[1]+r2+1] < 4
        mask1 = mask1 & mask0
        list1[max(p[0]-r2,0):p[0]+r2+1,max(p[1]-r2,0):p[1]+r2+1] = mask1*3\
         + (~mask1)*list1[max(p[0]-r2,0):p[0]+r2+1,max(p[1]-r2,0):p[1]+r2+1]

        #3eme couche
        mask0 = list1[max(p[0]-r3,0):p[0]+r3+1,max(p[1]-r3,0):p[1]+r3+1] > -1
        mask1 = list1[max(p[0]-r3,0):p[0]+r3+1,max(p[1]-r3,0):p[1]+r3+1] < 3
        mask1 = mask1 & mask0
        list1[max(p[0]-r3,0):p[0]+r3+1,max(p[1]-r3,0):p[1]+r3+1] = mask1*2\
         + (~mask1)*list1[max(p[0]-r3,0):p[0]+r3+1,max(p[1]-r3,0):p[1]+r3+1]

    #moving the goal if needed
    rg = 2 # radius of obstacle free goal
    region = list1[max(goal[0]-rg,0):goal[0]+rg+1,max(goal[1]-rg,0):goal[1]+rg+1]
    #if t>400:
    #    print("region around goal:", region)
    while ((region == -1)|(region == 4)|(region == 3)|(region == 2)).sum() != 0:
        print("goal close to obstacle")
        list1[goal[0],goal[1]] = 2
        index_current_setpoint += 1
        g_x,g_y = setpoints[index_current_setpoint]
        ind_gx = floor((g_x - min_x)/res_pos)
        ind_gy = floor((g_y - min_y)/res_pos)
        goal = [ind_gx,ind_gy]
        list1[goal[0],goal[1]] = 11
        region = list1[max(goal[0]-rg,0):goal[0]+rg+1,max(goal[1]-rg,0):goal[1]+rg+1]
    #    direction = 3/2-goal[1]
    #    mask = list1 == 10
    #    list1 = map.copy()
    #    list1[mask]=10
    #    goal[1]=goal[1]+1 if direction > 0 else goal[1]-1
    #    list1[goal[0],goal[1]] = 5
    #    print("this while")
    
    chain = []
    reference = start
    propagation_point = start
    i = 0
    for i in range(L):
        #print("the main while")
        #
        lower_bound_x,upper_bound_x= max(reference[0]-R,0),reference[0]+R+1
        lower_bound_y,upper_bound_y= max(reference[1]-R,0),reference[1]+R+1
        region = list1[lower_bound_x:upper_bound_x,lower_bound_y:upper_bound_y].copy()
        if (region == 11).sum() != 0:
            #if reference == start:
            print("breaking if")
            if ((start[0]-np.array(goal)[0])**2+(start[1]-np.array(goal)[1])**2)**0.5 <= 8**0.5:
                print("exception") 
                raise Goal_reached
            break

        mask = ((region != -1) & (region < 5))
        x_coord,y_coord = np.where(region == region[mask].min())
        allowed_points = np.vstack((x_coord,y_coord)).T + np.clip(np.array(reference)-R,0,np.inf).astype(int)   #-R to center the values around the reference point
        x_coord,y_coord = np.where(mask == True)
        all_points = np.vstack((x_coord,y_coord)).T + np.clip(np.array(reference)-R,0,np.inf).astype(int)

        #print("allowed_points:\n", allowed_points, " all_points:\n ", all_points)
        allowed_angles = vector_orientation2(np.array(goal)-np.array(propagation_point),allowed_points-np.array(propagation_point))
        all_angles = vector_orientation2(np.array(goal)-np.array(propagation_point),all_points-np.array(propagation_point))
        if abs(all_angles).min()<abs(allowed_angles).min():#if my actual reference point needs to be the new propagation  point
            allowed_angles = vector_orientation2(np.array(goal)-np.array(reference),allowed_points-np.array(reference))
            propagation_point = reference
        index_min = np.where(abs(allowed_angles)==abs(allowed_angles).min())[0][0]
        reference = allowed_points[index_min].tolist()
        chain.append(reference)
        list1[reference[0],reference[1]] = 5

        #if t > 329:
        #    print("region: ", region , " and mask: ", mask)
        #    print("allowed_points:\n", allowed_points, " all_points:\n ", all_points)
        #    print("allowed_angles:\n", allowed_angles, " all_angles:\n ", all_angles)
        #    print("propagation_point:", propagation_point)
                    

        # mask:(2R+1)x(2R+1) surrounding array with the allowed places
        #lower_bound_x,upper_bound_x= max(reference[0]-R,0),reference[0]+R+1
        #lower_bound_y,upper_bound_y= max(reference[1]-R,0),reference[1]+R+1
        #mask = (list1[lower_bound_x:upper_bound_x,lower_bound_y:upper_bound_y] == 1)\
        #     | (list1[lower_bound_x:upper_bound_x,lower_bound_y:upper_bound_y] == 0)
        #non_mask = (list1[lower_bound_x:upper_bound_x,lower_bound_y:upper_bound_y] == 7)\
        #         | (list1[lower_bound_x:upper_bound_x,lower_bound_y:upper_bound_y] == -1) 
        ## if we find a square with value 5 it means we have found the goal
        #if (list1[lower_bound_x:upper_bound_x,lower_bound_y:upper_bound_y] == 11).sum() != 0:
        #    #if reference == start:
        #    print("breaking if")
        #    if ((start[0]-np.array(goal)[0])**2+(start[1]-np.array(goal)[1])**2)**0.5 <= 8**0.5:
        #        print("exception") 
        #        raise Goal_reached
        #    break
        #
        ##allowed_points contains the list of coordinates of allowed places, if statement to verify for dead ends
        #a = np.where(mask==True)[0]
        #b = np.where(mask==True)[1]
        #a2 = np.where(non_mask==True)[0]
        #b2 = np.where(non_mask==True)[1]
        #allowed_points = np.vstack((a,b)).T + np.clip(np.array(reference)-R,0,np.inf).astype(int)   #-R to center the values around the reference point
        #non_allowed_points = np.vstack((a2,b2)).T + np.clip(np.array(reference)-R,0,np.inf).astype(int)
        ##print(" ")
        ##print("allowed_points before centering",np.vstack((a,b)).T )
        ##print("NON allowed_points before centering",np.vstack((a2,b2)).T )
        ##print("la zone", list1[lower_bound_x:upper_bound_x,lower_bound_y:upper_bound_y]) 
        ##print("OUTSIDE allowed_points:", allowed_points.T," and non_allowed_points: ", non_allowed_points.T)
        ##print("OUTSIDE propagation:", propagation_point, " start point: ", start)
        #if len(allowed_points) != 0:
        #    #print("normal management")
        #    #allowed_points -= [min(0,allowed_points.T[0].min()), min(0,allowed_points.T[1].min())] #adapt to the border
        #    #distance = ((goal[0]-allowed_points.T[0])**2+(goal[1]-allowed_points.T[1])**2)**0.5
        #    #print("allowed_points-np.array(propagation_point)", (allowed_points-np.array(propagation_point)).T)
        #    angle_allowed = vector_orientation2(np.array(goal)-np.array(propagation_point),allowed_points-np.array(propagation_point))
        #    #print("angle allowed", angle_allowed.T)
        #    index_min = np.where(abs(angle_allowed)==abs(angle_allowed).min())[0][0]
        #    #index_min = np.where(distance==distance.min())[0][0]
        #    reference = allowed_points[index_min].tolist()
        #    chain.append(reference)
        #    list1[reference[0],reference[1]] = 2
        #    
        #    if len(non_allowed_points) != 0:#If the best next point is an obstacle we change the propagation point
        #        
        #        #non_allowed_points -= [min(0,non_allowed_points.T[0].min()), min(0,non_allowed_points.T[1].min())]
        #        non_allowed_angle = vector_orientation2(np.array(goal)-np.array(propagation_point),non_allowed_points-np.array(propagation_point))
        #        #print(" non allowed_points", non_allowed_points," propagation_point", propagation_point,"non_allowed_angle", non_allowed_angle)
        #        if abs(non_allowed_angle).min()<abs(angle_allowed).min():
        #            propagation_point = reference
        #        #print(" ")
        #    
        #else:#dead end management
        #    print("dead_end management")
        #    if len(chain) != 0:
        #        chain.pop(-1)
        #        list1[reference[0],reference[1]] = 7 # 7 stands for non allowed block
        #        reference = chain[-1]
        #    else:
        #        print("to be handled") #when chain is empty, which means we are close to the goal
    
        #print(" ")
        #i = (abs(np.array(start)-np.array(reference))).max()
        #print("i:", i)
        #if t % 25 == 0:
        #if t>329:
        #    plt.imshow(list1,cmap='plasma')
        #    plt.savefig("map_scan.png")
        #    plt.pause(0.05)
    
    #à gérer en dehors de la fonction
    distance = ((start[0]-np.array(chain).T[0])**2+(start[1]-np.array(chain).T[1])**2)**0.5
    indexes_close = np.where(distance<=18**0.5)[0]
    list1[np.array(chain)[indexes_close].T[0],np.array(chain)[indexes_close].T[1]] = 0
    for i in reversed(indexes_close):
        chain.pop(i)

    #plt.imshow(list1,cmap='plasma')
    #plt.pause(0.05)

    #list1[start[0],start[1]] = 9  #may be redundant
    mask = list1 == 10
    map_past_pos[mask] = 10
    #list1[np.array(chain).T[0],np.array(chain).T[1]]=0  #empty the chain points from the list1
    #list1[chain[0][0],chain[0][1]] = 9

    if t % 25 == 0:# or t>673:# (t>600 and (t % 1 == 0)):
        print(t)
        plt.imshow(np.flip(list1,1),origin='lower',cmap='plasma') # flip the map to match the coordinate system (cmap='gray', vmin=-1, vmax=1)
        #plt.imshow(map, vmin=-1, vmax=1, cmap='gray', origin='lower')
        #plt.pause(0.01) #added
        plt.savefig("map_scan.png")
    t +=1

    chain = (np.array(chain)*res_pos + res_pos/2).tolist()
    #print("chain: ", chain)

    return chain, list1


def drone_goal_orientation(sensor_data,index_current_setpoint):#positive angle means dir drone is left from dir goal
    global setpoints
    v_goal = np.array(setpoints[index_current_setpoint])                            #goal position vector
    v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
    dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector
    dist_goal = v_goal-v_drone                                                       #drone->goal vector

    angle = vector_orientation(dist_goal,dir_drone)
    return dist_goal,angle   #dist_goal is a vector

def vector_orientation2(u,v):
    u = np.array(u).reshape((1,2))
    u_prime = np.array([[-u[0][1],u[0][0]]]) 
    v = np.array(v)
    angle = np.zeros((v.shape[0],1))
    det_uv = v.dot(u_prime.T)
    scal_prod = v.dot(u.T)

    mask = (scal_prod < 0) & (det_uv == 0)
    angle[mask] = np.pi
    if len(np.where(norm(v,axis=1).reshape((v.shape[0],1))[~mask].T==0)[0])!=0:
        print("les normes de u: ",norm(u)," et de v: ", norm(v,axis=1).reshape((v.shape[0],1))[~mask].T)
        print("scal_prod:", scal_prod[~mask].T)
        print("det_uv:",np.sign(det_uv[~mask]).T, " et arccos:",np.arccos(np.clip(scal_prod[~mask]/norm(u)/norm(v,axis=1).reshape((v.shape[0],1))[~mask],-1,1).T))
    angle[~mask] = np.sign(det_uv[~mask])*np.arccos(np.clip(scal_prod[~mask]/norm(u)/norm(v,axis=1).reshape((v.shape[0],1))[~mask],-1,1))

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

def omega_func(angle):
	return np.sign(angle)*2 if abs(angle)>0.4 else np.sign(angle)*(abs(angle)**0.25)

def omega_func2(angle):
    return 2*np.sign(angle)*abs(angle)**0.5

def occupancy_map(sensor_data):
    global map, t
    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']

    #my stuff
    #goal_x = 3.8
    #goal_y = 0.3
    #ind_dx = int(np.round((pos_x - min_x)/res_pos,0))
    #ind_dy = int(np.round((pos_y - min_y)/res_pos,0))
    #ind_gx = int(np.round((goal_x - min_x)/res_pos,0))
    #ind_gy = int(np.round((goal_y - min_y)/res_pos,0))

    #p_map = path_map(ind_dx, ind_dy, ind_gx, ind_gy, np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))))
    #p_map = path_map2(np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))))
    #mask = p_map == 0

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

    map = np.clip(map, -1, 0) # certainty can never be more than 100%
    #show_map = map.copy()#mask*map+p_map
    #map[ind_dx][ind_dy]=10
    #map.T[20] = 5*np.ones((1,int((max_x-min_x)/res_pos)))
    # only plot every Nth time step (comment out if not needed)
    #if t % 25 == 0:
    #    plt.imshow(np.flip(show_map,1),origin='lower') # flip the map to match the coordinate system (cmap='gray', vmin=-1, vmax=1)
    #    #plt.imshow(map, vmin=-1, vmax=1, cmap='gray', origin='lower')
    #    #plt.pause(0.01) #added
    #    plt.savefig("map_scan.png")
    #t +=1

def path_map2(map):
    global goals
    for i in range(len(goals)):
        ind_x = floor((goals[i][0] - min_x)/res_pos)
        ind_y = floor((goals[i][1] - min_y)/res_pos)
        map[ind_x,ind_y] = 5 
    return map





