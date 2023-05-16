# You can change anything in this file except the file name of 'my_control.py',
# the class name of 'MyController', and the method name of 'step_control'.

# Available sensor data includes data['t'], data['x_global'], data['y_global'],
# data['roll'], data['pitch'], data['yaw'], data['v_forward'], data['v_left'],
# data['range_front'], data['range_left'], data['range_back'],
# data['range_right'], data['range_down'], data['yaw_rate'].

import numpy as np
from numpy.linalg import norm

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        self.on_ground = True
        self.land_flag = False
        self.height_desired = 0.5
        self.past_height = 0
        self.take_off_counter = 0
        self.initial_pos = [0,0]
        self.scanning_state = 0
        self.delta1 = 0.3
        self.delta2 = 0.1
        self.setpoints = \
                [[3.8,0.1],[3.8,0.45],[3.8,0.8],[3.8,1.15],[3.8,1.5],[3.8,1.85],[3.8,2.2],[3.8,2.55],[3.8,2.9]
                ,[4.1,2.9],[4.1,2.55],[4.1,2.2],[4.1,1.85],[4.1,1.5],[4.1,1.15],[4.1,0.8],[4.1,0.45],[4.1,0.1]
                ,[4.4,0.1],[4.4,0.45],[4.4,0.8],[4.4,1.15],[4.4,1.5],[4.4,1.85],[4.4,2.2],[4.4,2.55],[4.4,2.9]
                ,[4.7,2.9],[4.7,2.55],[4.7,2.2],[4.7,1.85],[4.7,1.5],[4.7,1.15],[4.7,0.8],[4.7,0.45],[4.7,0.1]]
        self.index_current_setpoint = 0
        self.obstacle_counter = 0
        self.DT = 0.032
        self.N = 0
        self.N2 = 0
        self.clearing_distance = 0
        self.old_vec_per = None
        self.last_front_range = 0
        self.last_angle_goal_drone = 0
        self.closest_point = np.inf
        self.direction = 0 #1 left,-1 right
        self.land_goal = np.array([0,0])
        self.drone_stabilized = True
        self.immune = False

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
            
        side_speed = 0.2
        # Take off
        if self.on_ground and sensor_data['range_down'] < 0.49:
            self.last_front_range = sensor_data['range_front']
            _,self.last_angle_goal_drone = self.drone_goal_orientation(sensor_data,self.index_current_setpoint)
            omega = self.omega_func2(-self.last_angle_goal_drone)
            control_command = [0.0, 0.0, omega, self.height_desired]
            if self.take_off_counter == 2:
                self.initial_pos = [sensor_data['x_global'],sensor_data['y_global']]
            if self.take_off_counter > 2:
                self.past_height = sensor_data['range_down']
            self.take_off_counter += 1
            return control_command
        else:
            self.on_ground = False
        
        # Landing the Drone
        if (sensor_data['t']> 4 and (self.past_height-sensor_data['range_down']) > 0.05) or self.land_flag:
            v_x, v_y = 0,0
            if self.setpoints[self.index_current_setpoint] != self.initial_pos:
                print("drone_stabilized", self.drone_stabilized)
                if self.land_flag == False:
                    entery_point = np.array([sensor_data['x_global'],sensor_data['y_global']])
                    entery_velocity = self.B2I(sensor_data,np.array([sensor_data['v_forward'],sensor_data['v_left']]))
                    self.land_goal = (entery_point + 0.1*entery_velocity/norm(entery_velocity))
                    print("entery_velocity: ", entery_velocity," and entery_point:", entery_point)
                    self.drone_stabilized = False
                else:
                    print("land_goal:", self.land_goal)
                    v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']]) 
                    v_x,v_y = self.I2B(sensor_data,self.land_goal-v_drone)
                if self.drone_stabilized == False:
                    if norm(np.array([sensor_data['v_forward'],sensor_data['v_left']]))>0.05:
                        v_x,v_y = 0,0
                    else:
                        self.drone_stabilized = True
                else:
                    self.height_desired -= 0.005
            else:
                self.height_desired -= 0.005
                dist_goal,_ = self.drone_goal_orientation(sensor_data,self.index_current_setpoint)
                print("Current goal:", self.setpoints[self.index_current_setpoint]," dist_goal: ", dist_goal)
                v_x,v_y = self.I2B(sensor_data, dist_goal)

            self.land_flag = True
            
            if sensor_data['range_down']<0.014:
                print("LANDED")
                if self.setpoints[self.index_current_setpoint] != self.initial_pos:
                    self.land_flag = False
                    self.on_ground = True
                    self.index_current_setpoint = 0
                    self.setpoints[self.index_current_setpoint] = self.initial_pos
                    self.height_desired = 0.5
            
            control_command = [v_x, v_y, 0.0, self.height_desired]
            return control_command
        
        
        #goals update
        if sensor_data['range_front'] < 1:
            k = 0
            vec_drone_obstacle = sensor_data['range_front']*np.array([np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])])
            for i in range(len(self.setpoints)-self.index_current_setpoint):
                vec_drone_goal,_ = self.drone_goal_orientation(sensor_data,self.index_current_setpoint+i-k)
                dist = norm(vec_drone_goal - vec_drone_obstacle)
                if dist < 0.3:
                    print("GOAL DELETED")
                    self.setpoints.pop(self.index_current_setpoint+i-k)
                    k+=1
            print("goal list:", self.setpoints)
        
        
        vec_goal_drone,angle_goal_drone = self.drone_goal_orientation(sensor_data,self.index_current_setpoint)
        if self.edge_goal(self.setpoints[self.index_current_setpoint]):
            desired_velocity = self.velocity_clipper(vec_goal_drone)
        else:
            desired_velocity = 0.5*vec_goal_drone/norm(vec_goal_drone)
        velocity = self.progressiv_start(sensor_data,norm(desired_velocity))*desired_velocity/norm(desired_velocity)
        
        # obstacle_avoidance
        if self.obstacle_counter > 0: self.obstacle_counter -= 1
        if (sensor_data['range_front'] < 0.7 or self.obstacle_counter>0) and self.immune == False:
            if abs(self.last_front_range-sensor_data['range_front']) > 0.6 and self.direction == 0:
                #print("last_angle_goal_drone: ", last_angle_goal_drone," and angle_goal_drone: ", angle_goal_drone)
                self.direction = np.sign(self.last_angle_goal_drone-angle_goal_drone)*np.sign(self.last_front_range-sensor_data['range_front'])
            
            if self.obstacle_counter == 0: 
                self.obstacle_counter = 1000
                self.old_vec_per = self.unit_vec_per(sensor_data)
                #velocity = self.B2I(sensor_data,[sensor_data['v_forward'],sensor_data['v_left']])
                self.N = int((((sensor_data['range_front'])*np.cos(angle_goal_drone))-0.4)/norm(velocity)/self.DT)
                print("N1: ", self.N)
                self.closest_point = sensor_data['range_front']
            if sensor_data['range_front'] < self.closest_point: self.closest_point = sensor_data['range_front']
            velocity = desired_velocity*((abs(self.closest_point-0.2))**0.5)#*np.sign(sensor_data['range_front']-0.2)
        
            if self.direction == 0:
                omega = self.omega_func2(self.scanning(np.pi/16,angle_goal_drone))
                if self.obstacle_counter == 1000-self.N:
                    self.direction = -np.sign(self.curve_orientation(sensor_data))
            else:
                omega = self.omega_func2(-angle_goal_drone)
            print("we want to go in the direction: ", self.direction," and velocity: ", norm(velocity))
            if self.obstacle_counter <= 1000-self.N:
                omega = self.omega_func2(-angle_goal_drone)
                if sensor_data['range_front'] > 0.5 or self.obstacle_counter <= 500:
                    if self.obstacle_counter > 500: 
                        self.clearing_distance = 0.1*norm(vec_goal_drone)/(norm(vec_goal_drone)-self.last_front_range)
                        self.N2 = int(self.clearing_distance/side_speed/self.DT)
                        if self.obstacle_counter == 1000-self.N:
                            self.N2 = 10
                        print("N2: ", self.N2," last_front_range: ", self.last_front_range, " norm: ", norm(vec_goal_drone),"clear:"\
                            , self.clearing_distance)
                        self.old_vec_per = self.unit_vec_per(sensor_data)
                        self.obstacle_counter = 500
                    velocity = self.progressiv_start(sensor_data,norm(desired_velocity))*desired_velocity/norm(desired_velocity)
                    print("we want to go in the direction: ", self.direction," and velocity: ", norm(velocity))
                    if self.obstacle_counter <= 500-self.N2:
                        if self.obstacle_counter <= 500-self.N2-8: 
                            self.obstacle_counter = 0
                            self.closest_point = np.inf
                            self.direction = 0
                        velocity -= self.direction*side_speed*self.old_vec_per
                velocity += self.direction*side_speed*self.old_vec_per
        else:
            # close to goal
            distance_drone_to_goal = norm(vec_goal_drone)
            if distance_drone_to_goal < 0.3:
                self.immune = True
                print("HERE WE ARE")
                if self.setpoints[self.index_current_setpoint]!= self.initial_pos:
                    _,angle_goal_drone = self.drone_goal_orientation(sensor_data,self.index_current_setpoint+1)
                    omega = self.omega_func2(-angle_goal_drone)
                else:
                    omega = 0
            else:
                omega = self.omega_func2(self.scanning(np.pi/16,angle_goal_drone))
            if distance_drone_to_goal < 0.1:
                if self.setpoints[self.index_current_setpoint]!= self.initial_pos:
                    self.index_current_setpoint+=1
            if abs(angle_goal_drone)<0.1:
                self.immune = False
                    
        
        
        #print("obstacle_counter:", self.obstacle_counter)
        print("next_goal:", self.setpoints[self.index_current_setpoint])
        v_x_b,v_y_b = self.I2B(sensor_data,velocity)
        control_command = [v_x_b,v_y_b, omega, self.height_desired]
        
        self.last_front_range = sensor_data['range_front']
        self.last_angle_goal_drone = angle_goal_drone
        self.past_height = sensor_data['range_down']
        return control_command

    def edge_goal(self,goal):
        for i in range(len(self.setpoints)):
            if goal == self.setpoints[0] or goal == self.setpoints[len(self.setpoints)-1]:
                return True
            if goal == self.setpoints[i]:
                if goal[0] == self.setpoints[i-1][0] and goal[0] == self.setpoints[i+1][0]:
                    return False
                else:
                    return True

    def velocity_clipper(self,velocity):
        CLIP_VALUE = 0.5
        velocity = np.array(velocity)
        new_norm = min(norm(velocity),CLIP_VALUE)
        clipped_velocity = (velocity/norm(velocity))*new_norm if new_norm != 0 else velocity
        return clipped_velocity
    
    def B2I(self,sensor_data,vec_b):
        theta = sensor_data['yaw']
        M = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        vec_i = M.dot(np.array(vec_b))
        return vec_i
    
    def I2B(self,sensor_data,vec_i):  #vec_i = vector in the inertial frame
        theta = sensor_data['yaw']
        M = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        vec_b = np.linalg.inv(M).dot(np.array(vec_i))
        return vec_b             #vec_b = vector in the body frame
    
    def scanning(self,theta,alpha):          # theta = scanning angle, alpha = drone orientation
        seuil = 0.02
        scan_states = {'start':0,'goal_+':1,'goal_-':2,'end':3}
        phi = theta
        if self.scanning_state == scan_states['goal_-']: phi = -theta
    
        if self.scanning_state == scan_states['start']:
            if abs(theta-alpha) < seuil:
                self.scanning_state = scan_states['goal_-']
                phi = -theta
            else:
                self.scanning_state = scan_states['goal_+']
        elif self.scanning_state == scan_states['end']:
            phi = 0
            self.scanning_state = scan_states['start']
        else:
            if abs(phi-alpha) < seuil:
                self.scanning_state +=1
        #print("scanning_state: ",self.scanning_state," | phi:",phi," | phi-alpha:",phi-alpha)
        return phi-alpha
    
    def omega_func2(self,angle):
        return 2*np.sign(angle)*abs(angle)**0.5
    
    def drone_goal_orientation(self,sensor_data,index_current_setpoint):#positive angle means dir drone is left from dir goal
        v_goal = np.array(self.setpoints[index_current_setpoint])                            #goal position vector
        v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
        dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector
        dist_goal = v_goal-v_drone                                                       #drone->goal vector
    
        angle = self.vector_orientation(dist_goal,dir_drone)
        return dist_goal,angle   #dist_goal is a vector
    
    def unit_vec_per(self,sensor_data):
        v_goal = np.array(self.setpoints[self.index_current_setpoint])                            #goal position vector
        v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
        dir_goal = v_goal-v_drone                                                       #drone->goal vector
        v_per_unit = np.array([-dir_goal[1],dir_goal[0]])/np.linalg.norm(dir_goal)
        return v_per_unit
    
    def curve_orientation(self,sensor_data):
        v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
        dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector
        v_center = np.array([2.5,1.5])
        v_drone_center = v_center-v_drone
        angle = self.vector_orientation(v_drone_center,dir_drone)
        return angle
    
    def vector_orientation(self,u,v):
            u = np.array(u).reshape((1,2))
            v = np.array(v).reshape((1,2))
            det_uv = np.linalg.det(np.concatenate([u,v],axis=0))
            scal_prod = u.dot(v.T)
    
            if scal_prod < 0 and det_uv == 0:
                angle = np.pi
            else:
                angle = np.sign(det_uv)*np.arccos(scal_prod/np.linalg.norm(u)/np.linalg.norm(v))
    
            return np.squeeze(angle).tolist()
    
    def progressiv_start(self,sensor_data,desired_speed):
        acc = 0.5
        current_speed = norm(np.array([sensor_data['v_forward'],sensor_data['v_left']]))
        if desired_speed - current_speed > 0.1:
            speed = current_speed + (desired_speed-current_speed)/5
        else:
            speed = desired_speed
        return speed

