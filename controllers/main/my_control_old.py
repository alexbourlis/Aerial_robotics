# You can change anything in this file except the file name of 'my_control.py',
# the class name of 'MyController', and the method name of 'step_control'.

# Available sensor data includes data['t'], data['x_global'], data['y_global'],
# data['roll'], data['pitch'], data['yaw'], data['v_forward'], data['v_left'],
# data['range_front'], data['range_left'], data['range_back'],
# data['range_right'], data['range_down'], data['yaw_rate'].

import numpy as np

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        self.on_ground = True
        self.height_desired = 0.5
        self.past_height = 0
        self.pad_flag = True 
        self.setpoints = [[0, 0],[0, 0], [0, 0], [0, 0]]
        self.index_current_setpoint = 0
        self.goal_reached = False
        self.start_pos = [-1.0,-1.0]        #starting position
        self.landing = False
        self.print_flag = True
        self.obstacle_info = [0,0] #[obstacle type,around direction] The first index if for the Type of Obstacle encountered, the second is 
                                   #for the direction to follow to go around the obstacle
        self.take_off_counter = 0

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
        #Information about being on a pad or not,leaving a pad or detecting one
        pad_states = {"on_pad": 0, "off_pad": 1, "leaving_pad": 2, "entering_pad": 3}
        #Types of obstacles            

        self.initialize(sensor_data)
        
        #landing the drone
        if (self.goal_reached and self.landing) == True:
            control_command = self.landing_drone(sensor_data)
            return control_command
        #setting a new goal
        elif self.goal_reached == True and self.index_current_setpoint < 3:
            self.goal_reached = False
            self.index_current_setpoint += 1

        # Take off, we check on_ground + height
        if self.on_ground and sensor_data['range_down'] < 0.49:
            return self.take_off(sensor_data)

        pad_state = self.pad_state(sensor_data,pad_states)
        #Drone is on a pad
        if pad_state == pad_states['on_pad']:
            self.on_ground = False
            return self.path_planning(sensor_data)

        # Drone is leaving a pad
        if pad_state == pad_states['leaving_pad']:
            return self.path_planning(sensor_data)

        #Drone is on the field
        if pad_state == pad_states['off_pad']:
            return self.path_planning(sensor_data)

        #Setting the drone to land when entering a pad
        if pad_state == pad_states['entering_pad']:
            self.landing = True
            self.goal_reached = False
            dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector
            self.setpoints[self.index_current_setpoint] = [sensor_data['x_global']+dir_drone[0]*0.15,sensor_data['y_global']+dir_drone[1]*0.15]
            return self.path_planning(sensor_data)


    def initialize(self,sensor_data):
        # getting the initial position, we register the 2nd measurements of sensors (the 1st measurments are wrong)
        if self.take_off_counter == 1:
            delta = 0.29
            self.start_pos = [sensor_data['x_global'],sensor_data['y_global']]
            if sensor_data['y_global']<1.5: 
                self.setpoints = [[3.5+0.29,delta],[3.5+delta, 3-delta], [3.5+3*delta, 3-delta], [3.5+3*delta, delta]]
            else: 
                self.setpoints = [[3.5+0.29,3-delta],[3.5+delta, delta], [3.5+3*delta, delta], [3.5+3*delta, 3-delta]]

    def take_off(self,sensor_data):
        seuil = 0.02
        if self.take_off_counter > 2:
            v_goal = np.array(self.setpoints[self.index_current_setpoint])                  #goal position vector
            v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
            dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector
            dir_goal = v_goal-v_drone                                                       #drone->goal vector
            angle = self.vector_orientation(dir_goal,dir_drone)
            if abs(angle) >= seuil:  omega = -np.sign(angle)
            else: omega = 0
        else:
            omega = 0
        self.take_off_counter += 1
        control_command = [0.0, 0.0, omega, self.height_desired]

        self.past_height = sensor_data['range_down']
        if self.print_flag: print(control_command)
        return control_command

    def landing_drone(self,sensor_data):
            #if self.get_distance_to_goal(sensor_data)>0.07: 
            #   return self.path_planning(sensor_data)
            self.height_desired -= 0.005
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            self.on_ground = False
            self.past_height = sensor_data['range_down']
            return control_command

    def pad_state(self,sensor_data,pad_states):
        PAD_THRESHOLD = 0.05
        if self.pad_flag == True and self.past_height-sensor_data['range_down']<-PAD_THRESHOLD:
            self.pad_flag = False
            if self.print_flag: print("leaving pad")
            return pad_states['leaving_pad']
        elif self.pad_flag == True:
            if self.print_flag: print("on_pad")
            return pad_states['on_pad']
        elif self.pad_flag == False and self.past_height-sensor_data['range_down']>PAD_THRESHOLD:
            self.pad_flag = True
            if self.print_flag: print("entering_pad")
            return pad_states['entering_pad']
        else:
            if self.print_flag: print("off_pad")
            return pad_states['off_pad']

    def path_planning(self,sensor_data):
        
        height_desired, setpoints = self.height_desired, self.setpoints
        print_flag = self.print_flag
        seuil = 0.02

        self.past_height = sensor_data['range_down']

        # Hover at the final setpoint
        if self.index_current_setpoint == len(setpoints):
            control_command = [0.0, 0.0, 0.0, height_desired]
            return control_command

        # Get the goal position and drone position
        v_goal = np.array(setpoints[self.index_current_setpoint])                            #goal position vector            
        v_drone = np.array([sensor_data['x_global'], sensor_data['y_global']])          #drone position vector
        dir_drone = [np.cos(sensor_data['yaw']),np.sin(sensor_data['yaw'])]             #drone orientation vector
        dir_goal = v_goal-v_drone                                                       #drone->goal vector
        distance_drone_to_goal = np.linalg.norm(dir_goal)

        try:
            v_next_goal = np.array(setpoints[self.index_current_setpoint+1])                     #next goal position vector
            dir_next_goal = v_next_goal-v_drone                                             #drone->nextGoal vector
            angle = self.vector_orientation(dir_next_goal,dir_drone)
            if print_flag == True: print("drone orientation:", dir_drone," | drone->nextGoal vec:",dir_next_goal," | current goal: ", v_goal ," | next goal:", v_next_goal)
        except IndexError:
            angle = 0

        if print_flag == True: print("distance drone goal:", distance_drone_to_goal)    
        
        # When the drone reaches the goal setpoint, e.g., distance < 0.1m
        if distance_drone_to_goal < 0.1:
            # Select the next setpoint as the goal position
            if abs(angle) >= seuil:    
                omega = -np.sign(angle)*2 if abs(angle)>0.4 else -np.sign(angle)*(abs(angle)**0.25)
                control_command = [0.0, 0.0, omega, height_desired]
                return control_command
            if print_flag == True: print("alligned")
            self.goal_reached = True
            # Hover at the final setpoint
            if self.index_current_setpoint == len(setpoints):
                control_command = [0.0, 0.0, 0.0, height_desired]
                return control_command

        # Calculate the control command based on current goal setpoint
        theta = sensor_data['yaw']
        M = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]]) #MB2I
        v_goal = np.array(setpoints[self.index_current_setpoint])
        v_x, v_y = np.linalg.inv(M).dot(v_goal-v_drone)
        if v_x > v_y:                                                                           
            ratio = v_y/v_x
            v_x = np.clip(v_x,-0.5,0.5)
            v_y = v_x*ratio
        elif v_y >= v_x and v_y != 0:
            ratio = v_x/v_y
            v_y = np.clip(v_y,-0.5,0.5)
            v_x = ratio*v_y
        omega = -np.sign(angle)*abs(angle)**0.5
        control_command = [v_x,v_y , omega, height_desired]
        control_command = self.obstacle_avoidance(sensor_data,control_command)
        return control_command
            
    def vector_orientation(self,u,v):
        u = np.array(u).reshape((1,2))
        v = np.array(v).reshape((1,2))
        det_uv = np.linalg.det(np.concatenate([u,v],axis=0))
        scal_prod = u.dot(v.T)

        if scal_prod < 0 and det_uv == 0:
            angle = np.pi
        else:
            angle = np.sign(det_uv)*np.arccos(scal_prod/np.linalg.norm(u)/np.linalg.norm(v))

        if self.print_flag == True: print("vector orientation and angle_uv = ",angle)

        return np.squeeze(angle).tolist()

    def old_path_planning(self,sensor_data):

        # Hover at the final setpoint
        #if self.goal_reached == True:
        #    control_command = [0.0, 0.0, 0.0, self.height_desired]
        #    return control_command
        # check for obstacles
        #_,obstacle = self.obstacle_avoidance(sensor_data)
        #if obstacle:
        #    control_command,_ = self.obstacle_avoidance(sensor_data) 
        #    return control_command
        # Get the goal position and drone position
        self.past_height = sensor_data['range_down']
        x_goal, y_goal = self.setpoints[self.index_current_setpoint]
        x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
        distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal- y_drone])
        if self.print_flag: print("start pos:",self.start_pos," | goal pos :", x_goal,y_goal)
        if self.print_flag: print("distance to goal:", distance_drone_to_goal)
        if self.print_flag: print("")

        # When the drone reaches the goal setpoint, e.g., distance < 0.1m
        if distance_drone_to_goal < 0.07:
            # Hover at the final setpoint
            self.goal_reached = True
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command

        # Calculate the control command based on current goal setpoint
        #v_x,v_y = x_goal - x_drone, y_goal - y_drone#self.goal
        v_x,v_y = 0.2*np.sign(x_goal - x_drone),0.2*np.sign(y_goal - y_drone)
        if abs(y_goal - y_drone) < 0.2 : v_y = y_goal - y_drone
        if abs(x_goal - x_drone) < 0.2 : v_x = x_goal - x_drone
        theta = sensor_data['yaw']
        omega = -np.sign(theta) if abs(theta)>0.1 else 0
        control_command = [v_x, v_y, omega, self.height_desired]
        control_command = self.obstacle_avoidance(sensor_data,control_command)
        #if self.print_flag: print("the speeds: ", v_x, " and ", v_y)
        return control_command

    def obstacle_avoidance(self,sensor_data,control_command):
        # Obstacle avoidance with distance sensors
        obst_type = {"No_obstacle": 0,"front_obstacle": 1,"left_obstacle": 2,"right_obstacle": 3}
        #Direction to follow when getting around an obstacle 
        around_dir = {"No_direction": 0,"go_left": 1,"go_right": 2}
        O_TYPE,A_DIR = 0,1 #indices for the obstacle_info array,O_TYPE = obstacle type index | A_DIR = get around direction index

        if sensor_data['range_front'] < 0.3:
            self.obstacle_info[O_TYPE] = obst_type['front_obstacle']
            control_command[0] = 0.0
            if (sensor_data['range_left'] > sensor_data['range_right'] and self.obstacle_info[A_DIR] == around_dir['No_direction']
                ) or self.obstacle_info[A_DIR] == around_dir['go_left']:
                self.obstacle_info[A_DIR] = around_dir['go_left']
                control_command[1] = 0.7

            elif self.obstacle_info[A_DIR] == around_dir['No_direction'] or self.obstacle_info[A_DIR] == around_dir['go_right']:
                self.obstacle_info[A_DIR] = around_dir['go_right']
                control_command[1] = -0.7

        elif sensor_data['range_right'] < 0.3:# and sensor_data['v_left'] < -0:
            self.obstacle_info[O_TYPE] = obst_type['right_obstacle']
            control_command[1] = 0.2 if sensor_data['range_right'] < 0.14 else 0

        elif sensor_data['range_left'] < 0.3:# and sensor_data['v_left'] > 0: 
            self.obstacle_info[O_TYPE] = obst_type['left_obstacle']
            control_command[1] = -0.2 if sensor_data['range_left'] < 0.14 else 0
        else:
            self.obstacle_info[O_TYPE] = obst_type['No_obstacle']
            self.obstacle_info[A_DIR] = around_dir['No_direction']
        #else:
        #    control_command = None#[0.2, 0.0, 0.0, self.height_desired] #speed was 0.2 originaly

        return control_command

    def get_distance_to_goal(self,sensor_data):
        x_goal, y_goal = self.goal
        x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
        distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal- y_drone])
        return distance_drone_to_goal