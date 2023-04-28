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
        self.goal = [1,0]
        self.goal_reached = False
        self.start_pos = [-1.0,-1.0,False]        #starting position
        self.landing = False
        self.printnt = False

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
        pad_states = {"on_pad": 0, "off_pad": 1, "leaving_pad": 2, "entering_pad": 3}

        self.initialize(sensor_data)
        
        #landing the drone
        if (self.goal_reached and self.landing) == True:
            control_command = self.landing_drone(sensor_data)
            return control_command
        #setting a new goal
        elif self.goal_reached == True:
            self.goal_reached = False
            self.goal = [-0.7,-0]

        # Take off, we check on_ground + height
        if self.on_ground and sensor_data['range_down'] < 0.49:
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            self.past_height = sensor_data['range_down']
            if self.printnt: print(control_command)
            return control_command

        #Drone is on a pad
        pad_state = self.pad_state(sensor_data,pad_states)
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
            self.goal = [sensor_data['x_global']-self.start_pos[0]-0.15,sensor_data['y_global']-self.start_pos[1]]
            return self.path_planning(sensor_data)


    def initialize(self,sensor_data):
        # getting the initial position, we register the 2nd measurements of sensors (the 1st measurments are wrong)
        if self.start_pos[0] < 0 and self.start_pos[2]==True: 
            self.start_pos = [sensor_data['x_global'],sensor_data['y_global'],True]
            self.past_height = sensor_data['range_down']
            #print("here")
        elif self.start_pos[2]== False:
            self.start_pos[2] = True

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
            if self.printnt: print("leaving pad")
            return pad_states['leaving_pad']
        elif self.pad_flag == True:
            if self.printnt: print("on_pad")
            return pad_states['on_pad']
        elif self.pad_flag == False and self.past_height-sensor_data['range_down']>PAD_THRESHOLD:
            self.pad_flag = True
            if self.printnt: print("entering_pad")
            return pad_states['entering_pad']
        else:
            if self.printnt: print("off_pad")
            return pad_states['off_pad']

    def path_planning(self,sensor_data):

        # Hover at the final setpoint
        #if self.goal_reached == True:
        #    control_command = [0.0, 0.0, 0.0, self.height_desired]
        #    return control_command

        # Get the goal position and drone position
        self.past_height = sensor_data['range_down']
        x_goal, y_goal = [self.start_pos[0] + self.goal[0],self.start_pos[1] + self.goal[1]]
        x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
        distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal- y_drone])
        if self.printnt: print("start pos:",self.start_pos," | goal pos :", x_goal,y_goal)
        if self.printnt: print("distance to goal:", distance_drone_to_goal)
        if self.printnt: print("")

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
        control_command = [v_x, v_y, 0.0, self.height_desired]
        #if self.printnt: print("the speeds: ", v_x, " and ", v_y)
        return control_command

    def get_distance_to_goal(self,sensor_data):
        x_goal, y_goal = [self.start_pos[0] + self.goal[0],self.start_pos[1] + self.goal[1]]
        x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
        distance_drone_to_goal = np.linalg.norm([x_goal - x_drone, y_goal- y_drone])
        return distance_drone_to_goal