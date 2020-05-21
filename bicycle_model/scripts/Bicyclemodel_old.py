#!/usr/bin/env python  
import sys
import os
import math

class Bicyclemodel():

    def __init__(self, dist_rear_front = 1, max_steering_angle = 30, max_accel =1, max_speed = 20):

        # Initialize class variables
        self.dist_rear_front = dist_rear_front
        self.max_steering_angle = max_steering_angle
        self.max_accel = max_accel
        self.max_speed = max_speed

        # self.state = [x, y, yaw, v_x, v_y, w_z, a_x, a_y, delta]
        self.state = [None, None, None, None, None, None, None, None, None ]

    def init_state(self, x = 0, y = 0, yaw = 0, v_x = 0, v_y = 0, w_z = 0, a_x = 0, a_y = 0, delta = 0):
        self.state[0]  = x
        self.state[1]  = y
        self.state[2]  = yaw
        self.state[3]  = v_x
        self.state[4]  = v_y
        self.state[5]  = w_z
        self.state[6]  = a_x
        self.state[7]  = a_y
        self.state[8]  = delta

    def update(self, steering_angle = 1, speed = 1, delta_t = 0.1):
        current_steering_angle = steering_angle
        current_speed = speed
        delta_t = delta_t
        
        x       =   self.state[0]
        y       =   self.state[1]
        yaw     =   self.state[2]
        delta   =   self.state[8]

        # Desired point at the centre of front wheel; https://medium.com/@dingyan7361/simple-understanding-of-kinematic-bicycle-model-81cac6420357
        # ; https://borrelli.me.berkeley.edu/pdfpub/IV_KinematicMPC_jason.pdf

        # According to rear-axle model
        x_dot = current_speed * math.cos((yaw))
        y_dot = current_speed * math.sin((yaw))
        yaw_dot = current_speed * (math.tan(current_steering_angle)/(self.dist_rear_front)) # self.yaw_dot is theta_dot as given in aboce text
        delta_dot = current_steering_angle
        self.state[0]    =   x + x_dot * delta_t
        self.state[1]    =   y + y_dot * delta_t
        self.state[2]    =   yaw + yaw_dot * delta_t
        self.state[8]    =   delta + delta_dot * delta_t

        # Differentiating to get twist and accel

        v_x = (self.state[0] - x)/ delta_t
        v_y = (self.state[1] - y)/ delta_t
        w_z = (self.state[2] - yaw)/ delta_t

        self.state[3] = v_x
        self.state[4] = v_y
        self.state[5] = w_z

        a_x = (self.state[3] - v_x)/ delta_t
        a_y = (self.state[y] - v_y)/ delta_t
        self.state[6] = a_x
        self.state[7] = a_y

    def get_pose(self):
        current_pose = []
        current_pose[0] = self.state[0]
        current_pose[1] = self.state[1]
        current_pose[2] = self.state[2]

        return current_pose

    def get_twist(self):

        current_twist = []
        current_twist[0] = self.state[3]
        current_twist[1] = self.state[4]
        current_twist[2] = self.state[5]

    def get_accel(self):

        current_accel = []
        current_accel[0] = self.state[6] 
        current_accel[1] = self.state[7]

    def get_state(self):

        return self.state

# if __name__ == '__main__':

#     state = Bicyclemodel()
#     state.init_state()
#     current_state = state.update()
#     # print(current_state)