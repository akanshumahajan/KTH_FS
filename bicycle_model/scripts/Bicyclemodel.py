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

        #self.init_pose = from init_state()

        self.current_steering_angle = None
        self.current_speed = None
        self.delta_t = None

        self.x = None
        self.y = None
        self.yaw = None
        self.v_x = None
        self.v_y = None
        self.w_z = None
        self.a_x = None
        self.a_y = None
        self.delta = None

    def init_state(self, x = 0, y = 0, yaw = 0, v_x = 0, v_y = 0, w_z = 0, a_x = 0, a_y = 0, delta = 0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v_x = v_x
        self.v_y = v_y
        self.w_z = w_z
        self.a_x = a_x
        self.a_y = a_y
        self.delta = delta

        self.current_pose = []

        self.current_pose.append(self.x)        # self.current_pose[0]
        self.current_pose.append(self.y)        # self.current_pose[1]
        self.current_pose.append(self.yaw)      # self.current_pose[2]

        self.current_twist = []
        self.current_twist.append(self.v_x) 
        self.current_twist.append(self.v_y)
        self.current_twist.append(self.w_z)

        self.current_accel = []
        self.current_accel.append(self.a_x)
        self.current_accel.append(self.a_y)


    def update(self, steering_angle = 1, speed = 1, delta_t = 0.1):
        self.current_steering_angle = steering_angle
        self.current_speed = speed
        self.delta_t = delta_t

        # Desired point at the centre of front wheel; https://medium.com/@dingyan7361/simple-understanding-of-kinematic-bicycle-model-81cac6420357
        # ; https://borrelli.me.berkeley.edu/pdfpub/IV_KinematicMPC_jason.pdf

        self.x_dot = self.current_speed * math.cos((self.current_steering_angle + self.yaw))
        self.y_dot = self.current_speed * math.sin((self.current_steering_angle + self.yaw))
        self.yaw_dot = self.current_speed * math.sin((self.current_steering_angle)/(self.dist_rear_front)) # self.yaw_dot is theta_dot as given in aboce text
        self.delta_dot = self.current_steering_angle
        
        self.x = self.x + self.x_dot * self.delta_t
        self.y = self.x + self.x_dot * self.delta_t
        self.yaw = self.yaw + self.yaw_dot * self.delta_t
        self.delta = self.delta + self.delta_dot * self.delta_t
        

        self.get_state()

        return self.state

    def get_pose(self):

        self.current_pose[0] = (self.x)
        self.current_pose[1] = (self.y)
        self.current_pose[2] = (self.yaw)

    def get_twist(self):

        self.v_x = (self.x - self.current_pose[0])/ self.delta_t
        self.v_y = (self.y - self.current_pose[1])/ self.delta_t
        self.w_z = (self.yaw - self.current_pose[2])/ self.delta_t

        self.current_twist[0] = (self.v_x) 
        self.current_twist[1] = (self.v_y)
        self.current_twist[2] = (self.w_z)

    def get_accel(self):

        self.a_x = (self.v_x - self.current_twist[0])/ self.delta_t
        self.a_y = (self.v_y - self.current_twist[1])/ self.delta_t

        self.current_accel[0] = self.a_x 
        self.current_accel[1] = self.a_y

    def get_state(self):
        self.get_accel()
        self.get_twist()
        self.get_pose()

        self.state = []
        self.state = self.current_pose + self.current_twist + self.current_accel
        

if __name__ == '__main__':

    state = Bicyclemodel()
    state.init_state()
    current_state = state.update()
    #print(current_state)