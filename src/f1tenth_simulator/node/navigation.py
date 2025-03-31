#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import time

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry


#TODO - I assumed that 0 faces backwards, 180 faces forwards based on suggested angles
#We should ask TA

class WallFollow:
    def __init__(self):

        # Read the Wall-Following controller parameters form params.yaml
        # TODO - add parameters to params.yaml (Ask TA if we continue with one we already have)
        self.kp = rospy.get_param('~kp')
        self.kd = rospy.get_param('~kd')
        self.l = rospy.get_param('~wheelbase')
        self.delta_max = rospy.get_param('~delta_max')
        self.v_ds = rospy.get_param('~v_ds')
        self.d_tau = rospy.get_param('~d_tau')
        self.d_stop = rospy.get_param('~d_stop')
        self.bl_angle = rospy.get_param('~bl_angle')
        self.br_angle = rospy.get_param('~br_angle')
        self.al_angle = rospy.get_param('~al_angle')
        self.ar_angle = rospy.get_param('~ar_angle')
        self.delta_theta = rospy.get_param('~delta_theta')

        #Desired distance between left and right walls
        #Can change if we want it to be nonzero
        self.dlr_des = 0.0

        #If no minimum distance found - use this distance
        self.fallback_d_ob = 1.0 #

        #initialize velocity to 0
        self.vel = 0.0
    
        # Subscribe to LiDAR scan Wheel Odometry topics. This is to read the LiDAR scan data and vehicle actual velocity
        self.lidarscan_topic = rospy.get_param('~scan_topic')
        self.odom_topic = rospy.get_param('~odom_topic')
        self.drive_topic = rospy.get_param('~drive_topic')

        self.lidar_sub = rospy.Subscriber(self.lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback,queue_size=1)

        # Create a publisher for the Drive topic
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)

        #epsilon to prevent divide by zero errors
        self.epsilon = 1e-5
        
    #get the lidar distance data at some angle
    def get_dist(self, data, angle_rad):

        angle_index = int((angle_rad - data.angle_min) / data.angle_increment)

        #make sure index is within ranges to prevent program crash
        angle_index = max(0, min(angle_index, len(data.ranges) - 1))

        return data.ranges[angle_index]

    # The LiDAR callback function is where you read LiDAR scan data as it becomes available and compute the vehicle velocity and steering angle commands
    def lidar_callback(self, data):      

      # Extract the parameters of two walls on the left and right side of the vehicles. Referring to Fig. 1 in the lab instructions, these are al, bl, thethal, ...
      # Extract distances at angles bl, br, al, ar
        br = self.get_dist(data, math.radians(self.br_angle))
        bl = self.get_dist(data, math.radians(self.bl_angle))
        ar = self.get_dist(data, math.radians(self.ar_angle))
        al = self.get_dist(data, math.radians(self.al_angle))

        theta_l = math.radians(self.bl_angle - self.al_angle)
        theta_r = math.radians(self.br_angle - self.ar_angle)

        beta_l = math.atan2(al*math.cos(theta_l) - bl, al*math.sin(theta_l))
        beta_r = math.atan2(ar*math.cos(theta_r) - br, ar*math.sin(theta_r))

        alpha_l = -bl + 3*math.pi/2 - math.radians(self.bl_angle)
        alpha_r = -br + math.pi/2 - math.radians(self.br_angle)

        dl = bl*math.cos(beta_l)
        dr = br*math.cos(beta_r)

        dlr = dl - dr
        dlr_dot = -self.vel*math.sin(alpha_l) - self.vel*math.sin(alpha_r)
        dlr_tilde = dlr - self.dlr_des

      # Compute the steering angle command to maintain the vehicle in the middle of left and and right walls
        delta = math.atan(
            (-self.l/(self.vel**2 * (math.cos(alpha_r) + math.cos(alpha_l) + self.epsilon))) *
            (-self.kp*dlr_tilde - self.kd*dlr_dot))

        #make sure delta is within vehicle steering limit
        delta = max(min(delta, self.delta_max), -self.delta_max)
 
      # Find the closest obstacle point within a narrow viewing angle in front of the vehicle and compute the vehicle velocity command accordingly
      #  ...
        front_ranges = []
        #Convert to radians, divide by 2, we search +/- this angle in radians
        #TODO - I think 180 is front of vehicle?? may need to adjust this code
        half_cone = self.delta_theta*math.pi/180/2
        start_angle = math.pi - half_cone
        end_angle = math.pi + half_cone

        start_index = int((start_angle - data.angle_min) / data.angle_increment)
        end_index = int((end_angle - data.angle_min) / data.angle_increment)

        #ensure ranges are within bounds
        start_index = max(0, start_index)
        end_index = min(len(data.ranges) - 1, end_index)

        front_range = [data.ranges[i] for i in range(start_index, end_index + 1)
                       if not math.isinf(data.ranges[i])]

        d_ob = min(front_ranges) if front_ranges else self.fallback_d_ob

        #Update speed
        v_sc = self.v_ds*(1 - math.exp(-max(d_ob - self.d_stop, 0)/self.d_tau))

      # Publish steering angle and velocity commnads to the Drive topic
        drive_msg = AckermannDriveStamped() #create new message called drive_msg
        drive_msg.drive.steering_angle = delta #update drive_msg fields
        drive_msg.drive.speed = v_sc
        self.drive_pub.publish(drive_msg) #publish message


    # The Odometry callback reads the actual vehicle velocity from VESC.
    #Done
    def odom_callback(self, odom_msg):
        # update current speed
        self.vel = odom_msg.twist.twist.linear.x


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
