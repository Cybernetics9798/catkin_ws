#!/usr/bin/env python

import numpy as np
import sys
import cv2
import time
import rospy
import tf2_ros
import math


from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class OccupancyGridMap:
    def __init__(self):
        # Topics & Subs, Pubs
        # Read paramters form params.yaml
        lidarscan_topic = rospy.get_param('~scan_topic')
        odom_topic = "/odom_imu"

        self.t_prev=rospy.get_time()
        self.max_lidar_range=rospy.get_param('~scan_range')
        self.scan_beams=rospy.get_param('~scan_beams')
        
        # Read the map parameters from *.yaml file
        mapTopic = rospy.get_param('~occ_map_topic')
        pOcc = rospy.get_param('~p_occ')
        pFree = rospy.get_param('~p_free')
        width = rospy.get_param('~map_width')
        height = rospy.get_param('~map_height')
        res = rospy.get_param('~map_res')
        size = rospy.get_param('~object_size')
        
        self.map_occ_grid_msg = OccupancyGrid()

        # Initialize the map meta info in the Occupancy Grid Message, e.g., frame_id, stamp, resolution, width, height, etc.
        self.map_occ_grid_msg.header.frame_id = mapTopic
        self.map_occ_grid_msg.header.stamp = rospy.Time.now()

        self.map_occ_grid_msg.info.resolution = res
        self.map_occ_grid_msg.info.width = width
        self.map_occ_grid_msg.info.height = height

        self.map_occ_grid_msg.info.origin.position.x = 0.0
        self.map_occ_grid_msg.info.origin.position.y = 0.0
        self.map_occ_grid_msg.info.origin.position.z = 0.0

        self.map_occ_grid_msg.info.origin.orientation.x = 0.0
        self.map_occ_grid_msg.info.origin.orientation.y = 0.0
        self.map_occ_grid_msg.info.origin.orientation.z = 0.0
        self.map_occ_grid_msg.info.origin.orientation.w = 1.0

        # Initialize the cell occuopancy probabilites to 0.5 (unknown) with all cell data in Occupancy Grid Message set to unknown 
        for i in range(width*height):
            self.map_occ_grid_msg.data[i] = 0.5
    
        # Subscribe to Lidar scan and odomery topics with corresponding lidar_callback() and odometry_callback() functions 
        lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        # Create a publisher for the Occupancy Grid Map
        map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size = 1)


    # lidar_callback () uses the current LiDAR scan and Wheel Odometry data to uddate and publish the Grid Occupancy map 
    def lidar_callback(self, data):
        
        # Publish to map topic
        self.map_occ_grid_msg.header.stamp = rospy.Time.now()
        self.map_pub.publish(self.map_occ_grid_msg)

    # odom_callback() retrives the wheel odometry data from the publsihed odom_msg
     
    def odom_callback(self, odom_msg):
        self.base_x = odom_msg.pose.pose.position.x
        self.base_y = odom_msg.pose.pose.position.y
        orientation_z = odom_msg.pose.pose.orientation.z #orienttion_z = sin(yaw/2)
        self.base_yaw = 2.0*math.asin(orientation_z)

def main(args):
    rospy.init_node("occupancygridmap", anonymous=True)
    OccupancyGridMap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
