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

        self.t_prev = rospy.get_time()
        self.max_lidar_range = rospy.get_param('~scan_range')
        self.scan_beams = rospy.get_param('~scan_beams')

        # Read the map parameters from *.yaml file
        self.mapTopic = rospy.get_param('~occ_map_topic')
        self.pOcc = rospy.get_param('~p_occ')
        self.pFree = rospy.get_param('~p_free')
        self.width = rospy.get_param('~map_width')
        self.height = rospy.get_param('~map_height')
        self.res = rospy.get_param('~map_res')
        self.size = rospy.get_param('~object_size')

        self.map_occ_grid_msg = OccupancyGrid()

        # Initialize the map meta info in the Occupancy Grid Message, e.g., frame_id, stamp, resolution, width, height, etc.
        self.map_occ_grid_msg.header.frame_id = self.mapTopic
        self.map_occ_grid_msg.header.stamp = rospy.Time.now()

        self.map_occ_grid_msg.info.resolution = self.res
        self.map_occ_grid_msg.info.width = self.width
        self.map_occ_grid_msg.info.height = self.height

        self.map_occ_grid_msg.info.origin.position.x = 0.0
        self.map_occ_grid_msg.info.origin.position.y = 0.0
        self.map_occ_grid_msg.info.origin.position.z = 0.0

        self.map_occ_grid_msg.info.origin.orientation.x = 0.0
        self.map_occ_grid_msg.info.origin.orientation.y = 0.0
        self.map_occ_grid_msg.info.origin.orientation.z = 0.0
        self.map_occ_grid_msg.info.origin.orientation.w = 1.0

        #internal map with log probability
        self.log_p = [0 for i in range(self.width * self.height)]

        # Subscribe to Lidar scan and odomery topics with corresponding lidar_callback() and odometry_callback() functions
        lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        # Create a publisher for the Occupancy Grid Map
        map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)

    # lidar_callback () uses the current LiDAR scan and Wheel Odometry data to uddate and publish the Grid Occupancy map
    def lidar_callback(self, data):
        #Create list of LIDAR angles
        num_angles = len(data.ranges)
        step_size = (data.angle_max - data.angle_min)/(num_angles - 1)
        angles = [data.angle_min + i*step_size for i in range(num_angles)]

        #assign values to cells in grid
        for i in range(self.width):
            for j in range(self.height):
                #Get coordinates & angle from i, j
                x = i*self.res
                y = j*self.res
                angle = math.atan2(y - self.y_base, x - self.x_base) - self.base_yaw

                #Get distance to cell
                cell_distance = math.sqrt((x - self.x_base)**2 + (y - self.y_base)**2)

                #get closest LIDAR beam to cell and its associated distance
                closest_measurement = min(enumerate(angles), key=lambda x: abs(x[1] - angle))[0]
                measurement_distance = data.ranges[closest_measurement]

                if cell_distance < measurement_distance:
                    #cell is free
                    self.log_p[i*self.width + j] += math.log(self.pFree/(1-self.pFree))
                elif abs(cell_distance - measurement_distance) < self.res:
                    #cell is occupied
                    self.log_p[i*self.width + j] += math.log(self.pOcc/(1-self.pOcc))

        #use log_p map to update published map
        #P > POcc -> 100
        #P < PFree -> 0
        # -1 otherwise
        self.map_occ_grid_msg.data = [self.map_p(x) for x in self.log_p]

        # Publish to map topic
        self.map_occ_grid_msg.header.stamp = rospy.Time.now()
        self.map_pub.publish(self.map_occ_grid_msg)

    #Update vehicle position
    def odom_callback(self, odom_msg):
        self.x_base = odom_msg.pose.pose.position.x
        self.y_base = odom_msg.pose.pose.position.y
        orientation_z = odom_msg.pose.pose.orientation.z
        self.base_yaw = 2.0 * math.asin(orientation_z) #z = sin(yaw/2)

    def map_p(self, n):
        if (n > self.pOcc):
            return 100
        elif (n < self.pFree):
            return 0
        else:
            return -1

def main(args):
    rospy.init_node("occupancygridmap", anonymous=True)
    OccupancyGridMap()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
