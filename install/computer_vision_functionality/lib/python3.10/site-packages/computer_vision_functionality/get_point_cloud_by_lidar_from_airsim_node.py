#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from .airsim import *

import numpy as np
import pprint
import matplotlib.pyplot as plt

from sensor_msgs.msg  import LaserScan

class LidarReaderClass(Node):

    def __init__(self):
        super().__init__("point_cloud_from_airsim")

        self.get_logger().info("Point_cloud_by_lidar_from_airsim_node has been started")

        self.airsim_client = MultirotorClient(ip = "172.18.96.1", port = 41451)
        self.is_connected_to_server = self.connect_to_server()

        '''self.point_cloud_from_airsim_publisher_ = self.create_publisher(msg_type = Image, 
                                                                topic = "/drone_vision/image_from_airsim",
                                                                qos_profile = 10)
        
        '''
        self.publisher_timer_ = self.create_timer(timer_period_sec = 0.01, callback = self.lidar_callback)




        
    def connect_to_server(self) -> bool:
        """
        Connects to the airsim API and returns a status flag

        Returns:
            bool: connecting status
        """

        try:
            self.get_logger().info("Connecting to server...") 
            self.airsim_client.confirmConnection()
            self.get_logger().info("Connection successful!") 

            return True
        
        except:
            self.get_logger().info("Connection error")

            return False
        

    def lidar_callback(self):
        """
        Saves the image from the camera to the topic 
        if there was a connection to the server
        """

        if self.is_connected_to_server is True:

            for i in range(1,2):
                lidarData = self.airsim_client.getLidarData()
                if (len(lidarData.point_cloud) < 3):
                    self.get_logger().info("\tNo points received from Lidar data")
                else:
                    points = self.parse_lidarData(lidarData)

                    #self.get_logger().info("position: X: {X}, Y:{Y}, Z:{Z}".format(X=float(points[0][0]), Y=float(points[0][1]),Z=float(points[0][2])))
                    #self.get_logger().info("orientation: X: {X}, Y:{Y}, Z:{Z}".format(X=np.rad2deg(float(points[1][0])), Y=np.rad2deg(float(points[1][1])),Z=np.rad2deg(float(points[1][2]))))
                    #self.get_logger().info("orientation: X: {X}".format(X=np.rad2deg(float(points[1][0]))))
                    
                    #self.get_logger().info(str(points[0]))
                    self.get_logger().info("\tReading %d: time_stamp: %d number_of_points: %d" % (i, lidarData.time_stamp, len(points)))
                    #self.get_logger().info("\t\tlidar position: %s" % (pprint.pformat(lidarData.pose.position)))
                    #self.get_logger().info("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))
    def parse_lidarData(self, data):

        # reshape array of floats to array of [X,Y,Z]
        points = np.array(data.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0]/3), 3))
       
        return points

def main(args = None):
    rclpy.init(args = args)

    image_from_airsim_node = LidarReaderClass()
    rclpy.spin(image_from_airsim_node)

    rclpy.shutdown()
