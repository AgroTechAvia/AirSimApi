#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from .airsim import *

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

import numpy as np

class ImageFromAirsimNode(Node):

    def __init__(self):

        """
        When an instance of the Node class is initialized,
        a client is created to connect to AirSim, 
        as well as a publisher with a timer that will save the image 
        from the camera to the topic
        """

        super().__init__("image_from_airsim")
        self.get_logger().info("Image_from_airsim_node has been started")

        self.airsim_client = MultirotorClient(ip = "172.18.96.1", port = 41451)
        self.is_connected_to_server = self.connect_to_server()

        self.cv_bridge = CvBridge()

        self.image_from_airsim_publisher_ = self.create_publisher(msg_type = Image, 
                                                                topic = "/drone_vision/image_from_airsim",
                                                                qos_profile = 10)
        
        self.publisher_timer_ = self.create_timer(timer_period_sec = 0.06, callback = self.image_callback)

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

    def get_image_from_airsim(self) -> np.ndarray:
        """
        Accesses the API and receives an image from the camera, 
        converts it into a format for working in openCV

        Returns:
            ndarray: openCV image
        """

        raw_image_from_airsim = self.airsim_client.simGetImage(camera_name = "0", image_type =  ImageType.Scene)
        raw_cv2_image = cv2.imdecode(string_to_uint8_array(bstr = raw_image_from_airsim), flags = cv2.IMREAD_UNCHANGED)
            
        return raw_cv2_image

    def image_callback(self):
        """
        Saves the image from the camera to the topic 
        if there was a connection to the server
        """

        if self.is_connected_to_server is True:
            image_to_bridge = self.get_image_from_airsim()

            rgb_image_to_bridge = cv2.cvtColor(src = image_to_bridge, code = cv2.COLOR_RGBA2RGB)
            image_to_msg = self.cv_bridge.cv2_to_imgmsg(cvim = rgb_image_to_bridge, encoding = "rgb8")

            self.image_from_airsim_publisher_.publish(msg = image_to_msg)


def main(args = None):
    rclpy.init(args = args)

    image_from_airsim_node = ImageFromAirsimNode()
    rclpy.spin(image_from_airsim_node)

    rclpy.shutdown()
        
       