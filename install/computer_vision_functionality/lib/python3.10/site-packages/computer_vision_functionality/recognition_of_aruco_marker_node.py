#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image

import numpy as np

from .aruco_marker_recognizer import ArucoRecognizer
from .recognition_setting import aruco_dictionary, detector_parameters,marker_size,distance_coefficients,camera_matrix

class RecognitionOfArucoMarker(Node):

    def __init__(self):
        """
        When an instance of the Node class is initialized,
        a publisher and subscript for topics are created, 
        parameters for the detector and a dictionary of markers are also implemented
        """

        super().__init__("recognition_of_aruco_marker")
        self.get_logger().info("Recognition_of_aruco_marker_node has been started")

        self.cv_bridge = CvBridge()

        self.image_from_airsim_subscriber_ = self.create_subscription(msg_type = Image, 
                                                                    topic = "/drone_vision/image_from_airsim", 
                                                                    callback = self.image_from_airsim_callback, 
                                                                    qos_profile = 10)
        
        self.image_with_marker_publisher_ = self.create_publisher(msg_type = Image, 
                                                                topic = "/drone_vision/image_with_marks", 
                                                                qos_profile = 10)
        
        self.marker_tf_publisher_ = self.create_publisher(msg_type = TransformStamped,
                                                          topic = "/drone_vision/markers_tf", 
                                                          qos_profile = 10)


        self.aruco_recognizer = ArucoRecognizer(aruco_dictionary = aruco_dictionary,
                                                marker_size = marker_size,
                                                distance_coefficients = distance_coefficients,
                                                detector_parameters = detector_parameters,
                                                camera_matrix = camera_matrix)
        

    def image_from_airsim_callback(self, msg_image: Image) -> None:
        """
        Gets an image from a topic, finds markers on it,
        saves markers transforms and an image with drawn markers in topics

        Args:
            msg_imag (Image): message with an image from the topic
        """

        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg = msg_image, desired_encoding = "bgr8")

        cv_image_with_markers, markers_ids, rotation_vectors, translation_vectors = self.aruco_recognizer.detect_aruco_markers(cv_image)

        if markers_ids is not None:
            image_with_markers_to_msg = self.cv_bridge.cv2_to_imgmsg(cvim = cv_image_with_markers, encoding = "rgb8")
            self.image_with_marker_publisher_.publish(msg = image_with_markers_to_msg)

            self.position_and_orientation_detecting(markers_ids, rotation_vectors, translation_vectors)
           

    
    def position_and_orientation_detecting(self, ids: np.ndarray, rvec: np.ndarray, tvec: np.ndarray) -> None:
        """
        This function publishes the rotation and the translation matrixs to the TransformStamped topic

        Args:
            ids (ndarray): identifiers of recognized markers
            rvec (ndarray): rotation matrixs
            tvec (ndarray): translation matrix
        """

        if ids is not None:
            for i in range(len(ids)):
                marker_transform = TransformStamped()
                marker_transform.header.frame_id = "camera_frame"
                marker_transform.child_frame_id = "marker_" + str(ids[i])
                marker_transform.transform.translation.x = float(tvec[i][0][0])
                marker_transform.transform.translation.y = float(tvec[i][0][1])
                marker_transform.transform.translation.z = float(tvec[i][0][2])

                marker_transform.transform.rotation.x = float(rvec[i][0][0])
                marker_transform.transform.rotation.y = float(rvec[i][0][1])
                marker_transform.transform.rotation.z = float(rvec[i][0][2])

                self.marker_tf_publisher_.publish(marker_transform)


def main(args=None):

    rclpy.init(args=args)

    recognition_of_aruco_marker_node = RecognitionOfArucoMarker()
    rclpy.spin(recognition_of_aruco_marker_node)

    rclpy.shutdown()