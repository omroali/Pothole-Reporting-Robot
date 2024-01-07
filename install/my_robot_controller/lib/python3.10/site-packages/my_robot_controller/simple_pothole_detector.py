#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy import qos

from tf2_ros import Buffer, TransformListener


import image_geometry
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from time import sleep

from sensor_msgs.msg import Image, CameraInfo


class PotholeNode(Node):
    image_colour = None
    color2depth_aspect = 1.0  # for a simulated camera

    def __init__(self):
        # setting up the node name
        node_name = "simple_pothole_node"
        super().__init__(node_name)
        self.bridge = CvBridge()
        self.camera_model = None
        self.image_depth_ros = None

        self.get_logger().info("hello from pothole hello")
        self.get_logger().info("starting in timer callback")

        # 1 get image from the robot
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/limo/depth_camera_link/camera_info",
            self.camera_info_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.get_logger().info("camera_info_sub")

        self.object_location_pub = self.create_publisher(
            PoseStamped, "/limo/object_location", 10
        )
        self.get_logger().info("object_location_pub")

        self.image_sub = self.create_subscription(
            Image,
            "/limo/depth_camera_link/image_raw",
            self.image_color_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.get_logger().info("image_sub")

        self.image_depth_sub = self.create_subscription(
            Image,
            "/limo/depth_camera_link/depth/image_raw",
            self.image_depth_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.get_logger().info("image_depth_sub")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 2 process image for detection

        # 3 map detected pothole into report data

    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_color_callback(self, data):
        image_colour = None
        # wait for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        # covert images to open_cv
        try:
            image_colour = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)
            return

        if image_colour is not None:
            self.find_potholes(image_colour, image_depth)
        else:
            self.get_logger().info("No new image to capture")

    def find_potholes(self, image, image_depth):
        # Define the color range for detection (example: blue color)
        target_colour_bgr = np.array([197, 0, 213])
        # lower_bound = np.maximum(target_colour_bgr - tolerance, 0)
        # upper_bound = np.minimum(target_colour_bgr + tolerance, 255)

        self.get_logger().info("debug_1")
        lower_bound = np.array([150, 0, 100])
        upper_bound = np.array([210, 255, 235])

        # Perform color detection
        img = color_detector(image, lower_bound, upper_bound)
        img = cv2.GaussianBlur(
            img, (0, 0), sigmaX=3, sigmaY=3, borderType=cv2.BORDER_DEFAULT
        )

        # Perform Canny edge detection
        edges_result = canny_edge_detector(img, 10, 100)
        # dialting the image
        edges_result = cv2.dilate(edges_result, np.ones((2, 2), np.uint8), iterations=2)
        thresh = cv2.threshold(edges_result, 128, 255, cv2.THRESH_BINARY)[1]

        # get the (largest) contours
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        self.get_logger().info("debug_3")
        for idx, c in enumerate(contours):
            M = cv2.moments(c)
            # compute the center of the contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # getting the depth data
            # calculate the y,x centroid
            image_coords = (M["m01"] / M["m00"], M["m10"] / M["m00"])
            # "map" from color to depth image
            depth_coords = (
                image_depth.shape[0] / 2
                + (image_coords[0] - image.shape[0] / 2) * self.color2depth_aspect,
                image_depth.shape[1] / 2
                + (image_coords[1] - image.shape[1] / 2) * self.color2depth_aspect,
            )
            # get the depth reading at the centroid location
            depth_value = image_depth[
                int(depth_coords[0]), int(depth_coords[1])
            ]  # you might need to do some boundary checking first!

            # draw the contour and center of the shape on the image
            # cv2.drawContours(thresh, [c], -1, (0, 255, 0), 2)
            cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
            cv2.putText(
                image,
                f"{idx}: {round(depth_value,2)}",
                (cX - 20, cY - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )
        cv2.drawContours(image, contours, -1, (0, 255, 0), cv2.FILLED)
        # cv2.imshow("Color Detection", img)
        cv2.imshow("Contour Detection", image)
        cv2.waitKey(10)


def color_detector(image, lower_bound, upper_bound):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    result = cv2.bitwise_and(image, image, mask=mask)
    return result


def canny_edge_detector(image, low_threshold, high_threshold):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, low_threshold, high_threshold)
    return edges


def main(args=None):
    # initiallising ros communications
    rclpy.init(args=args)

    # creating the node
    node = PotholeNode()

    # keeping the node alive until it's killed
    rclpy.spin(node)

    # shutting down communications
    rclpy.shutdown()


if __name__ == "__main__":
    main()
