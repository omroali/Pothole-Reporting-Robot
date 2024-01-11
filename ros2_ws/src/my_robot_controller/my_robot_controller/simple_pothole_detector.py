#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy import qos

import image_geometry
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseStamped, PoseArray

# from custom
from custom_interfaces.msg import PotholeData
from time import sleep
from sensor_msgs.msg import Image, CameraInfo
from tf2_geometry_msgs import do_transform_pose


class PotholeNode(Node):
    COLOR_TO_DEPTH_ASPECT = 1.0
    MIN_DEPTH = 0.15
    MAX_DEPTH = 0.9
    X_PROCCESABLE = 10
    Y_PROCESSABLE = 20
    POTHOLE_BRG_LOWER = np.array([150, 0, 100])
    POTHOLE_BRG_UPPER = np.array([210, 255, 235])

    def __init__(self):
        super().__init__("simple_pothole_node")
        self.bridge = CvBridge()
        self.camera_model = None
        self.image_depth_ros = None

        self.get_logger().info("Starting Simple Pothole Detector node")

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/limo/depth_camera_link/camera_info",
            self.camera_info_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )

        self.image_sub = self.create_subscription(
            Image,
            "/limo/depth_camera_link/image_raw",
            self.image_color_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )

        self.image_depth_sub = self.create_subscription(
            Image,
            "/limo/depth_camera_link/depth/image_raw",
            self.image_depth_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )

        self.nearest_pothole_pub_custom = self.create_publisher(
            PotholeData, "/limo/nearest_pothole_custom", 10
        )

    def camera_info_callback(self, data):
        if not self.camera_model:
            self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_color_callback(self, data):
        # wait for camera_model and depth image to arrive
        if self.camera_model is None or self.image_depth_ros is None:
            return
        try:
            # covert images to open_cv
            image_colour = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            raise ValueError(e)
        self.find_potholes(image_colour, image_depth)

    def get_depth(self, image_coords, image_depth, image):
        y, x = image_coords
        shape_depth = image_depth.shape
        depth_coords = (
            shape_depth[0] / 2 + (y - image.shape[0] / 2) * self.COLOR_TO_DEPTH_ASPECT,
            shape_depth[1] / 2 + (x - image.shape[1] / 2) * self.COLOR_TO_DEPTH_ASPECT,
        )
        try:
            depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])]
        except Exception as e:
            self.get_logger().warning(f"Failed to evaluate depth_value: {str(e)}")
            raise ValueError(e)
        return depth_value

    def find_potholes(self, image, image_depth):
        # Perform color detection and grabbing contours
        img = color_detector(image, self.POTHOLE_BRG_LOWER, self.POTHOLE_BRG_UPPER)
        thresh = filter_image(img)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        accepted_contours = []

        for idx, c in enumerate(contours):
            # compute the center of the contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            image_coords = (cY, cX)

            # getting pothole depth
            try:
                depth_value = self.get_depth(image_coords, image_depth, image)
            except Exception as e:
                self.get_logger().warning(f"Failed to evaluate depth_value: {str(e)}")
                continue

            # making sure that potholes too close to image edges are ignored
            in_processable_window = (
                image_coords[1] > self.X_PROCCESABLE
                and image_coords[1] < (image.shape[1] - self.X_PROCCESABLE)
                and image_coords[0] > self.Y_PROCESSABLE
                and image_coords[0] < (image.shape[0] - self.Y_PROCESSABLE)
            )

            # making sure depth is within a band of acceptable values
            acceptable_depth = (
                depth_value > self.MIN_DEPTH and depth_value < self.MAX_DEPTH
            )

            if not (acceptable_depth and in_processable_window):
                # don't bother processing further go to next loop
                continue
            accepted_contours.append(c)

            camera_coords = self.project_relative_robot_coords(
                image_coords, depth_value
            )
            x, y, w, h = cv2.boundingRect(c)
            # to prevent hitting index 480 in img size 480
            w -= 1
            h -= 1

            # depth coords of the bounding box corners
            top_left_coords = (x, y)
            bottom_right_coords = (x + w, y + h)
            try:
                top_left_depth = self.get_depth((y, x), image_depth, image)
                bottom_right_depth = self.get_depth((y + h, x + w), image_depth, image)
            except Exception as e:
                self.get_logger().warning(f"Failed to evaluate depth_value: {str(e)}")
                continue
            top_left = self.project_relative_robot_coords(
                top_left_coords, top_left_depth
            )
            bottom_right = self.project_relative_robot_coords(
                bottom_right_coords, bottom_right_depth
            )
            pothole_width = abs(bottom_right[1] - top_left[1])
            pothole_height = abs(bottom_right[0] - top_left[0])
            pothole_width = pothole_width
            pothole_height = 0.0  # TODO: depth of the heigh of bb

            nearest_pothole = PotholeData()

            nearest_pothole.pothole_pose.pose.orientation.w = 1.0
            nearest_pothole.pothole_pose.pose.position.x = camera_coords[0]
            nearest_pothole.pothole_pose.pose.position.y = camera_coords[1]
            nearest_pothole.pothole_pose.pose.position.z = camera_coords[2]
            nearest_pothole.pothole_width = pothole_width
            nearest_pothole.pothole_height = pothole_height

            self.nearest_pothole_pub_custom.publish(nearest_pothole)

            # draw the contour and center of the shape on the image
            cv2.circle(image, (cX, cY), 7, (255, 255, 0), 2)
            cv2.putText(
                image,
                f"{idx}: {round(depth_value,1)}",
                (cX - 20, cY - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                2,
            )

        cv2.drawContours(image, accepted_contours, -1, (0, 255, 0), cv2.FILLED)
        cv2.imshow("Contour Detection", image)
        cv2.waitKey(1)

    def project_relative_robot_coords(self, coords, depth_value):
        # calculate object's 3d location in camera coords
        camera_coords = self.camera_model.projectPixelTo3dRay((coords[1], coords[0]))
        camera_coords = [x / camera_coords[2] for x in camera_coords]
        camera_coords = [x * depth_value for x in camera_coords]
        return camera_coords


def color_detector(image, lower_bound, upper_bound):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    result = cv2.bitwise_and(image, image, mask=mask)
    return result


def filter_image(image):
    image = cv2.GaussianBlur(
        image, (0, 0), sigmaX=3, sigmaY=3, borderType=cv2.BORDER_DEFAULT
    )

    # Perform Canny edge detection on blurred image
    edges_result = canny_edge_detector(image, 10, 100)

    # Morph filters on image to get better sections
    edges_result = cv2.dilate(edges_result, np.ones((2, 2), np.uint8), iterations=2)
    edges_result = cv2.erode(edges_result, np.ones((1, 1), np.uint8), iterations=1)
    thresh = cv2.threshold(edges_result, 128, 255, cv2.THRESH_BINARY)[1]
    return thresh


def canny_edge_detector(image, low_threshold, high_threshold):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, low_threshold, high_threshold)
    return edges


def main(args=None):
    rclpy.init(args=args)
    node = PotholeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
