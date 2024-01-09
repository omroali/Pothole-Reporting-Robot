#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy import qos

from tf2_ros import Buffer, TransformListener


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
        self.nearest_pothole_pub = self.create_publisher(
            PoseStamped, "/limo/nearest_pothole", 10
        )
        # self.pothole_pose = self.create_publisher(Pose, "/limo/pothole_pose", 10)
        self.nearest_pothole_pub_custom = self.create_publisher(
            PotholeData, "/limo/nearest_pothole_custom", 10
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

    def get_depth(self, image_coords, image_depth, image):
        y, x = image_coords
        shape_img = image.shape
        shape_depth = image_depth.shape
        depth_coords = (
            shape_depth[0] / 2 + (y - image.shape[0] / 2) * self.color2depth_aspect,
            shape_depth[1] / 2 + (x - image.shape[1] / 2) * self.color2depth_aspect,
        )
        # get the depth reading at the centroid location
        try:
            depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])]
        except Exception as e:
            self.get_logger().warning(f"Failed to evaluate depth_value: {str(e)}")
            raise ValueError(e)
        return depth_value

    def find_potholes(self, image, image_depth):
        # Define the color range for detection (example: blue color)
        # target_colour_bgr = np.array([197, 0, 213])
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
        edges_result = cv2.erode(edges_result, np.ones((1, 1), np.uint8), iterations=1)
        thresh = cv2.threshold(edges_result, 128, 255, cv2.THRESH_BINARY)[1]

        # get the contours
        contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )

        for idx, c in enumerate(contours):
            # compute the center of the contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            image_coords = (M["m01"] / M["m00"], M["m10"] / M["m00"])
            try:
                depth_value = self.get_depth(image_coords, image_depth, image)
            except Exception as e:
                self.get_logger().warning(f"Failed to evaluate depth_value: {str(e)}")
                continue

            # draw the contour and center of the shape on the image
            cv2.circle(image, (cX, cY), 7, (255, 255, 0), 2)
            cv2.putText(
                image,
                f"{idx}: {round(depth_value,1)}",
                (cX - 20, cY - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (254, 255, 255),
                2,
            )
            if idx == 0:
                camera_coords = self.project_relative_robot_coords(
                    image_coords, depth_value
                )
                x, y, w, h = cv2.boundingRect(c)
                w -= 1
                h -= 1
                image_coords = (M["m01"] / M["m00"], M["m10"] / M["m00"])
                print(image.shape)
                print(x, "x")
                print(y, "y")
                print(w, "w")
                print(h, "h")

                top_left_coords = (x, y)
                bottom_right_coords = (x + w, y + h)
                print("top_left_coords", top_left_coords)
                print("bottom_right_coords", bottom_right_coords)
                # APRROXIMATING WITH THE SAME DEPTH VALUE
                try:
                    top_left_depth = self.get_depth((y, x), image_depth, image)
                    bottom_left_depth = self.get_depth(
                        (y + h, x + w), image_depth, image
                    )
                except Exception as e:
                    self.get_logger().warning(
                        f"Failed to evaluate depth_value: {str(e)}"
                    )
                    continue
                top_left = self.project_relative_robot_coords(
                    top_left_coords, top_left_depth
                )
                bottom_right = self.project_relative_robot_coords(
                    bottom_right_coords, bottom_left_depth
                )
                pothole_width = bottom_right[1] - top_left[1]
                pothole_height = bottom_right[0] - top_left[0]

                print("width", pothole_width)
                print("height", pothole_height)

                # print(f"x: {x}, y: {y}, w:{w}, h:{h}")
                # print("camera_coords", camera_coords)
                # print("top_left", top_left)
                # print("bottom_right", bottom_right)
                # print("width:", bottom_right[1] - top_left[1])
                # print("height", bottom_right[0] - top_left[0])

                # define a point in camera coordinates
                nearest_pothole = PoseStamped()
                nearest_pothole.header.frame_id = "depth_link"
                nearest_pothole.pose.orientation.w = 1.0
                nearest_pothole.pose.position.x = camera_coords[0]
                nearest_pothole.pose.position.y = camera_coords[1]
                nearest_pothole.pose.position.z = camera_coords[2]

                nearest_pothole_custom = PotholeData()
                nearest_pothole_custom.pothole_pose.pose.orientation.w = 1.0
                nearest_pothole_custom.pothole_pose.pose.position.x = camera_coords[0]
                nearest_pothole_custom.pothole_pose.pose.position.y = camera_coords[1]
                nearest_pothole_custom.pothole_pose.pose.position.z = camera_coords[2]
                nearest_pothole_custom.pothole_width = pothole_width
                nearest_pothole_custom.pothole_height = pothole_height

                # publish so we can see that in rviziterate od
                self.nearest_pothole_pub.publish(nearest_pothole)
                self.nearest_pothole_pub_custom.publish(nearest_pothole_custom)

        cv2.drawContours(image, contours, -1, (0, 255, 0), cv2.FILLED)
        cv2.imshow("Contour Detection", image)
        cv2.waitKey(1)

    def project_relative_robot_coords(self, coords, depth_value):
        # calculate object's 3d location in camera coords
        print("camera_coords:", coords)
        # y, x = coords
        #
        # if coords[1] >= 480:
        #     coords[1] = 479
        # if coords[0] >= 0:
        #     coords[0] = 479

        camera_coords = self.camera_model.projectPixelTo3dRay((coords[1], coords[0]))
        camera_coords = [x / camera_coords[2] for x in camera_coords]
        camera_coords = [x * depth_value for x in camera_coords]
        return camera_coords


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
