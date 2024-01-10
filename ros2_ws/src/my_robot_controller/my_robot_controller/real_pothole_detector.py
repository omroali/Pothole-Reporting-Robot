#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy import qos

import image_geometry
import cv2
from cv_bridge import CvBridge, CvBridgeError

# from custom
from custom_interfaces.msg import PotholeData
from sensor_msgs.msg import Image, CameraInfo


class PotholeNode(Node):
    image_colour = None
    color2depth_aspect = 1.0  # for a simulated camera

    def __init__(self):
        # setting up the node name
        node_name = "real_pothole_node"
        super().__init__(node_name)
        self.bridge = CvBridge()
        self.camera_model = None
        self.image_depth_ros = None

        self.get_logger().info("Starting Real Pothole Detector node")

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
        image_colour = None
        # wait for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        try:
            image_colour = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            raise ValueError(e)

        if image_colour is not None:
            self.find_potholes(image_colour, image_depth)
        else:
            self.get_logger().info("No new image to capture")

    def get_depth(self, image_coords, image_depth, image):
        y, x = image_coords
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
        ##### SEND  IMAGE DATA TO YOLO MODEL
        contours = ["bounding", "boxes"]

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
            processable_window = (
                image_coords[1] > 30
                and image_coords[1] < 570
                and image_coords[0] > 20
                and image_coords[0] < 460
            )

            if depth_value < 0.81 and processable_window:
                camera_coords = self.project_relative_robot_coords(
                    image_coords, depth_value
                )
                pothole_width = 0.2
                pothole_height = 0.2

                nearest_pothole_custom = PotholeData()
                nearest_pothole_custom.pothole_pose.pose.orientation.w = 1.0
                nearest_pothole_custom.pothole_pose.pose.position.x = camera_coords[0]
                nearest_pothole_custom.pothole_pose.pose.position.y = camera_coords[1]
                nearest_pothole_custom.pothole_pose.pose.position.z = camera_coords[2]
                nearest_pothole_custom.pothole_width = pothole_width
                nearest_pothole_custom.pothole_height = pothole_height

                self.nearest_pothole_pub_custom.publish(nearest_pothole_custom)

        cv2.imshow("YOLO Detection", image)
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
