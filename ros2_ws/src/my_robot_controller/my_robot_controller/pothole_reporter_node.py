#!/usr/bin/env python3
import rclpy
from rclpy import qos

from rclpy.node import Node
from geometry_msgs.msg import Pose
from custom_interfaces.msg import PotholeData

from tf2_ros import Buffer, PoseStamped, TransformListener
from tf2_geometry_msgs import do_transform_pose

from datetime import datetime


class PotholeWorldData:
    def __init__(self, x, y, z, radius, confidence):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius
        self.confidence = confidence


class ReporterNode(Node):
    def __init__(self):
        # setting up the node name
        super().__init__("pothole_reporter")
        self.get_logger().info("Processing Pothole Data")  # logging in ros2
        self.pothole_storage = []

        self.pothole_data_subscriber = self.create_subscription(
            PotholeData,
            "/limo/nearest_pothole_custom",
            self.process_pothole_potion_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )

        self.pothole_position_publisher = self.create_publisher(
            PoseStamped, "limo/pothole_pose", 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def process_pothole_potion_callback(self, data):
        radius = data.pothole_width / 2
        pothole_pose = data.pothole_pose.pose
        try:
            pothole_world_tf = self.get_tf_transform("odom", "depth_link")
        except Exception as e:
            self.get_logger.warning(f"Failed to get TF: {e}")
            return
        pothole_world_pose = do_transform_pose(pothole_pose, pothole_world_tf)
        publish_position = PoseStamped()
        publish_position.pose = pothole_world_pose
        publish_position.header.frame_id = "odom"
        self.pothole_position_publisher.publish(publish_position)

        self.pothole_storage_evaluation(pothole_world_pose, radius)
        # maybe i need to allow the robot to navigate and orient first

        # track into another publisher?

        # getting robot position relative to the map
        # self.get_tf_transform('odom', )

    def pothole_storage_evaluation(self, pothole_pose, radius):
        # if pothole_has_been_seen:
        #     if the center_point is within the radius of another pothole
        #         check the confidence
        #         if the confidence of the position is higher
        #             replace the other pothole
        # if it has not been seen:
        #     add the pothole to reporter
        x = pothole_pose.position.x
        y = pothole_pose.position.y
        z = pothole_pose.position.z

        confidence = self.evaluate_pothole_position_confidence(pothole_pose)

        if radius >= 0.5:
            return

        if len(self.pothole_storage) == 0:
            self.store_pothole(x, y, z, radius, confidence)
            return

        for pothole in self.pothole_storage:
            # print("pothole:", pothole)
            # print("radius:", radius)
            bound_radius = radius

            # if the pothole is within a specified radius, remove it
            if x > (pothole["x"] - bound_radius) and x < (pothole["x"] + bound_radius):
                if y > (pothole["y"] - bound_radius) and y < (
                    pothole["y"] + bound_radius
                ):
                    # print("don't store")
                    return
        self.store_pothole(x, y, z, radius, confidence)

    def store_pothole(self, x, y, z, radius, confidence):
        self.pothole_storage.append(
            {
                # "timestamp": datetime.now(),
                "x": x,
                "y": y,
                "z": z,
                "radius": radius,
                "confidence": confidence,
            }
        )
        print(self.pothole_storage)

    def evaluate_pothole_position_confidence(self, pothole_pose):
        return 0.8

    def get_tf_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
            return transform
        except Exception as e:
            self.get_logger().warning(f"Failed to lookup transform: {str(e)}")
            return e


def main(args=None):
    # initiallising ros communications
    rclpy.init(args=args)

    # creating the node
    node = ReporterNode()

    # keeping the node alive until it's killed
    rclpy.spin(node)

    # shutting down communications
    rclpy.shutdown()


if __name__ == "__main__":
    main()
