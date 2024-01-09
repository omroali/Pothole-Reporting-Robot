#!/usr/bin/env python3
import rclpy
from rclpy import qos

from rclpy.node import Node
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
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
        self.markers = MarkerArray()

        self.pothole_data_subscriber = self.create_subscription(
            PotholeData,
            "/limo/nearest_pothole_custom",
            self.process_pothole_potion_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )

        self.pothole_position_publisher = self.create_publisher(
            PoseStamped, "limo/pothole_pose", 10
        )

        self.pothole_marker = self.create_publisher(
            MarkerArray, "/visualization_marker", 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def process_pothole_potion_callback(self, data):
        radius = data.pothole_width / 2
        confidence = 0.8
        pothole_pose = data.pothole_pose.pose
        try:
            pothole_world_tf = self.get_tf_transform("odom", "depth_link")
        except Exception as e:
            self.get_logger.warning(f"Failed to get TF: {e}")
            return e
        pothole_world_pose = do_transform_pose(pothole_pose, pothole_world_tf)
        publish_position = PoseStamped()
        publish_position.pose = pothole_world_pose
        publish_position.header.frame_id = "odom"
        self.pothole_position_publisher.publish(publish_position)

        should_store_pothole = self.pothole_storage_evaluation(
            pothole_world_pose, radius
        )
        if should_store_pothole:
            self.store_pothole(publish_position, radius, confidence)

    def pothole_storage_evaluation(self, pothole_pose, radius) -> bool:
        x = pothole_pose.position.x
        y = pothole_pose.position.y

        confidence = self.evaluate_pothole_position_confidence(pothole_pose)

        if radius >= 0.5:
            return False

        if len(self.pothole_storage) == 0:
            return True

        for pothole in self.pothole_storage:
            # print("pothole:", pothole)
            # print("radius:", radius)
            bound_radius = radius

            # if the pothole is within a specified radius, remove it
            if x > (pothole["x"] - bound_radius) and x < (pothole["x"] + bound_radius):
                if y > (pothole["y"] - bound_radius) and y < (
                    pothole["y"] + bound_radius
                ):
                    return False
        return True

    def store_pothole(self, publish_position, radius, confidence):
        pose = publish_position.pose

        self.pothole_storage.append(
            {
                # "timestamp": datetime.now(),
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "radius": radius,
                "confidence": confidence,
            }
        )

        marker = Marker()
        marker.id = len(self.markers.markers)
        marker.header.frame_id = "/odom"
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.pose = pose
        marker.type = 2
        marker.ns = "marked"
        marker.lifetime = rclpy.duration.Duration().to_msg()
        self.markers.markers.append(marker)

        self.pothole_marker.publish(self.markers)
        print(self.pothole_storage)
        print("")
        print("Total:", len(self.pothole_storage))

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
            raise ValueError(e)


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
