#!/usr/bin/env python3 import rclpy from rclpy import qos
import rclpy
from rclpy.node import Node
from rclpy import qos
from visualization_msgs.msg import Marker, MarkerArray
from custom_interfaces.msg import PotholeData

from tf2_ros import Buffer, PoseStamped, TransformListener
from tf2_geometry_msgs import do_transform_pose


class ReporterNode(Node):
    MIN_POTHOLE_RADIUS = 0.015
    MAX_POTHOLE_RADIUS = 0.085
    MAX_POTHOLE_DISTANCE = 0.11

    def __init__(self):
        super().__init__("pothole_reporter")
        self.get_logger().info("Processing Pothole Data")
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
        pothole_pose = data.pothole_pose.pose
        try:
            pothole_world_tf = self.get_tf_transform("odom", "depth_link")
        except Exception as e:
            self.get_logger().warning(f"Failed to get TF: {e}")
            raise ValueError(e)
        pothole_world_pose = do_transform_pose(pothole_pose, pothole_world_tf)
        publish_position = PoseStamped()
        publish_position.pose = pothole_world_pose
        publish_position.header.frame_id = "odom"
        self.pothole_position_publisher.publish(publish_position)

        should_store_pothole = self.pothole_storage_evaluation(
            pothole_world_pose, radius
        )
        if should_store_pothole:
            self.store_pothole(publish_position, radius)

    def pothole_storage_evaluation(self, pothole_pose, radius) -> bool:
        x = pothole_pose.position.x
        y = pothole_pose.position.y

        if radius >= self.MAX_POTHOLE_RADIUS or radius < self.MIN_POTHOLE_RADIUS:
            return False

        for idx, pothole in enumerate(self.pothole_storage):
            bound_radius = max(pothole["radius"], self.MAX_POTHOLE_DISTANCE)
            # if the pothole is within a specified radius, remove it
            if x > (pothole["x"] - bound_radius) and x < (pothole["x"] + bound_radius):
                if y > (pothole["y"] - bound_radius) and y < (
                    pothole["y"] + bound_radius
                ):
                    if pothole["radius"] < radius:
                        # updating the node position when we find a larger radius
                        self.update_pothole(idx, pothole_pose, radius)
                    return False
        return True

    def update_pothole(self, idx, pose, radius):
        for marker in self.markers.markers:
            if marker.id == idx:
                marker.action = Marker.MODIFY
                marker.pose = pose
                marker.scale.x = radius * 2
                marker.scale.z = radius * 2
        self.pothole_marker.publish(self.markers)

    def store_pothole(self, publish_position, radius):
        pose = publish_position.pose

        self.pothole_storage.append(
            {
                # "timestamp": datetime.now(),
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "radius": radius,
            }
        )

        self.publish_new_marker(pose, radius)

        for idx, pothole in enumerate(self.pothole_storage):
            self.get_logger().info(
                f"{idx} - x: {pothole['x']}, y: {pothole['y']}, radius: {pothole['radius']}"
            )
        self.get_logger().info(f"Total: {len(self.pothole_storage)}")

    def publish_new_marker(self, pose, scale=0.05):
        marker = Marker()
        marker.id = len(self.markers.markers)
        marker.header.frame_id = "/odom"
        marker.action = Marker.ADD
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.scale.x = scale * 2
        marker.scale.y = 0.05
        marker.scale.z = scale * 2
        marker.pose = pose
        marker.type = 2
        marker.ns = "marked"
        marker.lifetime.sec = 0
        self.markers.markers.append(marker)

        self.pothole_marker.publish(self.markers)

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
    rclpy.init(args=args)
    node = ReporterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
