from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    simple_detector_node = Node(
        package="my_robot_controller", executable="simple_pothole_detector"
    )

    waypoint_node = Node(package="my_robot_controller", executable="follow_waypoint")

    reporter_node = Node(package="my_robot_controller", executable="pothole_reporter")

    ld.add_action(simple_detector_node)
    # ld.add_action(waypoint_node)
    ld.add_action(reporter_node)

    return ld
