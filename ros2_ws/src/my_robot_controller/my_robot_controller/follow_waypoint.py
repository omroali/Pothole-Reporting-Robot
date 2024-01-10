#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


def new_pose(
    navigator, pos_x=None, pos_y=None, orient_z=None, orient_w=None
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    if pos_x is not None:
        pose.pose.position.x = pos_x
    if pos_y is not None:
        pose.pose.position.y = pos_y
    if orient_z is not None:
        pose.pose.orientation.z = orient_z
    if orient_w is not None:
        pose.pose.orientation.w = orient_w
    return pose


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    origin_pose = new_pose(navigator, 0.0, 0.0, 0.0, 1.0)
    navigator.setInitialPose(origin_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # set our demo's goal poses to follow
    goal_poses = []

    # front
    # z: 0.9961168337900886
    # w: 0.08804120308133494
    #
    #
    # right
    # z: -0.7099581434555075
    # w: 0.7042438743370147
    #
    # back
    # z: -0.9961168337900886
    # w: 0.08804120308133494
    #
    # left
    # z: 0.7098821725449702
    #   w: 0.7043204534179262

    # goal pose 1
    goal_poses.append(new_pose(navigator, 0.7, -0.05, 0.0, 0.09))
    goal_poses.append(new_pose(navigator, 0.9, -0.25, 0.70, -0.68))
    goal_poses.append(new_pose(navigator, 0.9, -0.35, 0.70, -0.68))
    goal_poses.append(new_pose(navigator, 1.1, -0.95, -0.97, 0.22))
    goal_poses.append(new_pose(navigator, 0.35, -0.95, -1.0, 0.03))
    goal_poses.append(new_pose(navigator, -0.70, -1.0, 0.98, 0.18))
    goal_poses.append(new_pose(navigator, -1.15, -0.55, 0.60, 0.80))
    goal_poses.append(new_pose(navigator, -1.18, -0.20, 0.30, 0.95))  # cries
    goal_poses.append(new_pose(navigator, -0.71, -0.12, 0.035, 0.95))
    goal_poses.append(new_pose(navigator, -0.31, -0.14, -0.48, 0.88))
    goal_poses.append(new_pose(navigator, -0.22, -0.20, -0.48, 0.88))
    goal_poses.append(new_pose(navigator, -0.15, -0.50, -0.7, 0.7))

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                "Executing current waypoint: "
                + str(feedback.current_waypoint + 1)
                + "/"
                + str(len(goal_poses))
            )
            now = navigator.get_clock().now()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal succeeded!")
    elif result == TaskResult.CANCELED:
        print("Goal was canceled!")
    elif result == TaskResult.FAILED:
        print("Goal failed!")
    else:
        print("Goal has an invalid return status!")

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == "__main__":
    main()
