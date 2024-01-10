#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to poses.
"""


def new_pose(
    navigator, pos_x: float, pos_y: float, orient_z: float, orient_w: float
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = pos_x
    pose.pose.position.y = pos_y
    pose.pose.orientation.z = orient_z
    pose.pose.orientation.w = orient_w
    return pose


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    navigator.setInitialPose(new_pose(navigator, 0.0, 0.0, 0.0, 1.0))

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # set our demo's goal poses to follow
    goal_poses = []

    # goal pose 1
    goal_poses.append(new_pose(navigator, 0.7, -0.05, 0.0, 0.09))  # 1
    goal_poses.append(new_pose(navigator, 1.1, -0.35, 0.70, -0.68))  # 2
    # 3 struggle with a turn at end
    goal_poses.append(new_pose(navigator, 1.18, -0.67, -0.76, 0.64))
    # 4 dse a very wide turn hits wall  then cries before reaching
    goal_poses.append(new_pose(navigator, 1.1, -0.95, -0.97, 0.22))
    goal_poses.append(new_pose(navigator, 0.35, -0.95, -1.0, 0.03))
    goal_poses.append(new_pose(navigator, -0.70, -1.0, 0.98, 0.18))  # 5
    goal_poses.append(new_pose(navigator, -1.15, -0.55, 0.60, 0.80))  # 6
    goal_poses.append(new_pose(navigator, -1.18, -0.20, 0.035, 0.95))
    goal_poses.append(new_pose(navigator, -0.71, -0.12, 0.035, 0.95))
    goal_poses.append(new_pose(navigator, -0.31, -0.14, -0.48, 0.88))
    goal_poses.append(new_pose(navigator, -0.23, -0.48, -0.69, 0.72))

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
