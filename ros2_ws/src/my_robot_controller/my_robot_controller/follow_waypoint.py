#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy import qos
from visualization_msgs.msg import MarkerArray

from reportlab.pdfgen import canvas
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfbase import pdfmetrics
from reportlab.lib import colors
from reportlab.lib.pagesizes import letter
from reportlab.platypus import SimpleDocTemplate, Table, TableStyle

from reportlab.lib import colors
import json
from datetime import datetime


class WaypointReporterNode(Node):
    def __init__(self):
        self.complete = False
        super().__init__("waypoint_navigation_node")
        self.get_logger().info("inside node")
        self.markers = None

        self.pothole_marker_sub = self.create_subscription(
            MarkerArray,
            "/visualization_marker",
            self.marker_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )

    def marker_callback(self, data):
        self.get_logger().info("inside callback")
        self.markers = data.markers
        if self.complete:
            report_data = [["idx", "x", "y", "size"]]
            for idx, marker in enumerate(self.markers):
                self.get_logger().info(f"id: {marker.id}")
                self.get_logger().info(f"x: {marker.pose.position.x}")
                self.get_logger().info(f"y: {marker.pose.position.y}")
                self.get_logger().info(f"z: {marker.pose.position.z}")
                self.get_logger().info(f"size: {marker.scale.z}")
                report_data.append(
                    [
                        idx,
                        round(marker.pose.position.x, 3),
                        round(marker.pose.position.y, 3),
                        # "z": round(marker.pose.position.z, 3),
                        round(marker.scale.z * 2, 3),  # get diameter
                    ]
                )
            # self.generate_pothole_report(report_data)
            print(report_data)
            self.destroy_node()
            rclpy.shutdown()

    def generate_pothole_report(self, report_data):
        print(report_data)
        now = datetime.now()  # current date and time
        date_time = now.strftime("%d-%M-%Y_%H-%M-%S")

        file_name = f"pothole_report_{date_time}.pdf"
        document_title = "sample"
        title = "Robot Programming Assignment"
        sub_title = "LIMO Pothole Evaluation"

        # creating a pdf object
        pdf = canvas.Canvas(file_name)
        pdf.setTitle(document_title)
        pdf.drawCentredString(300, 770, title)
        pdf.setFillColorRGB(0, 0, 255)
        pdf.setFont("Courier-Bold", 24)
        pdf.drawCentredString(290, 720, sub_title)

        pdf.line(30, 710, 550, 710)
        text = pdf.beginText(40, 680)
        text.setFont("Courier", 18)
        text.setFillColor(colors.black)
        # for line in report_data:
        pdf.drawText(text)

        # Define the table headers
        table_headers = ["Pothole", "X", "Y", "Size"]
        # Set the starting point for the table
        x, y = 100, 700

        # Extract data from the list of dictionaries
        # table_data = [
        #     [entry["idx"], entry["x"], entry["y"], entry["z"], entry["size"]]
        #     for entry in report_data
        # ]

        # Draw headers
        for header in table_headers:
            pdf.drawString(x, y, header)
            x += 100

        # Draw data
        y -= 20
        for entry in report_data:
            x = 100
            y -= 20
            for key in ["idx", "x", "y", "size"]:
                pdf.drawString(x, y, str(entry[key]))
                x += 100

        # Create the table
        # table = Table([table_headers] + table_data)
        #
        # # Add style to the table
        # style = TableStyle(
        #     [
        #         ("BACKGROUND", (0, 0), (-1, 0), colors.grey),
        #         ("TEXTCOLOR", (0, 0), (-1, 0), colors.whitesmoke),
        #         ("ALIGN", (0, 0), (-1, -1), "CENTER"),
        #         ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
        #         ("BOTTOMPADDING", (0, 0), (-1, 0), 12),
        #         ("BACKGROUND", (0, 1), (-1, -1), colors.beige),
        #         ("GRID", (0, 0), (-1, -1), 1, colors.black),
        #     ]
        # )
        #
        # table.setStyle(style)

        # Build the PDF document
        # pdf.build([table])

        # pdf.drawInlineImage(image, 130, 400)
        pdf.save()
        #     for idx, marker in enumerate(self.markers):

    #         self.get_logger().info(f"{idx}: {marker}")


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


def send_navgation_waypoints(report_waypoint: WaypointReporterNode):
    navigator = BasicNavigator()
    origin_pose = new_pose(navigator, 0.0, 0.0, 0.0, 1.0)
    navigator.setInitialPose(origin_pose)
    navigator.waitUntilNav2Active()
    goal_poses = []

    goal_poses.append(new_pose(navigator, 0.8, 0.00, 0.0, 0.8))
    goal_poses.append(new_pose(navigator, 1.2, -0.15, 0.70, -0.68))
    goal_poses.append(new_pose(navigator, 1.1, -0.95, -0.97, 0.22))
    goal_poses.append(new_pose(navigator, 0.35, -0.95, -1.0, 0.03))
    goal_poses.append(new_pose(navigator, -0.70, -1.0, 0.98, 0.18))
    goal_poses.append(new_pose(navigator, -1.15, -0.55, 0.60, 0.80))
    goal_poses.append(new_pose(navigator, -1.18, -0.20, 0.15, 0.95))  # cries
    goal_poses.append(new_pose(navigator, -0.71, -0.12, 0.035, 0.95))
    # goal_poses.append(new_pose(navigator, -0.31, -0.14, -0.48, 0.88))
    goal_poses.append(new_pose(navigator, -0.22, -0.20, -0.48, 0.88))
    goal_poses.append(new_pose(navigator, -0.15, -0.50, -0.7, 0.7))

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
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

    result = navigator.getResult()
    navigator.lifecycleShutdown()

    if result != TaskResult.SUCCEEDED:
        print("Task was not successful")
        return False

    print("All Waypoints have been reached, generating report now")
    report_waypoint.complete = True


def main():
    rclpy.init()
    report_waypoint = WaypointReporterNode()
    send_navgation_waypoints(report_waypoint)
    rclpy.spin(report_waypoint)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
