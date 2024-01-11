#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy import qos
from visualization_msgs.msg import MarkerArray
from datetime import datetime

from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.platypus import (
    SimpleDocTemplate,
    Table,
    TableStyle,
    Paragraph,
    Image,
    Spacer,
)
from reportlab.lib.styles import getSampleStyleSheet
from reportlab.lib.units import inch

import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from io import BytesIO


class WaypointReporterNode(Node):
    def __init__(self):
        self.complete = False
        super().__init__("waypoint_navigation_node")
        self.markers = None

        self.pothole_marker_sub = self.create_subscription(
            MarkerArray,
            "/visualization_marker",
            self.marker_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )

    def marker_callback(self, data):
        self.markers = data.markers
        if self.complete:
            report_data = [["Pothole", "Position x", "Position y", "Size"]]
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
                        round(marker.scale.z * 2, 3),  # get diameter
                    ]
                )
            self.generate_pothole_report(report_data)
            self.destroy_node()
            rclpy.shutdown()

    def plot_data(self, data):
        idx_values = [item[0] for item in data[1:]]
        x_values = [item[1] for item in data[1:]]
        y_values = [item[2] for item in data[1:]]
        size_values = [item[3] for item in data[1:]]

        # Create a scatter plot
        plt.scatter(x_values, y_values, s=[s * 1000 for s in size_values], alpha=0.5)

        # Load an image to use as the background
        img_path = "src/my_robot_controller/resource/simple_pothole_world.png"
        img = plt.imread(img_path)

        # Display the image as the background
        plt.imshow(
            img,
            extent=[-1.45, 1.45, -1.22, 0.175],
            aspect="auto",
            alpha=0.8,
        )
        plt.axis("off")
        for i, txt in enumerate(idx_values):
            plt.text(
                x_values[i],
                y_values[i],
                str(txt),
                fontsize=8,
                ha="right",
            )

        buf = BytesIO()
        canvas = FigureCanvasAgg(plt.gcf())
        canvas.print_png(buf)
        image_data = buf.getvalue()
        image_reader = BytesIO(image_data)
        plt.close()
        return image_reader

    def generate_pothole_report(self, data):
        now = datetime.now()
        datetime_file = now.strftime("%Y-%m-%d_%H-%M-%S")
        datetime_title = now.strftime("%d/%m/%Y %H:%M:%S")
        count = len(data) - 1

        filename = f"summary_reports/pothole_report_{datetime_file}.pdf"
        document = SimpleDocTemplate(filename, pagesize=letter)

        page_width, page_height = letter
        table_width = page_width - 2 * 72
        elements = []

        title_style = getSampleStyleSheet()["Title"]
        title_text = "Pothole Location Summary"
        title = Paragraph(title_text, title_style)
        elements.append(title)

        paragraph_style = getSampleStyleSheet()["BodyText"]
        paragraph_text_1 = f"""
            This summary was produced at {datetime_title}. A total of {count} potholes were detected on this run of the pothole detection. The table below lists the positions and sizes for these potholes.
        """
        paragraph_1 = Paragraph(paragraph_text_1, paragraph_style)
        elements.append(paragraph_1)
        elements.append(Spacer(1, 0.1 * inch))

        table = Table(data, colWidths=[table_width / len(data[0])] * len(data[0]))
        style = TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.grey),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.whitesmoke),
                ("ALIGN", (0, 0), (-1, -1), "CENTER"),
                ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
                ("BOTTOMPADDING", (0, 0), (-1, 0), 4),
                ("BACKGROUND", (0, 1), (-1, -1), colors.beige),
            ]
        )

        table.setStyle(style)
        elements.append(table)
        paragraph_text_2 = """
        The plot below visually overlaps the positions of the detected potholes as well as their sizes onto the world map.
        """
        paragraph_2 = Paragraph(paragraph_text_2, paragraph_style)
        elements.append(paragraph_2)
        image = Image(self.plot_data(data), width=400, height=200)
        elements.append(image)
        document.build(elements)


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
