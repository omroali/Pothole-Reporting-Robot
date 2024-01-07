import cv2
import numpy as np
import imutils
from PIL import Image, ImageFilter


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


if __name__ == "__main__":
    # Read the input image
    image = cv2.imread(
        "/home/krono/dev/RobotProgramming/Pothole-Reporting-Robot/ros2_ws/simplepothole.png"
    )

    # Define the color range for detection (example: blue color)
    target_colour_bgr = np.array([197, 0, 213])
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    tolerance = 100
    # lower_bound = np.maximum(target_colour_bgr - tolerance, 0)
    # upper_bound = np.minimum(target_colour_bgr + tolerance, 255)
    lower_bound = np.array([150, 0, 100])
    upper_bound = np.array([210, 255, 235])

    # Perform color detection
    img = color_detector(image, lower_bound, upper_bound)
    img = cv2.GaussianBlur(
        img, (0, 0), sigmaX=3, sigmaY=3, borderType=cv2.BORDER_DEFAULT
    )

    kernel = np.ones((2, 2), np.uint8)

    # Perform Canny edge detection
    edges_result = canny_edge_detector(img, 10, 100)
    # dialting the image
    edges_result = cv2.dilate(edges_result, np.ones((2, 2), np.uint8), iterations=2)
    edges_result = cv2.erode(edges_result, np.ones((1, 1), np.uint8), iterations=1)

    thresh = cv2.threshold(edges_result, 128, 255, cv2.THRESH_BINARY)[1]

    # get the (largest) contour
    contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for c in contours:
        M = cv2.moments(c)
        # compute the center of the contour
        M = cv2.moments(c)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        # draw the contour and center of the shape on the image
        # cv2.drawContours(thresh, [c], -1, (0, 255, 0), 2)
        cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
        # cv2.putText(
        #     image,
        #     "center",
        #     (cX - 20, cY - 20),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     0.5,
        #     (255, 255, 255),
        #     2,
        # )
    cv2.drawContours(image, contours, -1, (0, 255, 0), cv2.FILLED)
    # cv2.imshow("Color Detection", img)
    cv2.imshow("Contour Detection", image)
    # cv2.imshow("Canny Edge Detection", edges_result)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
