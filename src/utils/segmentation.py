import pybullet as p
import cv2
import numpy as np

from utils.perspectiveCorrection import convert_2D_to_3D
from config import TABLE_LENGTH, TABLE_WIDTH, PAMI_HEIGHT

def colorSegmentation(image, color_bounds, contour_color=(0, 255, 0), corner_color=(0, 0, 255)):
    def segment_color(image, lower_bound, upper_bound):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        kernel = np.ones((16, 16), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # Suppression du bruit
        return mask
    
    masks = {color: segment_color(image, bounds[0], bounds[1]) for color, bounds in color_bounds.items()}

    # Dictionary to store object corners
    object_corners = {}

    # Detection and labeling of objects with corners
    for color, mask in masks.items():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        result = image.copy()
        
        for i, contour in enumerate(contours):
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.drawContours(result, [contour], -1, contour_color, 2)
                cv2.putText(result, f"{color}-{i}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.drawMarker(result, (cx, cy), corner_color, cv2.MARKER_CROSS, 10, 2)

                # Corner detection
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # Store corner points
                object_corners[f"{color}-{i}"] = [tuple(point.ravel()) for point in approx]

                for point in approx:
                    x, y = point.ravel()
                    cv2.circle(result, (x, y), 5, corner_color, -1)  # Draw corners

        cv2.namedWindow(f"Detected {color} {contour_color}", cv2.WINDOW_NORMAL)
        cv2.imshow(f"Detected {color} {contour_color}", result)

    # Print the detected corners
    for obj, corners in object_corners.items():
        print(f"{obj}: {corners}")

    return object_corners

def drawObstacles3D(object_corners, image, cam_position, line_color=(0, 1, 0)):

    # Convert 2D image coordinates to 3D and draw lines
    for obj, corners in object_corners.items():
        transformed_corners = []
        
        for corner in corners:
            x_2d, y_2d = corner
            x_3d, y_3d = convert_2D_to_3D(
                y_2d, x_2d, image, cam_position, PAMI_HEIGHT
            )
            transformed_corners.append((x_3d, y_3d, PAMI_HEIGHT))  # Adding height dimension

        # Draw lines between corners
        for j in range(len(transformed_corners)):
            start_point = transformed_corners[j]
            end_point = transformed_corners[(j + 1) % len(transformed_corners)]  # Loop back to the first point

            p.addUserDebugLine(start_point, end_point, line_color, 2)  # Customizable line color
