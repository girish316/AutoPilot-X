# -*- coding: utf-8 -*-
"""road_lane_detection_video.py"""

import cv2
import numpy as np

# Video source path
video_path = "NeuroDrive/lane_detect_test/vid1.mp4"

def grey(image):
    # Convert to grayscale
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def gauss(image):
    # Apply Gaussian Blur to reduce noise and smoothen the image
    return cv2.GaussianBlur(image, (5, 5), 0)

def canny(image):
    # Outline the strongest gradients in the image (edges)
    edges = cv2.Canny(image, 50, 150)
    return edges

def region(image):
    height, width = image.shape
    # Isolate the gradients that correspond to the lane lines
    triangle = np.array([[(100, height), (475, 325), (width, height)]])
    # Create a mask (triangle that isolates the region of interest in the image)
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, triangle, 255)
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image

def display_lines(image, lines):
    lines_image = np.zeros_like(image)
    # Make sure array isn't empty
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line
            # Draw lines on a black image
            cv2.line(lines_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return lines_image

def average(image, lines):
    left = []
    right = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            # Fit line to points, return slope and y-int
            parameters = np.polyfit((x1, x2), (y1, y2), 1)
            slope = parameters[0]
            y_int = parameters[1]
            # Lines on the right have positive slope, and lines on the left have negative slope
            if slope < 0:
                left.append((slope, y_int))
            else:
                right.append((slope, y_int))

    # Take averages among all the lines
    right_avg = np.average(right, axis=0) if len(right) > 0 else [1, 1]
    left_avg = np.average(left, axis=0) if len(left) > 0 else [-1, 1]
    # Create lines based on calculated averages
    left_line = make_points(image, left_avg)
    right_line = make_points(image, right_avg)
    return np.array([left_line, right_line])

def make_points(image, average):
    slope, y_int = average
    y1 = image.shape[0]
    y2 = int(y1 * (3 / 5))  # 3/5 the size of the image
    x1 = int((y1 - y_int) // slope)
    x2 = int((y2 - y_int) // slope)
    return np.array([x1, y1, x2, y2])

# Open the video file
cap = cv2.VideoCapture(video_path)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Preprocess the frame
    copy = np.copy(frame)
    gray_image = grey(copy)
    blur_image = gauss(gray_image)
    edges = canny(blur_image)
    isolated = region(edges)

    # Detect lines and draw them
    lines = cv2.HoughLinesP(isolated, 2, np.pi / 180, 100, np.array([]), minLineLength=40, maxLineGap=5)
    averaged_lines = average(copy, lines)
    black_lines = display_lines(copy, averaged_lines)

    # Combine the original frame with the detected lane lines
    lanes = cv2.addWeighted(copy, 0.8, black_lines, 1, 1)

    # Display the frame with lane lines
    cv2.imshow("Lane Detection", lanes)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Release the video capture object and close display windows
cap.release()
cv2.destroyAllWindows()
