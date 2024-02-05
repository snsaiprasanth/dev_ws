#!/usr/bin/python3
'''
FUNCTIONS FOR COLOR DETECTION CODE
'''
import cv2
import numpy as np
# Resize and show image
def show_image(img, window_name):
    img_res = cv2.resize(img, None, fx=0.3, fy=0.3)
    cv2.imshow(window_name, img_res)
    cv2.waitKey(1)
# Get color limits
def get_color_range(color):
    # Complete only for the color you want to detect
    if(color == 'blue'):
        lower_range = np.array([100,50,50])
        upper_range = np.array([140,255,255])
    #elif(color == 'green'):
    #    lower_range = # <COMPLETE>
    #    upper_range = # <COMPLETE>
    #elif(color == 'blue'):
    #    lower_range = # <COMPLETE>
    #    upper_range = # <COMPLETE>
    #else: # Yellow
    #    lower_range = # <COMPLETE>
    #    upper_range = # <COMPLETE>
    return lower_range, upper_range
# Detects the color
def detect_color(img, lower_range, upper_range):
    # Perform a Gaussian filter
    image_gauss = cv2.GaussianBlur(img, (5,5), 0)
    # Convert gauss image to HSV
    hsv_image = cv2.cvtColor(image_gauss, cv2.COLOR_BGR2HSV)
    # Get color mask
    mask = cv2.inRange(hsv_image, lower_range, upper_range)
    # Define rectangular kernel 25x25
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25,25))
    # Apply openning to mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask
# Get maximum contour, area and its center
def get_max_contour(mask):
    contour_max = []
    area_max = 10
    center = (-1,-1)
    # Find contours
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # For each contour
    for cnt in contours:
        # Get area of the contour
        area = cv2.contourArea(cnt)
        # If area is bigger than area_max
        if(area > area_max):
            # Update area max value
            area_max = area
            # Update contour_max value
            contour_max = cnt
            # Get center of the contour using cv2.moments
            M = cv2.moments(cnt)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            center = (cx, cy)
    return contour_max, area_max, center