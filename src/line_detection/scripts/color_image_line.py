#!/usr/bin/python3


import cv2



# Show image 
def show_image(img, window_name): 
    cv2.imshow(window_name, img)
    cv2.waitKey(1)


# Get red color limits
def get_color_range():
    lower_range = (0, 100, 100)
    upper_range = (10, 255, 255)
    
    return lower_range, upper_range


# Detects the color 
def detect_color(img, lower_range, upper_range):

    # Perform a Gaussian filter 
    img_gaussian = cv2.GaussianBlur(img, (5, 5), 0)

    # Convert image to HSV
    img_hsv = cv2.cvtColor(img_gaussian, cv2.COLOR_BGR2HSV)

    # Get color mask
    mask = cv2.inRange(img_hsv, lower_range, upper_range)

    # Define rectangular kernel 5x5
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    # Apply openning
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # Apply dilation
    mask = cv2.dilate(mask, kernel, iterations=2)

    return mask


# Get maximum contour, area and its center 
def get_max_contour(mask): 

    contour_max = []
    area_max = 0
    center = (-1,-1)
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    # For each contour
    for contour in contours:

        # Get area of the contour 
        area = cv2.contourArea(contour)


        # If area is bigger than area_max
        if area > area_max:
            # Update area_max value 
            area_max = area

            # Update contour_max value
            contour_max = contour

            # Get center of the contour using cv2.moments
            M = cv2.moments(contour)
            center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))


    return contour_max, area_max, center