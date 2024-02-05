#!/usr/bin/python3

''' 
Code to detect and follow a color line  
    
Run the line_detection launch commands before this file 

Execute with python3 line_detection.py 

Complete this template and the template files color_image_line and velocity_line 
'''

import cv2
import numpy as np 
import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from color_image_line import show_image, get_color_range, detect_color, get_max_contour
from velocity_line import get_velocity


# Variables 
########################################################################
bridge = CvBridge()


# Image callback -> it is called when the topic receives information
################################################################
def image_callback(msg):
    
    rospy.loginfo("Image received")

    # Get image and publisher 
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    show_image(img, "Robot camera")

    # Get half width of the image 
    mid_width = img.shape[1] // 2

    # Create velocity publisher and variable of velocity
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel = Twist()

    # Do color detection 
    ##########################################

    # Get color range with get_color_range()
    lower_range, upper_range = get_color_range()

    # Get color mask using detect_color from color_image_line.py
    mask = detect_color(img, lower_range, upper_range)

    # Show mask 
    show_image(mask, "Color detection")

    # Get max contour, area and center using get_mask_contour from color_image_line.py
    contour, area, center = get_max_contour(mask)
    print("Maximum area: ", area)

    # If area is bigger than 0 
    if area > 0:
        # Draw contour and center of the maximum contour
        cv2.drawContours(img, [contour], -1, (0, 255, 0), 2)
        cv2.circle(img, center, 5, (0, 0, 255), -1)
        show_image(img, "Contour detection")

        # Get robot velocity using get_velocity from velocity_line.py
        vel = get_velocity(vel, center[0], mid_width)

    # Look for color spinning 
    else:
        print("Looking for color: spinning to the left")
        # Add angular spinning velocity
        vel.angular.z = 0.5

    # Publish velocity
    vel_pub.publish(vel)

# Init node and suscribe to image topic 
################################################################   
def main():

    # Create line_detection node
    rospy.init_node('line_detection', anonymous=True)

    # Suscribe to image topic and add callback + spin
    rospy.Subscriber('/camera/image', Image, image_callback)
    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    
    
    
    