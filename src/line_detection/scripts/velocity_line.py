#!/usr/bin/python3


'''
FUNCTIONS FOR LINE DETECTION CODE
'''

# Gets the robot speed and direction depending on the color detection 
##############################################
# Parameters:                                #
# - vel: Twist vector to modify velocity     #
# - x: x coordinate of the contour center    #
# - mid_width: middle width of the image     # 
##############################################
def get_velocity(vel, x, mid_width): 

    # Add an offset so the robot only turns when the center of the contour has deviated a specific distance to the right or left
    offset = 25

    print(x,mid_width)

    # If the color detected is on the right part of the image + an offset
    if x > mid_width + offset:
        # Go straight and spin to the right 
        print("Go straight and spin to the right")
        vel.linear.x = 0.1
        vel.angular.z = -0.5
         

    # If the color detected is on the left part of the image + an offset
    elif x<mid_width - offset:
        # Go straight and spin to the left
        print("Go straight and spin to the left")
        vel. linear.x = 0.1
        vel.angular.z = 0.5 


    # Go straight 
    else:
        print("Go straight ")
        vel. linear.x = 0.1
        vel.angular.z = 0.0

    
    return vel
    
