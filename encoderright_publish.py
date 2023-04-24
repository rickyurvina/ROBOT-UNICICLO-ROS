#!/usr/bin/env python3
from RPi import GPIO

#import math
#from math import sin, cos, pi

import rospy
#import tf
#from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
#from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# Wheel right pinouts
clk_right = 5 
dt_right  = 6

# Configuration pins GPIO wheel right
GPIO.setmode(GPIO.BCM)

# Wheel right
GPIO.setup(clk_right, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dt_right, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

r = 5.90 # radius of wheel from center in cm
pi = 3.14
n = 57 # number of steps for one rotation
distance = 0 # distance 
L = 40.1 # Distancia entre las dos ruedas en cm
#rpm_to_radians = 0.10471975512
#rad_to_deg = 57.29578

def init_node():
    rospy.init_node('encoder_publisher')

    odom_pub = rospy.Publisher('encoder_node_right', Float32, queue_size=10)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    counter_right = 0

    clkLastState_right = GPIO.input(clk_right)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        #current_time = rospy.Time.now()

        clkState_right = GPIO.input(clk_right)
        dtState_right  = GPIO.input(dt_right)

        if clkState_right != clkLastState_right:
            if dtState_right != clkState_right:

                counter_right  += 1
            else:

                counter_right -= 1


            distance_right = ((2*pi*r)/n) * counter_right
          
            print(distance_right)

            odom_pub.publish(distance_right)

            rate.sleep()
        clkLastState_right = clkState_right

        #clkLastState_left = clkState_left

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            init_node()
    except rospy.ROSInterruptException:
        pass