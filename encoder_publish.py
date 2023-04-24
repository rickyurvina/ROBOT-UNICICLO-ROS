#!/usr/bin/env python3
from RPi import GPIO

#import math
#from math import sin, cos, pi

import rospy
#import tf
#from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
#from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# Wheel left pinouts
clk_left = 24
dt_left  = 25


# Configuration pins GPIO wheel left
GPIO.setmode(GPIO.BCM)
# Wheel left
GPIO.setup(clk_left, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dt_left, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

r = 5.90 # radius of wheel from center in cm
pi = 3.14
n = 57 # number of steps for one rotation
distance = 0 # distance 
#L = 40.1 # Distancia entre las dos ruedas en cm

def init_node():
    rospy.init_node('encoder_publisher')

    odom_pub = rospy.Publisher('encoder_node_left', Float32, queue_size=10)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    counter_left = 0

    clkLastState_left = GPIO.input(clk_left)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        #current_time = rospy.Time.now()

        clkState_left = GPIO.input(clk_left)
        dtState_left  = GPIO.input(dt_left)
    
        if clkState_left != clkLastState_left: 
            if dtState_left != clkState_left:

                counter_left   += 1
                
            else:
                counter_left  -= 1

            distance_left  = ((2*pi*r)/n) * counter_left
     

            odom_pub.publish(distance_left)

            rate.sleep()
        clkLastState_left = clkState_left

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            init_node()
    except rospy.ROSInterruptException:
        pass