#!/usr/bin/env python3

# import libraries
import RPi.GPIO as GPIO
import time

import rospy
from std_msgs.msg import Float32

distance1 = 0
distance2 = 0
distance3 = 0
distance4 = 0
distance5 = 0
distance6 = 0


def callback_ultrasonic_1(msg: Float32):
    global distance1
    distance1 = msg.data
    #rospy.loginfo(distance1)
def callback_ultrasonic_2(msg: Float32):
    global distance2
    distance2 = msg.data
def callback_ultrasonic_3(msg: Float32):
    global distance3
    distance3 = msg.data
def callback_ultrasonic_4(msg: Float32):
    global distance4
    distance4 = msg.data
def callback_ultrasonic_5(msg: Float32):
    global distance5
    distance5 = msg.data
def callback_ultrasonic_6(msg: Float32):
    global distance6
    distance6 = msg.data



def initialize_node():
    rospy.init_node('ultrasonic_sub')
    sub = rospy.Subscriber('ultrasonic_node_1', Float32, callback_ultrasonic_1)
    sub = rospy.Subscriber('ultrasonic_node_2', Float32, callback_ultrasonic_2)
    sub = rospy.Subscriber('ultrasonic_node_3', Float32, callback_ultrasonic_3)
    sub = rospy.Subscriber('ultrasonic_node_4', Float32, callback_ultrasonic_4)
    sub = rospy.Subscriber('ultrasonic_node_5', Float32, callback_ultrasonic_5)
    sub = rospy.Subscriber('ultrasonic_node_6', Float32, callback_ultrasonic_6)
    rospy.loginfo('Nodo iniciado')

    #print(distance1,distance2,distance3,distance4,distance5,distance6)

    #rospy.spin()


if __name__ == '__main__':
    try:
        initialize_node()
    except rospy.ROSInterruptException:
        pass
    rate = rospy.Rate(6)
    while not rospy.is_shutdown():
        print(distance1,distance2,distance3,distance4,distance5,distance6)

        rate.sleep()