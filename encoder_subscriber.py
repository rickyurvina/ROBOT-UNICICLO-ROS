#!/usr/bin/env python3
from RPi import GPIO
from time import sleep
import rospy
#from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

left_ticks  = 0
right_ticks = 0


def leftTicksCallback(msg):
    global left_ticks 
    left_ticks = msg.data
    #rospy.loginfo(msg.data)

def rightTicksCallback(msg):
    global right_ticks 
    right_ticks = msg.data

def init_node():
        rospy.init_node('encoder_subscriber')
        left_ticks_sub  = rospy.Subscriber('encoder_node_left', Float32, leftTicksCallback)
        right_ticks_sub = rospy.Subscriber('encoder_node_right', Float32, rightTicksCallback)
        rospy.loginfo("Distancia de la rueda izquierda: " +str(left_ticks))
        rospy.loginfo("Distancia de la rueda derecha: "   +str(right_ticks))
        rospy.loginfo("Distancia total :" +str((left_ticks+right_ticks) / 2))
        #rospy.spin()


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            init_node()
    except rospy.ROSInterruptException:
        pass