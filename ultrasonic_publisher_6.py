#!/usr/bin/env python3

# import libraries
import RPi.GPIO as GPIO
import time

import rospy
from std_msgs.msg import Float32

global starttime
global endtime

# GPIO Pins para los ultrasonicos

GPIO_SIG_6  = 21

# GPIO Modus (BOARD / BCM)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

duration6 = 0

def initialize_node():

    rospy.init_node('ultrasonic_pub_6')
    pub = rospy.Publisher('ultrasonic_node', Float32, queue_size=10)
    #rospy.loginfo('Nodo iniciado')
    msg = Float32()

    # Configuracion de los pines de salida con un valor de 0

    GPIO.setup(GPIO_SIG_6, GPIO.OUT)
    GPIO.output(GPIO_SIG_6, 0)
    time.sleep(0.000002)
    #send trigger signal
    GPIO.output(GPIO_SIG_6, 1)
    time.sleep(0.000005)
    GPIO.output(GPIO_SIG_6, 0)
    GPIO.setup(GPIO_SIG_6, GPIO.IN)
    while GPIO.input(GPIO_SIG_6) == 0:
        starttime = time.time()
    while GPIO.input(GPIO_SIG_6) == 1:
        endtime = time.time()
    duration6 = endtime - starttime


    while True:
       
        distance6 = (duration6*34000)/2.0

        #print(distance1,distance2,distance3,distance4,distance5,distance6)
        msg = distance6
        pub.publish(msg)
        rate = rospy.Rate(10)
        return distance6

 
if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            initialize_node()
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass