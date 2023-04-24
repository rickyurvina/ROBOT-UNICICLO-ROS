#!/usr/bin/env python3

# import libraries
import RPi.GPIO as GPIO
import time

import rospy
from std_msgs.msg import Float32


global starttime
global endtime


# GPIO Pins para los ultrasonicos
GPIO_SIG_3  = 22

# GPIO Modus (BOARD / BCM)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

duration3 = 0

def initialize_node():

    rospy.init_node('ultrasonic_pub_3')
    pub = rospy.Publisher('ultrasonic_node', Float32, queue_size=10)
    #rospy.loginfo('Nodo iniciado')
    msg = Float32()

    # Configuracion de los pines de salida con un valor de 0

     GPIO.setup(GPIO_SIG_3, GPIO.OUT)
     GPIO.output(GPIO_SIG_3, 0)
     time.sleep(0.000002)
     #send trigger signal
     GPIO.output(GPIO_SIG_3, 1)
     time.sleep(0.000005)
     GPIO.output(GPIO_SIG_3, 0)
     GPIO.setup(GPIO_SIG_3, GPIO.IN)
     while GPIO.input(GPIO_SIG_3) == 0:
         starttime = time.time()
     while GPIO.input(GPIO_SIG_3) == 1:
         endtime = time.time()
     duration3 = endtime - starttime

    while True:

        distance3 = (duration3*34000)/2.0
    
        #print(distance1,distance2,distance3,distance4,distance5,distance6)
        msg = distance3
        pub.publish(msg)
        rate = rospy.Rate(10)
        return distance3

 
if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            initialize_node()
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass