#!/usr/bin/env python3
# import libraries
import RPi.GPIO as GPIO
import time

import rospy
from std_msgs.msg import Float32


global starttime
global endtime


# GPIO Pins para los ultrasonicos
GPIO_SIG_1 = 17

# GPIO Modus (BOARD / BCM)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

duration1 = 0

def initialize_node():

    rospy.init_node('ultrasonic_pub_1')
    pub = rospy.Publisher('ultrasonic_node_1', Float32, queue_size=10)
    #rospy.loginfo('Nodo iniciado')
    msg = Float32()

    #Configuracion de los pines de salida con un valor de 0
    GPIO.setup(GPIO_SIG_1, GPIO.OUT)
    GPIO.output(GPIO_SIG_1, 0)
    time.sleep(0.000002)
    #send trigger signal
    GPIO.output(GPIO_SIG_1, 1)
    time.sleep(0.000005)
    GPIO.output(GPIO_SIG_1, 0)
    GPIO.setup(GPIO_SIG_1, GPIO.IN)
    while GPIO.input(GPIO_SIG_1) == 0:
        #print(GPIO.input(GPIO_SIG_1))
        starttime = time.time()
        #print("error2")
    while GPIO.input(GPIO_SIG_1) == 1:
        #print(GPIO.input(GPIO_SIG_1))
        endtime = time.time()
    duration1 = endtime - starttime 
    #print(duration1)

    while True:
       
        distance1 = (duration1*34000)/2.0
        msg = distance1
        pub.publish(msg)
        rate = rospy.Rate(10)
        return distance1

 
if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            initialize_node()
            rospy.sleep(0.35)
    except rospy.ROSInterruptException:
        #print("error")
        pass
        