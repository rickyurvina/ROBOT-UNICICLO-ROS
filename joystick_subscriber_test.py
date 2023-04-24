#!/usr/bin/env python3
import rospy
from std_msgs.msg import String 
from sensor_msgs.msg import Joy
from time import sleep
from evdev import InputDevice, categorize, ecodes
import RPi.GPIO as GPIO

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Declaration of variables
forward_backward = 0

pwm_left = 12 # ping left for the controller in the Raspberry
pwm_right = 19  #ping right for the controller in the Raspberry
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) #set pin numbering system
GPIO.setup(pwm_left,GPIO.OUT)
GPIO.setup(pwm_right,GPIO.OUT)
pi_pwm_left = GPIO.PWM(pwm_left,60) #creation PWM left
pi_pwm_right = GPIO.PWM(pwm_right,60) #creation PWM right


def callback(msg: Twist):
    #rospy.loginfo(msg.x + " " + msg.y + " " + msg.x)
    rospy.loginfo('Call callback')


def joystic_callback(msg: Joy):
    #rospy.loginfo(msg.buttons[1]) # This code is for setting Button B
    rospy.loginfo(msg.axes[0])    # This code is for reading Axes Up/Dn

    #forward_backward = 3.5*msg.axes[3] + 8.5
    #left_right       = 3.5*msg.axes[0] + 8.5

    omega_speed_vleft  = msg.axes[3] - 0.3*(msg.axes[0])
    omega_speed_vright = msg.axes[3] + 0.3*(msg.axes[0])
    

    if msg.axes[0]>=0:
        rospy.loginfo("izq")
        #omega_speed_vleft  = omega_speed_vleft  
        #omega_speed_vright = omega_speed_vright - abs(msg.axes[0]) 
    
    if msg.axes[0]<0:
        rospy.loginfo("der")
        #omega_speed_vleft  = omega_speed_vleft - abs(msg.axes[0])  
        #omega_speed_vright = omega_speed_vright 

    omega_speed_v = 3.5*omega_speed_vleft + 8.5
    omega_speed_w = 3.5*omega_speed_vright + 8.5
    
    rospy.loginfo("{:.2f}".format(omega_speed_v) + " " +"{:.2f}".format(omega_speed_w)) 
    #rospy.loginfo(msg.axes[3])

    #rospy.loginfo("{:.2f}".format(msg.axes[3]) + " " +"{:.2f}".format(msg.axes[0]) + " " +"{:.2f}".format(omega_speed_left) + " " + "{:.2f}".format(omega_speed_right)) 
    #rospy.loginfo(msg.axes[3])


    pi_pwm_left.ChangeDutyCycle(omega_speed_v)
    pi_pwm_right.ChangeDutyCycle(omega_speed_w) 


def pwm_setting():

    pi_pwm_right.start(0)
    pi_pwm_left.start(0)


def subscriber_sensors():

    rospy.init_node('joystick_subs')
    sub = rospy.Subscriber('joystick_node',Twist,callback)
    sub = rospy.Subscriber("/joy", Joy , joystic_callback)
    pwm_setting()

    # rospy.spin()

if __name__ == '__main__':    
    #start_node()
    #rospy.loginfo('Inicia suscriber')

    try:
        pwm_setting()
        subscriber_sensors()
    except rospy.ROSInterruptException:
        pass

    rate = rospy.Rate(20) # 20hz
  
    while not rospy.is_shutdown():

        rate.sleep()