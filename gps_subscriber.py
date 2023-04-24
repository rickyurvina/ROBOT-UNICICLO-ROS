#!/usr/bin/env python3


import io
import pynmea2

import rospy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

#Declaration of variables
latidude = 0
longitude = 0
velocity = 0
azimuth = 0

def callback_altitude(msg: Quaternion):
    global latidude
    latidude = msg.x
    if(latidude > 0.0):
        rospy.loginfo('Latitude: ' + str(latidude))
def callback_longitude(msg: Quaternion):
    global longitude
    longitude = msg.y
    if(longitude > 0.0):
        rospy.loginfo("longitude: " +str(longitude))
def callback_vel(msg: Quaternion):
    global velocity
    velocity = msg.z
    if(velocity > 0.0):
        rospy.loginfo("velocity: " +str(velocity))

def callback_azimuth(msg: Quaternion):
    global azimuth
    azimuth = msg.w
    if(azimuth > 0.0):
        rospy.loginfo("azimuth: " +str(azimuth))

def setup_node():
    rospy.init_node('gps_subscriber')
    sub = rospy.Subscriber('gps_node',Quaternion,callback_altitude)
    sub = rospy.Subscriber('gps_node',Quaternion,callback_longitude)
    sub = rospy.Subscriber('gps_node',Quaternion,callback_vel)
    sub = rospy.Subscriber('gps_node',Quaternion,callback_azimuth)

    rospy.loginfo("x = latitud, y = longitud , z = velocidad , w = azimuth")
    #rospy.loginfo("altitud" + str(latidude) + "longitud"+ str(longitude) + "velocidad" + str(velocity) + "azimuth" + str(az))
    #rospy.spin()

if __name__ == '__main__':
    try:
        setup_node()
    except rospy.ROSInterruptException:
        pass
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #if(latidude > 0.0 and longitude > 0.0 and velocity > 0.0 and azimuth > 0.0):
        #rospy.loginfo("latitud: " + str(latidude) + "longitud: "+ str(longitude) + "velocidad: " + str(velocity) + "azimuth: " + str(azimuth))
        rospy.spin()

	
