#!/usr/bin/env python3

#Libraries IMU MPU-9250
import os
import sys
import time
import smbus
from imusensor.MPU9250 import MPU9250

#Libraries ROS 
import rospy 
from sensor_msgs.msg import Imu
#from geometry_msgs.msg import Vector3,Quaternion

def talker():

	address = 0x68 # declaration the address the sensor
	bus = smbus.SMBus(1)  # declaration the bus the sensor
	imu = MPU9250.MPU9250(bus, address) # create a object MPU9250
	imu.begin() # start 

	rospy.init_node('imu_node')
	pub = rospy.Publisher('imu_sub', Imu, queue_size = 10)

	msg = Imu()

	while not rospy.is_shutdown():

		imu.readSensor() # read the data of sensor
		imu.computeOrientation() # The roll, pitch and yaw can be accessed by imu.roll, imu.pitch and imu.yaw

		q1 = imu.roll 
		q2 = imu.pitch  
		q3 = imu.yaw 		

		# Angulos de la imu
		msg.linear_acceleration.x = q1 
		msg.linear_acceleration.y = q2
		msg.linear_acceleration.z = q3

		# velocidad de la imu
		msg.angular_velocity.x = imu.GyroVals[0]
		msg.angular_velocity.y = imu.GyroVals[1]
		msg.angular_velocity.z = imu.GyroVals[2]

		rate = rospy.Rate(7.5)
		pub.publish(msg)

		rate.sleep()

if __name__ == '__main__':    

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
        