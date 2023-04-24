#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

def callback_imu(msg: Imu):
	rospy.loginfo(msg.linear_acceleration) #imprime los angulos de la imu
	rospy.loginfo(msg.angular_velocity) #imprime la velocidad de la imu
	#rospy.loginfo(msg.orientation_covariance[1])
	#rospy.loginfo(msg.orientation_covariance[2])
def imu_subs():

	rospy.init_node('imu_subscriber')
	rospy.Subscriber('imu_sub', Imu, callback_imu)
	rospy.loginfo('Nodo iniciado')
	rospy.spin()


if __name__ == '__main__':
	try:
		imu_subs()
	except rospy.ROSInterruptException:
		pass