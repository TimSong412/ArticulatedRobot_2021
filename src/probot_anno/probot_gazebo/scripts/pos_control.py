#! /usr/bin/python3
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np


vel_pub = rospy.Publisher(
    '/probot_anno/arm_pos_controller/command', Float64MultiArray, queue_size=10)
rospy.init_node('POScontroller_py', anonymous=True)
zero = [0, 0, 0, 0, 0, 0]
# p = [-20.000000000000014, 24.664197194270606, -66.97750144851543, -14.281670103895822, 103.06225558932488, 70.1236007288458]
p = [20, 20, 20, 20, 20, 20]
# p = [-45, 60, -45, 60, -45, 60]
rad = [i*np.pi / 180.0 for i in p]
rospy.loginfo("deg= ")
rospy.loginfo(p)
rospy.loginfo("rad= ")
rospy.loginfo(rad)
pos = Float64MultiArray()
pos.data = zero
rate = rospy.Rate(10)
for i in range(5):
    vel_pub.publish(pos)
    rate.sleep()

for i in range(5):
    rate.sleep()

pos.data = rad
vel_pub.publish(pos)
rate.sleep()
for i in range(5):
    rate.sleep()

p = [-45, 60, -45, 60, -45, 60]
rad = [i*np.pi / 180.0 for i in p]
pos.data = rad
vel_pub.publish(pos)
rate.sleep()
 
# while not rospy.is_shutdown():
#     vel_pub.publish(pos)
#     rate.sleep()

rospy.loginfo("py_sentinfo")
rospy.spin()
