#! /usr/bin/python3
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np


vel_pub = rospy.Publisher(
    '/probot_anno/arm_vel_controller/command', Float64MultiArray, queue_size=10)
rospy.init_node('VELcontroller_py', anonymous=True)
zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

v = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

rospy.loginfo("vel= ")
rospy.loginfo(v)
vel = Float64MultiArray()
vel.data = zero
rate = rospy.Rate(10)
for i in range(10):
    vel_pub.publish(vel)
    rate.sleep()
vel.data = v
vel_pub.publish(vel)
wait = rospy.Rate(1000)
# wait.sleep()
# vel.data = zero
# vel_pub.publish(vel)

rospy.loginfo("py_sentinfo")
rospy.spin()
