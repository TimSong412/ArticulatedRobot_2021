#! /usr/bin/python3
import rospy
import numpy as np
import trajectory_planning
from std_msgs.msg import Float32MultiArray


def main():
    # posC = pos_calc.Pos_Calc()
    trajP = trajectory_planning.trajectory_planner()
    # rate = rospy.Rate(10)

    pos_pub = rospy.Publisher(
        '/position_chatter', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)
    pos0 = [0, 0, 0, 0, 0, 0]
    pos1 = [29.98163936884934, -58.480989079538055, -3.263688667637193, 0, 0, 0]
    pos2 = [0.0, -49.62886723910115, 15.101453644190375, 0, 0, 0]
    # pos3 = [-40.60129464500447, -96.95427059284668,
    #         46.401990235507434, 0, 0, 0]
    pos1 = [i*np.pi/180 for i in pos1]
    pos2 = [i*np.pi/180 for i in pos2]
    # pos3 = [i*np.pi/180 for i in pos3]
    # print("pos3= ", pos3)
    trajP.set_init_end(pos0, pos1)
    trans01 = trajP.trajectory_generate(5)
    trajP.set_init_end(pos1, pos2)
    trans12 = trajP.trajectory_generate(5)
    # trajP.set_init_end(pos2, pos3)
    # trans23 = trajP.trajectory_generate(5)
    # trajP.set_init_end(pos3, pos0)
    # trans30 = trajP.trajectory_generate(5)
    trajP.set_init_end(pos2, pos0)
    trans20 = trajP.trajectory_generate(5)

    data01 = Float32MultiArray()
    data01.data = trans01
    data12 = Float32MultiArray()
    data12.data = trans12
    # data23 = Float32MultiArray()
    # data23.data = trans23
    # data30 = Float32MultiArray()
    # data30.data = trans30
    data20 = Float32MultiArray()
    data20.data = trans20
    while True:
        pos_pub.publish(data01)
        rate.sleep()
        pos_pub.publish(data12)
        rate.sleep()
        # pos_pub.publish(data23)
        # rate.sleep()
        # pos_pub.publish(data30)
        # rate.sleep()
        pos_pub.publish(data20)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('POScontroller', anonymous=True)
    main()
