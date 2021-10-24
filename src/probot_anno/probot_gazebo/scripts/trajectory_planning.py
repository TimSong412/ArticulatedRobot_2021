# -*- coding: utf-8 -*-
import math
import numpy as np
import time
# FOR REAL ROBOT


class trajectory_planner:
    def __init__(self):
        # About time step and total time
        self.total_time = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
        # About theta
        self.theta_init = [0, 0, 0, 0, 0, 0]
        self.theta_end = [0, 0, 0, 0, 0, 0]
        # About joints max_speed and max_acc
        self.max_speed = [0.7854, 0.6981, 0.7854, 0.9599, 0.9599, 0.9599]
        self.max_acc = [0.3491, 0.2618, 0.3491, 0.4363, 0.4363, 0.4363]
        self.p1 = self.cal_parameter(0)
        self.p2 = self.cal_parameter(1)
        self.p3 = self.cal_parameter(2)
        self.p4 = self.cal_parameter(3)
        self.p5 = self.cal_parameter(4)
        self.p6 = self.cal_parameter(5)
        self.p = [self.p1, self.p2, self.p3, self.p4, self.p5, self.p6]

    def set_init_end(self, theta0, theta1):
        self.theta_init = theta0
        self.theta_end = theta1
        self.p1 = self.cal_parameter(0)
        self.p2 = self.cal_parameter(1)
        self.p3 = self.cal_parameter(2)
        self.p4 = self.cal_parameter(3)
        self.p5 = self.cal_parameter(4)
        self.p6 = self.cal_parameter(5)
        self.p = [self.p1, self.p2, self.p3, self.p4, self.p5, self.p6]

    def trajectory_generate(self, t):
        theta = []
        for j in range(6):
            parameter = self.p[j]
            temp = self.cal_theta(t, parameter)
            theta.append(temp)
        # transform speed
        theta[0] = theta[0]*30*180/np.pi
        theta[1] = theta[1]*205*180/(3*np.pi)
        theta[2] = theta[2]*50*180/np.pi
        theta[3] = theta[3]*125*180/(2*np.pi)
        theta[4] = theta[4]*125*180/(2*np.pi)
        theta[5] = theta[5]*200*180/(9*np.pi)
        return theta

    def cal_parameter(self, joint_num):
        # calculate time segment
        theta_0 = self.theta_init[joint_num]
        theta_1 = self.theta_end[joint_num]
        T = self.total_time[joint_num]

        max_acc = self.max_acc[joint_num]
        max_speed = self.max_speed[joint_num]

        # positive or negative
        if theta_1 < theta_0:
            max_acc = -max_acc
            max_speed = -max_speed
        t1 = T/2 - math.sqrt(T*T/4.0 - (theta_1-theta_0)/max_acc)
        t2 = T - t1

        parameter = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        if abs(t1*max_acc) > max_speed:
            print("Joint %d out of speed range", joint_num)
        else:
            # function segment 1
            a02 = max_acc/2
            a01 = 0
            a00 = theta_0
            # function segment 2
            a11 = max_acc*t1
            a10 = -max_acc/2*t1*t1+theta_0
            # function segment 3
            a22 = -max_acc/2
            a21 = max_acc*T
            a20 = -max_acc/2*T*T+theta_1
            parameter = [t1, t2, a02, a01, a00, a11, a10, a22, a21, a20]

        return parameter

    def cal_theta(self, t, parameter):
        ret = 0
        t1 = parameter[0]
        t2 = parameter[1]
        a02 = parameter[2]
        a01 = parameter[3]
        a00 = parameter[4]
        a11 = parameter[5]
        a10 = parameter[6]
        a22 = parameter[7]
        a21 = parameter[8]
        a20 = parameter[9]

        if t <= t1:
            ret = a02*t*t + a01*t + a00
        if t > t1 and t <= t2:
            ret = a11*t + a10
        if t > t2:
            ret = a22*t*t + a21*t + a20

        return ret


def main():
    cal = trajectory_planner([0, 0, 0, 0, 0, 0], [1, 1, 1, 1, 1, 1])
    st = time.time()
    res = cal.trajectory_generate(5)
    ed = time.time()
    print(res)
    print("time= ", ed-st)


if __name__ == '__main__':
    main()
