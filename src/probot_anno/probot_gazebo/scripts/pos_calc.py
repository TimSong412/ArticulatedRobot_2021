import numpy as np
import time


class Pos_Calc:
    def __init__(self, l1=284, l2=225, l3=228.9, l4=55, t1_ini=0, t2_ini=90, t3_ini=0, t4_ini=0, t5_ini=0, t6_ini=0):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        self.acc = 16
        self.theta1_init = t1_ini * np.pi / 180
        self.theta2_init = t2_ini * np.pi / 180
        self.theta3_init = t3_ini * np.pi / 180
        self.theta4_init = t4_ini * np.pi / 180
        self.theta5_init = t5_ini * np.pi / 180
        self.theta6_init = t6_ini * np.pi / 180
        pass

    def xyz_trans(self, dx, dy, dz, rx, ry, rz):
        # 转换末端坐标到wrist坐标 输出单位为mm
        crx = np.cos(rx)
        srx = np.sin(rx)
        cry = np.cos(ry)
        sry = np.sin(ry)
        crz = np.cos(rz)
        srz = np.sin(rz)
        Q = np.array([[cry*crz, srx*sry*crz - crx*srz, srx*srz + crx*sry*crz],
                      [cry*srz, crx*crz + srx*sry*srz, crx*sry*srz - crz*srx],
                      [-sry, cry*srx, crx*cry]])

        l_v = np.array([[0], [0], [-self.l4]])
        print("Q = ", Q)
        res1 = Q.dot(l_v)
        res2 = np.array([[dx], [dy], [dz]])
        res = res1/1000 + res2
        xx = res[0, 0] * 1000
        yy = res[1, 0] * 1000
        zz = res[2, 0] * 1000

        return xx, yy, zz

    def ret_rxryrz(self, T):
        R = T
        gamma = np.around(np.arctan2(R[1, 0], R[0, 0]), self.acc)
        beta = np.around(np.arctan2(-R[2, 0], np.around(np.cos(gamma) * R[0, 0] + np.sin(gamma) * R[1, 0], self.acc)),
                         self.acc)

        alpha = np.around(np.arctan2(
            np.around(np.sin(gamma)*R[0, 2]-np.cos(gamma)*R[1, 2], self.acc),
            np.around(-np.sin(gamma)*R[0, 1]+np.cos(gamma)*R[1, 1], self.acc)),
            self.acc)
        return alpha, beta, gamma

    def calc_angle(self, dx, dy, dz, rx=0, ry=0, rz=0):
        # 逆运动学
        dx, dy, dz = self.xyz_trans(dx, dy, dz, rx, ry, rz)
        print("dx= ", dx, " dy= ", dy, " dz= ", dz)

        theta1 = np.arctan2(dy, dx)
        c1 = np.cos(theta1)
        s1 = np.sin(theta1)

        a = np.around(-2*self.l2 * (dx*c1 + dy*s1), self.acc)
        b = np.around(2 * self.l2 * (self.l1 - dz), self.acc)
        c = np.around(self.l3**2 - ((dx*c1+dy*s1)
                                    ** 2 + self.l1**2 - 2*self.l1*dz + self.l2**2 + dz**2), self.acc)
        r = np.sqrt(a**2 + b**2)
        theta2 = -(np.arctan2(a, b) - np.arctan2(c, np.sqrt(r**2 - c**2)))

        c2 = np.cos(theta2)
        s2 = np.sin(theta2)

        theta3 = np.arctan2((dx*c1+dy*s1 - self.l2 *
                             c2), (self.l1 + self.l2*s2-dz)) - theta2

        c3 = np.cos(theta3)
        s3 = np.sin(theta3)

        R03 = np.around(np.array([[c1*c2*c3 - c1*s2*s3, s1, c1*c2*s3+c1*s2*c3],
                        [s1*c2*c3 - s1*s2*s3, -c1, s1*c2*s3+s1*s2*c3],
                        [s2*c3+s3*c2, 0, s2*s3-c2*c3]]),
                        self.acc)

        crx = np.around(np.cos(rx), self.acc)
        srx = np.around(np.sin(rx), self.acc)
        cry = np.around(np.cos(ry), self.acc)
        sry = np.around(np.sin(ry), self.acc)
        crz = np.around(np.cos(rz), self.acc)
        srz = np.around(np.sin(rz), self.acc)

        R06 = np.around(np.array([
                        [cry*crz, srx*sry*crz - crx*srz, srx*srz + crx*sry*crz],
                        [cry*srz, crx*crz + srx*sry*srz, crx*sry*srz - crz*srx],
                        [-sry, cry*srx, crx*cry]]),
                        self.acc)

        R36 = np.around(R03.T.dot(R06), self.acc)

        theta4 = np.arctan2(R36[1, 2], R36[0, 2])

        theta5 = np.around(np.arctan2(
            np.sqrt(R36[0, 2]**2 + R36[1, 2]**2), R36[2, 2]),
            self.acc)

        theta6 = np.arctan2(R36[2, 1], -R36[2, 0])

        res = (theta1 - self.theta1_init) * 180 / np.pi, (theta2 - self.theta2_init) * 180 / np.pi, (theta3 - self.theta3_init) * 180 / \
            np.pi, (theta4 - self.theta4_init) * 180 / np.pi, (theta5 -
                                                               self.theta5_init) * 180 / np.pi, (theta6 - self.theta6_init) * 180 / np.pi

        return res

    # input: deg
    def calc_pos(self, theta1, theta2, theta3, theta4, theta5, theta6):
        # 正运动学
        c1 = np.cos(theta1*np.pi/180+self.theta1_init)
        s1 = np.sin(theta1*np.pi/180+self.theta1_init)
        c2 = np.cos(theta2*np.pi/180+self.theta2_init)
        s2 = np.sin(theta2*np.pi/180+self.theta2_init)
        c3 = np.cos(theta3*np.pi/180+self.theta3_init)
        s3 = np.sin(theta3*np.pi/180+self.theta3_init)
        c4 = np.cos(theta4*np.pi/180+self.theta4_init)
        s4 = np.sin(theta4*np.pi/180+self.theta4_init)
        c5 = np.cos(theta5*np.pi/180+self.theta5_init)
        s5 = np.sin(theta5*np.pi/180+self.theta5_init)
        c6 = np.cos(theta6*np.pi/180+self.theta6_init)
        s6 = np.sin(theta6*np.pi/180+self.theta6_init)

        T03 = np.around(np.array([
            [c1*c2*c3 - c1*s2*s3, s1, c1*c2*s3+c1*s2*c3, self.l2*c1*c2],
            [s1*c2*c3 - s1*s2*s3, -c1, s1*c2*s3+s1*s2*c3, self.l2*c2*s1],
            [s2*c3+s3*c2, 0, s2*s3-c2*c3, self.l2*s2+self.l1],
            [0, 0, 0, 1]]),
            self.acc)

        T36 = np.around(np.array([
            [c4*c5*c6 - s4*s6, -c6*s4 - c4*c5*s6, c4*s5, self.l4*c4*s5],
            [c4*s6 + c5*c6*s4, c4*c6 - c5*s4*s6, s4*s5, self.l4*s4*s5],
            [-c6*s5, s5*s6, c5, self.l3+self.l4*c5],
            [0, 0, 0, 1]]),
            self.acc)

        T06 = np.around(T03.dot(T36), self.acc)
        print("T06= ")
        print(T06)
        a, b, c = self.ret_rxryrz(T06[0:3, 0:3])  # rpy角
        dx, dy, dz = T06[0:3, 3]/1000  # 坐标 输出单位为m
        return dx, dy, dz, a, b, c


def main():
    calculator = Pos_Calc()
    st = time.time()
    # print(calculator.calc_angle(0.12118647584078515, 0.037261586658715866, 0.6893272826835242, -0.4420150775729468, -0.3463237779149588, -2.068034976781502))
    print(calculator.calc_pos(20, 20, 20, 20, 20, 20))
    # print(calculator.calc_pos(0, 0, 0, 0, 0, 0))
    ed = time.time()
    print("time= ", ed-st)


if __name__ == '__main__':
    main()
