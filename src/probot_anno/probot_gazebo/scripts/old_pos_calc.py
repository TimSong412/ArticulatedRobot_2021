import numpy as np
import time
import threading


class Pos_Calc:
    def __init__(self, l1=284, l2=225, l3=228.9, l4=0, t1_ini=0, t2_ini=90, t3_ini=0, t4_ini=0, t5_ini=0, t6_ini=0):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        self.acc = 16
        self.Trans = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        self.theta1_init = t1_ini * np.pi / 180
        self.theta2_init = t2_ini * np.pi / 180
        self.theta3_init = t3_ini * np.pi / 180
        self.theta4_init = t4_ini * np.pi / 180
        self.theta5_init = t5_ini * np.pi / 180
        self.theta6_init = t6_ini * np.pi / 180
        pass

    def xyz_trans(self, dx, dy, dz, rx, ry, rz):
        crx = np.cos(rx)
        srx = np.sin(rx)
        cry = np.cos(ry)
        sry = np.sin(ry)
        crz = np.cos(rz)
        srz = np.sin(rz)
        Q = np.array([[cry*crz, srx*sry*crz - crx*srz, srx*srz + crx*sry*crz],
                      [cry*srz, crx*crz + srx*sry*srz, crx*sry*srz - crz*srx],
                      [-sry, cry*srx, crx*cry]])

        l_v = np.array([[-self.l4], [0], [0]])
        # res1 = self.Trans.dot(Q.dot(l_v))
        res1 = Q.dot(l_v)
        xyz = np.array([[dx], [dy], [dz]])
        # res2 = self.Trans.dot(xyz)
        res2 = xyz
        res = res1 + res2
        xx = res[0, 0] * 1000
        yy = res[1, 0] * 1000
        zz = res[2, 0] * 1000
        # xx = -dy * 1000 + res[0, 0]
        # yy = dx * 1000 + res[1, 0]
        # zz = dz * 1000 + res[2, 0]
        return xx, yy, zz

    def ret_xyz(self, t):
        # res = self.Trans.T.dot(t)
        res = t
        xx = res[0]/1000
        yy = res[1]/1000
        zz = res[2]/1000
        return xx, yy, zz

    def ret_rxryrz(self, T):
        # R = self.Trans.T.dot(T)
        R = T
        gamma = np.around(np.arctan2(R[1, 0], R[0, 0]), self.acc)
        beta = np.around(np.arctan2(-R[2, 0], np.around(np.cos(gamma) * R[0, 0] + np.sin(gamma) * R[1, 0], self.acc)),
                         self.acc)

        alpha = np.around(np.arctan2(
            np.around(np.sin(gamma)*R[0, 2]-np.cos(gamma)*R[1, 2], self.acc),
            np.around(-np.sin(gamma)*R[0, 1]+np.cos(gamma)*R[1, 1], self.acc)),
            self.acc)
        # print("a, b, c= ", alpha, beta, gamma)
        return alpha, beta, gamma

    def calc_angle(self, dx, dy, dz, rx=0, ry=0, rz=0):

        dx, dy, dz = self.xyz_trans(dx, dy, dz, rx, ry, rz)
        print("dx= ", dx, " dy= ", dy, " dz= ", dz)

        theta1 = np.arctan2(dy, dx)
        # theta1 = - np.pi / 2

        c1 = np.cos(theta1)
        s1 = np.sin(theta1)

        a = np.around(-2*self.l2 * (dx*c1 + dy*s1), self.acc)
        b = np.around(2 * self.l2 * (self.l1 - dz), self.acc)
        c = np.around(self.l3**2 - ((dx*c1+dy*s1)
                                    ** 2 + self.l1**2 - 2*self.l1*dz + self.l2**2 + dz**2), self.acc)
        r = np.sqrt(a**2 + b**2)
        # +- sqrt below
        theta2 = -(np.arctan2(a, b) - np.arctan2(c, np.sqrt(r**2 - c**2)))
        # theta2 = - np.pi / 2

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

        # R06_new = np.around(self.Trans.dot(R06), self.acc)

        # R36 = np.around(R03.T.dot(R06_new), self.acc)
        R36 = np.around(R03.T.dot(R06), self.acc)

        theta4 = np.arctan2(R36[1, 2], R36[0, 2])
        # theta4 = 0
        theta5 = np.around(np.arctan2(
            np.sqrt(R36[0, 2]**2 + R36[1, 2]**2), R36[2, 2]),
            self.acc)
        # theta5 = 0
        theta6 = np.arctan2(R36[2, 1], -R36[2, 0])

        res = (theta1 - self.theta1_init) * 180 / np.pi, (theta2 - self.theta2_init) * 180 / np.pi, (theta3 - self.theta3_init) * 180 / \
            np.pi, (theta4 - self.theta4_init) * 180 / np.pi, (theta5 -
                                                               self.theta5_init) * 180 / np.pi, (theta6 - self.theta6_init) * 180 / np.pi

        return res

    # input: deg
    def calc_pos(self, theta1, theta2, theta3, theta4, theta5, theta6):
        st = time.time()
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
        # ed = time.time()
        # print("c1= ", c1, " c2= ", c2, " c3= ", c3)
        # print("s1= ", s1, " s2= ", s2, " s3= ", s3)
        # st = time.time()
        # T03 = np.array([
        #     [c1*c2*c3 - c1*s2*s3, s1, c1*c2*s3+c1*s2*c3, self.l2*c1*c2],
        #     [s1*c2*c3 - s1*s2*s3, -c1, s1*c2*s3+s1*s2*c3, self.l2*c2*s1],
        #     [s2*c3+s3*c2, 0, s2*s3-c2*c3, self.l2*s2+self.l1],
        #     [0, 0, 0, 1]])

        # # ed = time.time()
        # T36 = np.array([
        #     [c4*c5*c6 - s4*s6, -c6*s4 - c4*c5*s6, c4*s5, self.l4*c4*s5],
        #     [c4*s6 + c5*c6*s4, c4*c6 - c5*s4*s6, s4*s5, self.l4*s4*s5],
        #     [-c6*s5, s5*s6, c5, self.l3+self.l4*c5],
        #     [0, 0, 0, 1]])

        # T06 = np.around(T03.dot(T36), self.acc)
        T06 = np.array([[s1*(c4*s6 + c5*c6*s4) - (s4*s6 - c4*c5*c6)*(c1*c2*c3 - c1*s2*s3) - c6*s5*(c1*c2*s3 + c1*c3*s2), s1*(c4*c6 - c5*s4*s6) - (c6*s4 + c4*c5*s6)*(c1*c2*c3 - c1*s2*s3) + s5*s6*(c1*c2*s3 + c1*c3*s2), c5*(c1*c2*s3 + c1*c3*s2) + s1*s4*s5 + c4*s5*(c1*c2*c3 - c1*s2*s3), (c1*c2*s3 + c1*c3*s2)*(self.l3 + c5*self.l4) + c1*c2*self.l2 + self.l4*s1*s4*s5 + c4*self.l4*s5*(c1*c2*c3 - c1*s2*s3)],
                        [- (s4*s6 - c4*c5*c6)*(c2*c3*s1 - s1*s2*s3) - c1*(c4*s6 + c5*c6*s4) - c6*s5*(c2*s1*s3 + c3*s1*s2), s5*s6*(c2*s1*s3 + c3*s1*s2) - c1*(c4*c6 - c5*s4*s6) - (c6*s4 + c4*c5*s6)*(c2*c3*s1 - s1*s2*s3),
                            c5*(c2*s1*s3 + c3*s1*s2) - c1*s4*s5 + c4*s5*(c2*c3*s1 - s1*s2*s3), (c2*s1*s3 + c3*s1*s2)*(self.l3 + c5*self.l4) + c2*self.l2*s1 - c1*self.l4*s4*s5 + c4*self.l4*s5*(c2*c3*s1 - s1*s2*s3)],
                        [c6*s5*(c2*c3 - s2*s3) - (s4*s6 - c4*c5*c6)*(c2*s3 + c3*s2),                                   - (c6*s4 + c4*c5*s6)*(c2*s3 + c3*s2) - s5*s6*(c2*c3 - s2*s3),
                         c4*s5*(c2*s3 + c3*s2) - c5*(c2*c3 - s2*s3),                         self.l1 - (c2*c3 - s2*s3)*(self.l3 + c5*self.l4) + self.l2*s2 + c4*self.l4*s5*(c2*s3 + c3*s2)],
                        [0,                                                                                              0,                                                                 0,                                                                                            1]])
        ed = time.time()
        print("ptime= ", ed-st)
        st = time.time()
        a, b, c = self.ret_rxryrz(T06[0:3, 0:3])
        dx, dy, dz = self.ret_xyz(T06[0:3, 3])
        ed = time.time()
        print("etime= ", ed-st)
        return dx, dy, dz, a, b, c


def main():
    # arctan2(y, x)
    # cos(rad)
    calculator = Pos_Calc()
    st = time.time()
    # res = calculator.calc_angle(0.28, -0.24, 0.08, 0, 0, 0)
    res = calculator.calc_pos(20, 20, 20, 20, 20, 20)
    # print(calculator.calc_pos(0, 0, 0, 0, 0, 0))
    ed = time.time()
    print(res)
    print("time= ", ed-st)


class T1(threading.Thread):
    global c1, c2, c3, c4, c5, c6, s1, s2, s3, s4, s5, s6, res1

    def __init__(self):
        threading.Thread.__init__(self)
        pass

    def run(self):
        res1 = np.around(s1*(c4*s6 + c5*c6*s4) - (s4*s6 - c4*c5*c6) *
                         (c1*c2*c3 - c1*s2*s3) - c6*s5*(c1*c2*s3 + c1*c3*s2), 15)
        print(res1)


if __name__ == '__main__':
    main()
    # t1 = T1()
    # theta1 = 30
    # theta2 = 40
    # theta3 = 50
    # theta4 = 60
    # theta5 = 70
    # theta6 = 80
    # st = time.time()
    # c1 = np.cos(theta1*np.pi/180)
    # s1 = np.sin(theta1*np.pi/180)
    # c2 = np.cos(theta2*np.pi/180)
    # s2 = np.sin(theta2*np.pi/180)
    # c3 = np.cos(theta3*np.pi/180)
    # s3 = np.sin(theta3*np.pi/180)
    # c4 = np.cos(theta4*np.pi/180)
    # s4 = np.sin(theta4*np.pi/180)
    # c5 = np.cos(theta5*np.pi/180)
    # s5 = np.sin(theta5*np.pi/180)
    # c6 = np.cos(theta6*np.pi/180)
    # st = time.time()
    # s6 = np.sin(theta6*np.pi/180)
    # ed = time.time()
    # st = time.time()
    # res1 = 0
    # t1.start()
    # t1.join()
    # ed = time.time()
    # print("res= ", res1)
    # print("time= ", ed-st)
