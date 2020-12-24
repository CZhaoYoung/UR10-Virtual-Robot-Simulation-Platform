#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
import matplotlib.pyplot as plt

pi = math.pi
K_alpha = 1  # 控制系数
k_p = 1  # 控制系数
T1 = 0.2  # 控制系数
m1 = 7.1  # link1
m2 = 12.7  # link2
m3 = 4.27  # link3
gc = 9.8  # 重力加速度
delta_t = 0.01  # 迭代时间间隔
step1 = 10000

q = np.array([[pi / 2], [0], [0]], dtype=np.float)  # 角度初始化
dq = np.zeros([3, 1], dtype=np.float)  # 角速度
ut = np.zeros([3, 1], dtype=np.float)  # 力矩初始化

T = 0
skl = np.array([[-0.0655], [-0.09713], [-0.01057]], dtype=np.float)
xl = np.array([[0], [-1.1843], [0.118]], dtype=np.float)
alphal = np.array([[3.27], [4.84], [0.335]], dtype=np.float)


# build matrix M, C, G
def mcg(q, dq, m2, m3):
    q2 = q[1, 0]
    q3 = q[2, 0]
    dq1 = dq[0, 0]
    dq2 = dq[1, 0]
    dq3 = dq[2, 0]

    M = np.empty([3, 3])
    M[0, 0] = (8555197 * m3) / 40000000. - (9025071 * m2) / 200000000. - (9025071 * m2 * math.cos(2 * q2)) / 200000000. + (
                37540129 * m3 * math.cos(2 * q2)) / 200000000. + (327241 * m3 * math.cos(2 * q2 + 2 * q3)) / 12500000. + (
                          5079283 * m3 * math.cos(q3)) / 25000000. + (
                          5079283 * m3 * math.cos(2 * q2 + q3)) / 25000000. + 1123 / 2000.
    M[0, 1] = (24293 * m3 * math.sin(q2 + q3)) / 625000. + (484033 * m2 * math.sin(q2)) / 5000000. + (
                104159 * m3 * math.sin(q2)) / 2500000.
    M[0, 2] = (24293 * m3 * math.sin(q2 + q3)) / 625000.
    M[1, 0] = (24293 * m3 * math.sin(q2 + q3)) / 625000. + (484033 * m2 * math.sin(q2)) / 5000000. + (
                104159 * m3 * math.sin(q2)) / 2500000.
    M[1, 1] = (8555197 * m3) / 20000000. - (9025071 * m2) / 100000000. + (5079283 * m3 * math.cos(q3)) / 12500000. + 1 / 20.
    M[1, 2] = (327241 * m3) / 6250000. + (5079283 * m3 * math.cos(q3)) / 25000000. + 1 / 100.
    M[2, 0] = (24293 * m3 * math.sin(q2 + q3)) / 625000
    M[2, 1] = (327241 * m3) / 6250000. + (5079283 * m3 * math.cos(q3)) / 25000000. + 1 / 100.
    M[2, 2] = (327241 * m3) / 6250000. + 1 / 100.

    C = np.empty([3, 3])
    C[0, 0] = -dq2 * ((37540129 * m3 * math.sin(2 * q2)) / 200000000. - (9025071 * m2 * math.sin(2 * q2)) / 200000000. + (
                327241 * m3 * math.sin(2 * q2 + 2 * q3)) / 12500000. + (
                                  5079283 * m3 * math.sin(2 * q2 + q3)) / 25000000.) - (dq3 * m3 * (
                5079283 * math.sin(2 * q2 + q3) + 1308964 * math.sin(2 * q2 + 2 * q3) + 5079283 * math.sin(
            q3))) / 50000000.
    C[0, 1] = dq2 * ((24293 * m3 * math.cos(q2 + q3)) / 625000. + (484033 * m2 * math.cos(q2)) / 5000000. + (
                104159 * m3 * math.cos(q2)) / 2500000.) - dq1 * ((37540129 * m3 * math.sin(2 * q2)) / 200000000. - (
                9025071 * m2 * math.sin(2 * q2)) / 200000000. + (327241 * m3 * math.sin(2 * q2 + 2 * q3)) / 12500000. + (
                                                                            5079283 * m3 * math.sin(
                                                                        2 * q2 + q3)) / 25000000.) + (
                          24293 * dq3 * m3 * math.cos(q2 + q3)) / 625000.
    C[0, 2] = -(m3 * (1308964 * dq1 * math.sin(2 * q2 + 2 * q3) - 1943440 * dq2 * math.cos(
        q2 + q3) - 1943440 * dq3 * math.cos(q2 + q3) + 5079283 * dq1 * math.sin(q3) + 5079283 * dq1 * math.sin(
        2 * q2 + q3))) / 50000000.
    C[1, 0] = dq1 * ((37540129 * m3 * math.sin(2 * q2)) / 200000000. - (9025071 * m2 * math.sin(2 * q2)) / 200000000. + (
                327241 * m3 * math.sin(2 * q2 + 2 * q3)) / 12500000. + (5079283 * m3 * math.sin(2 * q2 + q3)) / 25000000.)
    C[1, 1] = -(5079283 * dq3 * m3 * math.sin(q3)) / 25000000.
    C[1, 2] = -(5079283 * m3 * math.sin(q3) * (dq2 + dq3)) / 25000000.
    C[2, 0] = (dq1 * m3 * (5079283 * math.sin(2 * q2 + q3) + 1308964 * math.sin(2 * q2 + 2 * q3) + 5079283 * math.sin(
        q3))) / 50000000.
    C[2, 1] = (5079283 * dq2 * m3 * math.sin(q3)) / 25000000.
    C[2, 2] = 0

    G = np.empty([3, 1])
    G[0, 0] = 0
    G[1, 0] = -(gc * (3316 * m3 * math.cos(q2 + q3) + 2327 * m2 * math.cos(q2) + 6127 * m3 * math.cos(q2))) / 10000.
    G[2, 0] = -(829 * gc * m3 * math.cos(q2 + q3)) / 2500.

    return M, C, G


def dynamic_fun(q, dq, ut, m2, m3):
    M, C, g = mcg(q, dq, m2, m3)
    inv_M = np.linalg.inv(M)
    ddq = np.dot(inv_M, ut - np.dot(C, dq) - g)  # 角加速度

    dq = ddq * delta_t + dq
    q = dq * delta_t + q
    return q, dq


def controller(q, dq):
    global T
    r_2 = 0.0025
    c = 1
    q1 = q[0, 0]
    q2 = q[1, 0]
    q3 = q[2, 0]

    M, C, g = mcg(q, dq, m2, m3)

    J = np.empty([3, 3])
    J[0, 0] = (1429 * math.sin(q3) * (
                (4967757600021511 * math.cos(q1) * math.cos(q2)) / 81129638414606681695789005144064. - math.sin(
            q1) * math.sin(q2))) / 2500. + (
                          1486203653092373 * math.cos(q1) * math.sin(q2)) / 39614081257132168796771975168000. + (
                          6127 * math.cos(q2) * math.sin(q1)) / 10000. + (1429 * math.cos(q3) * (
                (4967757600021511 * math.cos(q1) * math.sin(q2)) / 81129638414606681695789005144064. + math.cos(
            q2) * math.sin(q1))) / 2500.
    J[0, 1] = (1429 * math.sin(q3) * (math.cos(q1) * math.cos(q2) - (
                4967757600021511 * math.sin(q1) * math.sin(q2)) / 81129638414606681695789005144064.)) / 2500. + (
                          6127 * math.cos(q1) * math.sin(q2)) / 10000. + (
                          1486203653092373 * math.cos(q2) * math.sin(q1)) / 39614081257132168796771975168000. + (
                          1429 * math.cos(q3) * (math.cos(q1) * math.sin(q2) + (
                              4967757600021511 * math.cos(q2) * math.sin(
                          q1)) / 81129638414606681695789005144064.)) / 2500.
    J[0, 2] = (1429 * math.sin(q3) * (math.cos(q1) * math.cos(q2) - (
                4967757600021511 * math.sin(q1) * math.sin(q2)) / 81129638414606681695789005144064.)) / 2500. + (
                          1429 * math.cos(q3) * (math.cos(q1) * math.sin(q2) + (
                              4967757600021511 * math.cos(q2) * math.sin(
                          q1)) / 81129638414606681695789005144064.)) / 2500.
    J[1, 0] = (1429 * math.sin(q3) * (math.cos(q1) * math.sin(q2) + (
                4967757600021511 * math.cos(q2) * math.sin(q1)) / 81129638414606681695789005144064.)) / 2500. - (
                          6127 * math.cos(q1) * math.cos(q2)) / 10000. + (
                          1486203653092373 * math.sin(q1) * math.sin(q2)) / 39614081257132168796771975168000. - (
                          1429 * math.cos(q3) * (math.cos(q1) * math.cos(q2) - (
                              4967757600021511 * math.sin(q1) * math.sin(
                          q2)) / 81129638414606681695789005144064.)) / 2500.
    J[1, 1] = (1429 * math.sin(q3) * (
                (4967757600021511 * math.cos(q1) * math.sin(q2)) / 81129638414606681695789005144064. + math.cos(
            q2) * math.sin(q1))) / 2500. - (
                          1486203653092373 * math.cos(q1) * math.cos(q2)) / 39614081257132168796771975168000. + (
                          6127 * math.sin(q1) * math.sin(q2)) / 10000. - (1429 * math.cos(q3) * (
                (4967757600021511 * math.cos(q1) * math.cos(q2)) / 81129638414606681695789005144064. - math.sin(
            q1) * math.sin(q2))) / 2500.
    J[1, 2] = (1429 * math.sin(q3) * (
                (4967757600021511 * math.cos(q1) * math.sin(q2)) / 81129638414606681695789005144064. + math.cos(
            q2) * math.sin(q1))) / 2500. - (1429 * math.cos(q3) * (
                (4967757600021511 * math.cos(q1) * math.cos(q2)) / 81129638414606681695789005144064. - math.sin(
            q1) * math.sin(q2))) / 2500.
    J[2, 0] = 0
    J[2, 1] = (1429 * math.sin(q2) * math.sin(q3)) / 2500. - (1429 * math.cos(q2) * math.cos(q3)) / 2500. - (
                6127 * math.cos(q2)) / 10000.
    J[2, 2] = (1429 * math.sin(q2) * math.sin(q3)) / 2500. - (1429 * math.cos(q2) * math.cos(q3)) / 2500.

    X = np.empty([3, 1])
    X[0, 0] = J[1, 0]
    X[1, 0] = -(1429 * math.sin(q3) * (
                (4967757600021511 * math.cos(q1) * math.cos(q2)) / 81129638414606681695789005144064. - math.sin(
            q1) * math.sin(q2))) / 2500. - (
                          1486203653092373 * math.cos(q1) * math.sin(q2)) / 39614081257132168796771975168000. - (
                          6127 * math.cos(q2) * math.sin(q1)) / 10000. - (1429 * math.cos(q3) * (
                (4967757600021511 * math.cos(q1) * math.sin(q2)) / 81129638414606681695789005144064. + math.cos(
            q2) * math.sin(q1))) / 2500.
    X[2, 0] = 59 / 500. - (1429 * math.cos(q2) * math.sin(q3)) / 2500. - (1429 * math.cos(q3) * math.sin(q2)) / 2500. - (
                6127 * math.sin(q2)) / 10000.

    dx = np.empty([3, 1])
    dx[0, 0] = (X[0, 0] - xl[0, 0]) / delta_t
    dx[1, 0] = (X[1, 0] - xl[1, 0]) / delta_t
    dx[2, 0] = (X[2, 0] - xl[2, 0]) / delta_t

    xl[0, 0] = X[0, 0]
    xl[1, 0] = X[1, 0]
    xl[2, 0] = X[2, 0]

    X0 = np.empty([3, 1])
    X0[0, 0] = 0.5 + 0.3 * math.cos(0.05 * T)
    X0[1, 0] = 0.3 * math.sin(0.05 * T)
    X0[2, 0] = 0.2

    dX0 = np.empty([3, 1])
    dX0[0, 0] = -0.3 * 0.05 * math.sin(0.05 * T)
    dX0[1, 0] = 0.3 * 0.05 * math.cos(0.05 * T)
    dX0[2, 0] = 0
    dX00 = np.empty([3, 1])
    dX00[0, 0] = -0.3 * 0.05 * 0.05 * math.sin(0.05 * T)
    dX00[1, 0] = 0.3 * 0.05 * 0.05 * math.cos(0.05 * T)
    dX00[2, 0] = 0

    # 计算误差
    err_p = np.empty([3, 1])
    err_p[0, 0] = (X[0, 0] - X0[0, 0])
    err_p[1, 0] = (X[1, 0] - X0[1, 0])
    err_p[2, 0] = (X[2, 0] - X0[2, 0])

    X_X00 = (X[0, 0] - X0[0, 0]) * (X[0, 0] - X0[0, 0])
    X_X01 = (X[1, 0] - X0[1, 0]) * (X[1, 0] - X0[1, 0])
    X_X02 = (X[2, 0] - X0[2, 0]) * (X[2, 0] - X0[2, 0])
    tmp1 = X_X00 + X_X01 + X_X02

    tmp2 = tmp1 - r_2
    if tmp2 > 0:
        f = tmp2
    else:
        f = 0

    FD_p = np.empty([3, 1])
    FD_p[0, 0] = 2 * c * f * (X[0, 0] - X0[0, 0])
    FD_p[1, 0] = 2 * c * f * (X[1, 0] - X0[1, 0])
    FD_p[2, 0] = 2 * c * f * (X[2, 0] - X0[2, 0])

    alpha = np.empty([3, 1])
    alpha[0, 0] = -K_alpha * FD_p[0, 0]
    alpha[1, 0] = -K_alpha * FD_p[1, 0]
    alpha[2, 0] = -K_alpha * FD_p[2, 0]

    dalpha = np.empty([3, 1])
    dalpha[0, 0] = (alpha[0, 0] - alphal[0, 0]) / delta_t
    dalpha[1, 0] = (alpha[1, 0] - alphal[1, 0]) / delta_t
    dalpha[2, 0] = (alpha[2, 0] - alphal[2, 0]) / delta_t

    alphal[0, 0] = alpha[0, 0]
    alphal[1, 0] = alpha[1, 0]
    alphal[2, 0] = alpha[2, 0]

    z = dx - dX0 - alpha
    k = k_p * z + FD_p

    sk = np.empty([3, 1])
    sk[0, 0] = k[0, 0] * delta_t + skl[0, 0]
    sk[1, 0] = k[1, 0] * delta_t + skl[1, 0]
    sk[2, 0] = k[2, 0] * delta_t + skl[2, 0]

    skl[0, 0] = sk[0, 0]
    skl[1, 0] = sk[1, 0]
    skl[2, 0] = sk[2, 0]

    inv_J = np.linalg.inv(J)
    ut = -np.dot(np.dot(M, inv_J), sk) - np.dot(np.dot(M, inv_J), z) * 1 / T1 + np.dot(np.dot(M, J),(dX00 - dalpha)) + g + np.dot(C, dq)
    T = T + delta_t

    return ut, err_p, q, dq, X


if __name__ == '__main__':
    ax = []
    ay1 = []
    ay2 = []
    ay3 = []

    for i in range(step1):
        # ut 力矩初始化
        #
        q, dq = dynamic_fun(q, dq, ut, m2, m3)
        ut, err, q, dq, X = controller(q, dq)
        print("第"+str(i/100.)+"秒")
        print("距离目标点误差：")
        print(err)
        print("三个轴的角度")
        print(q)

        plt.ion()
        ax.append(i)  # 添加 i 到 x 轴的数据中
        ay1.append(err[0, 0])  # 添加 i 的平方到 y 轴的数据中
        ay2.append(err[1, 0])
        ay3.append(err[2, 0])
        plt.clf()
        plt.plot(ax, ay1, 'b-', label='X')
        plt.plot(ax, ay2, 'r-', label='Y')
        plt.plot(ax, ay3, 'g-', label='Z')
        plt.legend(loc="upper left")
        plt.pause(0.01)