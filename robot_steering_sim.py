# -*- coding: utf-8 -*-
import sys
import os
import matplotlib.pyplot as plt
import math
import matplotlib.patches as patches
import numpy as np
from scipy.interpolate import interp1d

# サンプリング周期[s]
DT = 0.01
# ロボットの車輪間の幅[m]
W = 0.5
# Look ahead distance
L = 1


class Visualize:
    def __init__(self):
        self.r = 0.2
        self.robot_pos_x_list = []
        self.robot_pos_y_list = []
        self.way_point_x = []
        self.way_point_y = []

        self.ax = plt.axes()

    def config_screen(self):
        self.ax.cla()
        self.ax.axis('equal')
        self.ax.set_xlim(-1, 7)
        self.ax.set_ylim(-5, 5)
        self.ax.set_xlabel("X [m]", fontsize=20)
        self.ax.set_ylabel("Y [m]", fontsize=20)

    def move_robot(self, pos):
        x, y, theta = pos

        # 座標系を描画
        self.draw_coordinate(pos)
        # ロボットの軌跡を描画
        self.draw_trajectory(pos)

        xn = x + self.r * math.cos(theta)
        yn = y + self.r * math.sin(theta)
        self.ax.plot([x, xn], [y, yn], color="black")
        c = patches.Circle(xy=(x, y), radius=self.r, fill=False, color="black")
        self.ax.add_patch(c)

    def set_way_point(self, way_point_x, way_point_y):
        self.way_point_x = way_point_x
        self.way_point_y = way_point_y

    def draw_way_point(self):
        self.ax.scatter(self.way_point_x, self.way_point_y,
                        marker="*", s=5, color="black")

    def draw_coordinate(self, pose):
        x, y, theta = pose
        ux = 0.3 * math.cos(theta)
        vx = 0.3 * math.sin(theta)
        self.ax.quiver(x, y, ux, vx, angles='xy',
                       scale_units='xy', alpha=0.3, width=0.003, scale=1)
        self.ax.annotate("x", xy=(x+ux, y+vx))

        uy = - 0.3 * math.sin(theta)
        vy = 0.3 * math.cos(theta)
        self.ax.quiver(x, y, uy, vy, angles='xy',
                       scale_units='xy', alpha=0.3, width=0.003, scale=1)
        self.ax.annotate("y", xy=(x + uy, y + vy))

    def draw_trajectory(self, robot_pos):
        x, y, _ = robot_pos
        self.robot_pos_x_list.append(x)
        self.robot_pos_y_list.append(y)
        self.ax.scatter(self.robot_pos_x_list,
                        self.robot_pos_y_list, s=5, color="blue", label="trajectory")
        self.ax.legend()

# ロボットの移動時に正規分布に基づくノイズをのせる


def noise(u, w):
    print("正規分布")


class MotionControl:
    def __init__(self, way_point_list):
        self.v = 0.
        self.look_ahead = 1.0
        self.way_point_list = way_point_list
        self.interpolation()

        print(len(self.ix))

    # 一次補間 or スプライン補間??
    # サンプリング周期DTで参照位置を離散化する
    def interpolation(self, alg="cubic"):
        self.ix = []
        self.iy = []

        temp_x = temp_y = np.array([])
        for way_point in way_point_list:
            temp_x = np.append(temp_x, way_point[0])
            temp_y = np.append(temp_y, way_point[1])
        # スプライン補間
        cubic_spline = None
        if alg == "linear":
            cubic_spline = interp1d(temp_x, temp_y)
        # 線形補間
        elif alg == "cubic":
            cubic_spline = interp1d(temp_x, temp_y, kind='cubic')

        self.ix = np.linspace(
            way_point_list[0][0], way_point_list[-1][0], num=700)
        self.iy = cubic_spline(self.ix)

    # ロボットの速度が目標の速度へ追従するように比例制御(P制御)を行う
    def pid_velocity_control(self, target_velocity):
        return 1 * (target_velocity - self.v)

    # Look Ahead Distanceに基づきリファレンスデータを得る
    def seach_reference_point(self, state):
        try:
            x, y, theta = state
            index = 0
            distance = 0.1 * self.v + 1.0
            self.look_ahead = 0
            while index <= len(self.ix)-1:
                dx = self.ix[index] - x
                dy = self.iy[index] - y
                self.look_ahead = math.sqrt(pow(dx, 2) + pow(dy, 2))
                index += 1
                if distance < self.look_ahead:
                    return index-1, distance
                if len(self.ix)-1 <= index:
                    break
        except Exception as message:
            print(str(message))
        # 指定したlook ahead distanceに満たない場合は最も距離が近い
        # リファレンスデータを参照するようにする
        #dx = [x - t_ix for t_ix in self.ix]
        #dy = [y - t_iy for t_iy in self.iy]
        # self.look_ahead = [math.sqrt(pow(idx, 2) + pow(idy, 2))
        #                   for (idx, idy) in zip(dx, dy)]
        # return distance.index(min(distance)), min(distance)

    # pure pursuitによりステアンリング角度を計算する
    def calc_pure_pursuit(self, state):
        x, y, theta = state
        # リファレンスパスを検索する
        index, self.look_ahead = self.seach_reference_point(state)
        x_ref = self.ix[index]
        y_ref = self.iy[index]

        # 方位誤差を計算する
        alpha = math.atan2(y_ref - y, x_ref - x) - theta

        if self.v < 0:  # back
            alpha = math.pi - alpha

        # ステアリング角を計算する
        delta = math.atan2(2.0 * self.look_ahead *
                           math.sin(alpha) / (1.0 + 0.1 * self.v), 1.0)
        print("look ahead: {}".format(self.look_ahead))

        return delta

    # ロボットの運動モデルにはKinematics Modelを使用する
    # 前進速度とステアリング角によりt+1後の位置を計算する
    # https://myenigma.hatenablog.com/entry/2017/05/14/151539#Kinematic-Model
    def steering_control(self, state, ak, steering_angle):
        x, y, theta = state

        x = x + self.v * math.cos(theta) * DT
        y = y + self.v * math.sin(theta) * DT
        theta = theta + (self.v / self.look_ahead) * \
            math.tan(steering_angle) * DT
        print("theta: {}".format(theta))
        self.v = self.v + ak * DT

        return [x, y, theta]


way_point_list = [[0, 0], [0.5, 0.5], [1.5, -0.2],
                  [2.7, -0.6], [4.0, 0.1], [6.3, -0.4]]


if __name__ == "__main__":
    controller = MotionControl(way_point_list)
    visualize = Visualize()

    # ロボットの位置 [m]
    robot_pos = np.array([0, 0.5, 0])
    # ロボットの目標速度 [m/s]
    target_velocity = 1.5

    visualize.set_way_point(controller.ix, controller.iy)

    count = 0
    print("simulation start.")
    while True:
        visualize.config_screen()
        # way pointを描画する
        visualize.draw_way_point()

        # P制御により計算されたロボットの速度
        ak = controller.pid_velocity_control(target_velocity)
        # pure pursitにより計算されたステアリング角
        steering_angle = controller.calc_pure_pursuit(robot_pos)
        # 方位、前進速度、現在位置から次の位置を更新する
        robot_pos = controller.steering_control(
            robot_pos, ak, steering_angle)

        # ロボットを描画する
        visualize.move_robot(robot_pos)

        plt.pause(DT)
        count = count + 1
        if 100 < float(DT * count):
            break
    print("simulation done.")
