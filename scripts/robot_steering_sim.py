# -*- coding: utf-8 -*-
import sys
import os
import matplotlib.pyplot as plt
import math
import matplotlib.patches as patches
import numpy as np
import matplotlib.animation as animation
from scipy.interpolate import interp1d

# サンプリング周期[s]
DT = 0.01

class Visualize:
    def __init__(self):
        self.r = 0.2
        self.robot_pos_x_list = []
        self.robot_pos_y_list = []
        self.way_point_x = []
        self.way_point_y = []

        self.fig = plt.figure()
        self.ims = []
        self.ax = plt.axes()

    def config_screen(self):
        self.ax.cla()
        self.ax.axis('equal')
        self.ax.set_xlim(-1, 7)
        self.ax.set_ylim(-6, 6)
        self.ax.set_xlabel("X [m]", fontsize=20)
        self.ax.set_ylabel("Y [m]", fontsize=20)

    def save_gif(self):
        ani = animation.ArtistAnimation(self.fig, self.ims, interval=50, blit=True,
                                        repeat_delay=1000)
        ani.save('anim.gif', writer="imagemagick")

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

    def set_way_point(self, way_point_list):
        self.way_point_x, self.way_point_y = interpolation(
            way_point_list)
        self.way_point_list = way_point_list

    def draw_way_point(self):
        self.ax.scatter(self.way_point_x, self.way_point_y,
                        marker="o", s=2, color="red")
        for way_point in self.way_point_list:
            self.ax.scatter(way_point[0], way_point[1],
                            marker="*", s=20, color="black")

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
                        self.robot_pos_y_list, s=1, color="blue", label="trajectory")
        self.ax.legend()

# 一次補間 or スプライン補間??
    # サンプリング周期DTで参照位置を離散化する
def interpolation(way_point_list, alg="cubic"):
    ix = []
    iy = []

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

    ix = np.linspace(
        way_point_list[0][0], way_point_list[-1][0], num=700)
    iy = cubic_spline(ix)

    return ix, iy

class MotionControl:
    def __init__(self, way_point_list):
        self.v = 0.
        self.look_ahead = 2.0
        self.way_point_list = way_point_list
        self.ix, self.iy = interpolation(way_point_list)

    # ロボットの速度が目標の速度へ追従するように比例制御(P制御)を行う
    def pid_velocity_control(self, target_velocity):
        return 1 * (target_velocity - self.v)

    # 正規分布に基づくノイズをロボットに与える
    def noise(self, robot_pos):
        w = 0
        return robot_pos

    # Look Ahead Distanceに基づきリファレンスデータを得る
    def seach_reference_point(self, robot_pos):
        try:
            x, y, theta = robot_pos
            dx = [x - t_ix for t_ix in self.ix]
            dy = [y - t_iy for t_iy in self.iy]
            distance = [abs(math.sqrt(pow(idx, 2) + pow(idy, 2))) for (idx, idy) in zip(dx, dy)]
            index = distance.index(min(distance))
            self.look_ahead = 0.
            look_ahead_filter = 0.1 * self.v + 0.3

            while look_ahead_filter > self.look_ahead and (index + 1) < len(self.ix):
                dx = self.ix[index + 1] - self.ix[index]
                dy = self.iy[index + 1] - self.iy[index]
                self.look_ahead += math.sqrt(dx ** 2 + dy ** 2)
                index += 1
            return index
        except Exception as message:
            print(str(message))

    # pure pursuitによりステアンリング角度を計算する
    def calc_pure_pursuit(self, robot_pos):
        x, y, theta = robot_pos
        # リファレンスパスを検索する
        index = self.seach_reference_point(robot_pos)

        # 方位誤差を計算する
        alpha = math.atan2(self.iy[index] - y, self.ix[index] - x) - theta

        # ステアリング角を計算する
        steering_angle = math.atan2(2.0 * self.look_ahead *
                                    math.sin(alpha) / (0.3 + 0.1 * self.v), 1.0)

        return steering_angle

    # ロボットの運動モデルにはKinematics Modelを使用する
    # 前進速度とステアリング角によりt+1後の位置を計算する
    # https://myenigma.hatenablog.com/entry/2017/05/14/151539#Kinematic-Model
    def steering_control(self, robot_pos, ak, steering_angle):
        x, y, theta = robot_pos

        x = x + self.v * math.cos(theta) * DT
        y = y + self.v * math.sin(theta) * DT
        theta = theta + (self.v / self.look_ahead) * \
            math.tan(steering_angle) * DT
        self.v = self.v + ak * DT

        return [x, y, theta]


way_point_list = [[0, 0], [0.5, 0.5], [1.5, -0.2],
                  [2.7, -0.6], [4.0, 0.1], [6.3, -0.4]]


def check_goal(robot_pos):
    threshold = 0.1
    x, y, _ = robot_pos
    if (way_point_list[-1][0] - threshold < x < way_point_list[-1][0] + threshold) and (way_point_list[-1][1] - threshold < y < way_point_list[-1][1] + threshold):
        return True
    else:
        return False


if __name__ == "__main__":
    controller = MotionControl(way_point_list)
    visualize = Visualize()

    # ロボットの位置 [m]
    robot_pos = np.array([-0.1, 0.5, 0])
    # ロボットの目標速度 [m/s]
    target_velocity = 7.5

    visualize.set_way_point(way_point_list)

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

        if check_goal(robot_pos) is False:
            # 方位、前進速度、現在位置から次の位置を更新する
            robot_pos = controller.steering_control(
                robot_pos, ak, steering_angle)

        # ロボットを描画する
        visualize.move_robot(robot_pos)

        plt.pause(DT)
        count += 1
        if 100 < float(DT * count):
            break
    print("simulation done.")
