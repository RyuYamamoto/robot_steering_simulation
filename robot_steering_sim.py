# -*- coding: utf-8 -*-
import sys
import os
import matplotlib.pyplot as plt
import math
import matplotlib.patches as patches
import numpy as np
import threading
import signal

DT = 0.01


class Visualize:
    def __init__(self):
        self.r = 0.2
        self.robot_pos_x_list = []
        self.robot_pos_y_list = []

        fig = plt.figure(figsize=(8, 8))
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

    def draw_way_point(self, way_point_list):
        for way_point in way_point_list:
            self.ax.scatter(way_point[0], way_point[1],
                            marker="*", color="black")

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


class MotionControl:
    def __init__(self):
        self.v = 0.
        self.w = 0.
        self.way_point_list = []

    def set_velocity(self, v, w):
        self.v = v
        self.w = w

    def interpolation_way_point(self):
        print("interpolation...")

    def set_way_point(self, way_point_list):
        self.way_point_list = way_point_list

    def calc_pure_pursuit(self):
        print("calculate pure pursuit control...")

    def steering_control(self, state):
        x, y, theta = state

        R = 0
        if self.w != 0.:
            R = self.v / self.w
            x1 = x - R * math.sin(theta) + R * math.sin(theta + self.w * DT)
            y1 = y + R * math.cos(theta) - R * math.cos(theta + self.w * DT)
        else:
            R = self.v
            x1 = x + R * math.cos(theta) * DT
            y1 = y + R * math.sin(theta) * DT
        theta1 = theta + self.w * DT

        return [x1, y1, theta1]


if __name__ == "__main__":
    controller = MotionControl()
    visualize = Visualize()

    robot_pos = np.array([0, 0, 0])
    robot_old_pos = robot_pos

    way_point_list = [[0, 0], [0.5, 0.5], [1.5, -0.2],
                      [2.7, -0.6], [4.0, 0.1], [6.3, -0.4]]

    controller.set_velocity(1.5, 0)
    count = 0
    while True:
        visualize.config_screen()
        visualize.draw_way_point(way_point_list)
        if float(DT * count) < 1.0:
            controller.set_velocity(1.5, -0.8)
        elif 1.0 <= float(DT * count) < 2.0:
            controller.set_velocity(1.5, 1.5)
        elif 2.0 <= float(DT * count) < 3.5:
            controller.set_velocity(1.0, -1.0)
        elif 3.5 <= float(DT * count) < 4.5:
            controller.set_velocity(-1.5, 0)
        else:
            controller.set_velocity(-1.5, 1.0)
        robot_pos = controller.steering_control(robot_old_pos)
        visualize.move_robot(robot_pos)
        robot_old_pos = robot_pos
        plt.pause(DT)
        count = count + 1
        if 6.0 < float(DT * count):
            break
    print("simulation done.")
