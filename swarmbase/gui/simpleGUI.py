"""
Author       : Hanqing Qi
Date         : 2023-11-11 14:47:43
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-12 16:30:56
FilePath     : /GUI/SimpleGUI_V3/simpleGUI.py
Description  : The GUI for bicopter control V3
"""

import matplotlib.pyplot as plt
from gui.simpleGUIutils import *
import time


class SimpleGUI:
    def __init__(self):
        # Plotting initialization
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=GS)
        self.ax.set_facecolor(C["k"])  # Set background color
        self.fig.patch.set_facecolor(C["k"])  # Set background color
        self.ax.set_xlim(0, GS[0])
        self.ax.set_ylim(0, GS[1])
        self.ax.set_aspect("equal", "datalim")  # Set aspect ratio
        self.ax.set_xticks([])  # Remove x ticks
        self.ax.set_yticks([])  # Remove y ticks
        self.ax.axis("off")  # Remove axis

        init_yaw(self)
        init_height(self)
        init_variables(self)
        init_battery(self)
        init_distance(self)
        init_connection_status(self)
        init_buttons(self)

    def update(self, cur_yaw: float = 0, des_yaw: float = 0, cur_height: float = 0, des_height: float = 0, battery: float = 0, distance: float = 0, connection_status: bool = False):
        update_yaw(self, cur_yaw, des_yaw)
        update_height(self, cur_height, des_height)
        update_battery(self, battery)
        update_distance(self, distance)
        update_connection_status(self, connection_status)


if __name__ == "__main__":
    mygui = SimpleGUI()
    import math

    for i in range(101):
        mygui.update(
            cur_yaw=math.pi * i / 100,
            des_yaw=- math.pi * i / 100,
            cur_height=15 * i / 100,
            des_height=15 * (1 - i / 100),
            battery=4.5 * i / 100,
            distance=400 * i / 100,
            connection_status=True,
        )
        plt.pause(0.05)
    plt.ioff()
    plt.show()
