"""
Generate and save animations of a robot moving over points in the table
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys

from src.Table import Table
from src.Camera import Camera
from src.Robot import Robot
from src.innerpoint import innerpoint



def main():

    fig = plt.figure(figsize = (16, 8), layout="constrained")
    gs0 = fig.add_gridspec(1, 2, width_ratios=[2, 1])
    ax1 = fig.add_subplot(gs0[0], projection = '3d')
    gs1 = gs0[1].subgridspec(2,1)
    ax2 = fig.add_subplot(gs1[0])
    ax3 = fig.add_subplot(gs1[1])
    ax1.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax1.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax1.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax1.set_xlabel('x [cm]')
    ax1.set_ylabel('y [cm]')
    ax1.set_zlabel('z [cm]')
    ax2.set_xlabel('x [cm]')
    ax2.set_ylabel('y [cm]')
    ax3.set_xlabel('z [cm]')
    ax3.set_ylabel('y [cm]')
    ax1.axes.set_xlim3d(left=-70, right=70.0) 
    ax1.axes.set_ylim3d(bottom=-70, top=70.0)   
    ax2.axes.set_xlim((-40.0, 70.0))
    ax2.axes.set_ylim((-70.0, 40.0))
    ax3.axes.set_xlim((-1.0, 1.0))
    ax3.axes.set_ylim((-1.0, 1.0))



    table = Table(0.01, 0.0)
    table.plotTable(ax1, ax2, 'g.')

    # Generate the camera  
    camera = Camera(x = 5 , y = 0, z = 10, psi = math.pi/180*5, theta = - math.pi/180*4, phi = math.pi/180*3.5, cx = -0.5, cy = -0.5, focaldistance = 20, sigmaCamera = 0.001)
    camera = Camera(x = 5 , y = 0, z = 10, psi =0, theta = 0, phi = 0, cx = -0.5, cy = -0.5, focaldistance = 20, sigmaCamera = 0.001)

    # Generate the robot
    robot = Robot(60.77, 38.0, 24.0, 34.0, table, camera, fig, ax1, ax2, ax3)


    pos_J = (1.4496846765933695, -2.3673601581311843, 39.41379211339806, 2.223481693056126)

    print(len(table.actualPoints))
    pos = innerpoint(pos_J[0], pos_J[1], 0, pos_J[3])


    print('Animation')
    ani = robot.animatedMove(robot.currentPos, 3)
    ani = robot.animatedMove(pos, 20)
    plt.show()
    ani.save("animacion.gif", writer="ffmpeg")


#     robot.MoveRobotTo(pos)
#     robot.cameraAim([25, -25, 0])

#     print(robot.currentCartesianPos.r)
#     print(robot.currentPos.Z)
#     print('r0 Camera', robot.camera.r0)
#     print('Pos Camera:', robot.camera.cartesianpos.r)
#     print('Camera rotation:')
#     print('psi: ', robot.camera.rotation0.psi)
#     print('theta: ', robot.camera.rotation0.theta)
#     print('phi: ', robot.camera.rotation0.phi)    

    


if __name__ == "__main__":
    main()
