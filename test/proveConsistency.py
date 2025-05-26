from statsmodels.base.model import GenericLikelihoodModel
import numpy as np
import copy
import sys
import os
import random

# Get the root directory of the project
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# Add the test/ directory explicitly
sys.path.append(os.path.join(project_root, "test"))

# Now, import the function from makeMeasurements.py
from makeMeasurements import *
from src.Table import Table
from src.Camera import Camera
from src.Robot import Robot
from src.innerpoint import innerpoint
from src.EulerRotation import EulerRotation

from statsmodels.stats.outliers_influence import variance_inflation_factor  # check colinearity

"""
This Script includes a Likelihood function capable of finding the camera's parameters 
by taking pictures of the points and minimizing the positions

Author: Raul Penagos
Date: February 12th 2025
"""


class Test():
    def __init__(self, real_robot, r2, r3, r4):
        self.robot = real_robot 
        self.robot2 = r2
        self.robot3 = r3
        self.robot4 = r4

        # Simulo las medidas reales, con robot1 real
        measurements, points, camera_measurements = make_measurements_camera_pointing(self.robot, False)      
        self.measurements = measurements
        self.camera_measurements = camera_measurements
        self.real_points = points
        n = len(self.real_points)

    def aim_same_points(self):
        plt.close('all')
        plt.figure()
        for i in range(20):
            pos = innerpoint(random.uniform(-np.pi, np.pi), random.uniform(-np.pi, np.pi), random.uniform(-np.pi, np.pi), 0 )
            # pos = innerpoint(0.2*random.uniform(-np.pi, np.pi), 0.2*random.uniform(-np.pi, np.pi), random.uniform(-np.pi, np.pi), 0 )

            self.robot.MoveRobotTo(pos)
            robot_points_to = self.robot.cameraPointing()[0:2]
            print(robot_points_to)
            plt.plot(robot_points_to[0], robot_points_to[1], 'or')
        
            self.robot2.MoveRobotTo(pos)
            robot_points_to = self.robot2.cameraPointing()[0:2]
            plt.plot(robot_points_to[0], robot_points_to[1], 'og')

            self.robot3.MoveRobotTo(pos)
            robot_points_to = self.robot3.cameraPointing()[0:2]
            plt.plot(robot_points_to[0], robot_points_to[1], 'ob')

            self.robot4.MoveRobotTo(pos)
            robot_points_to = self.robot4.cameraPointing()[0:2]
            plt.plot(robot_points_to[0], robot_points_to[1], 'ok')

            
            

        plt.show()

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

   # Generate the camera  
   camera = Camera(x = 5.0, y = 0, z = 24.0, psi = 0.01, theta =  0.02, phi = 0.03, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)
   # psi = np.pi/170, theta =  np.pi/160, phi = np.pi/180
   # psi = np.pi/360, theta =  2*np.pi/360, phi = np.pi/360
   # psi = 0.08, theta =  0.06, phi = 0.054,

   # Generate the robot
   robot = Robot(60.77, 38.0, 24.0, 34.0, table, camera, fig, ax1, ax2, ax3)

   #  My guess for the actual robot
   robot2 = copy.deepcopy(robot)
   camera2 = Camera(4.9404, -0.0002, 22.0001, 0.0101, 0.0200, -0.0301 , cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)

   robot3 = copy.deepcopy(robot)
   camera3 = Camera(4.9851, 0.4961, 22.0148, 0.01, 0.01, 0.01 , cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)

   robot4 = copy.deepcopy(robot)
   camera4 = Camera(4.9, 0, 22, -1.5380, 0.0180, 0.0267, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)
#    camera4 = Camera(0, 0, 0, 0, 0, 0, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)


    
   # camera2 = Camera(4.9, 0.57, 2.5, -0.06, -0.07 , 0.006, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)

   robot2.camera = camera2
   robot3.camera = camera3
   robot4.camera = camera4
   
   test = Test(robot, robot2,robot3,robot4)

   test.aim_same_points()



    






if __name__ == "__main__":
   main()
