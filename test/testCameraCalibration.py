import numpy as np
import sys
import matplotlib.pyplot as plt
from src.Robot import Robot
from src.Table import Table
from src.Likelihood import MyLikelihood
from src.Camera import Camera





if __name__=='__main__':
    
    # R1 and R2 of the robot
    R1 = 30.0
    R2 = 30.0

    # Generate the table
    table = Table(tolerance = 0.001, z = -30.0)
    table.generatePoints()
      
    # Generate the camera  
    camera = Camera(x = 2.0, y = 0.0, z = 3.0, psi = 0.0, theta = 0.0, phi = 0.0, sigmaCamera = 0.001)

    # Generate the robot
    robot = Robot(R1, R2, table, camera)


   