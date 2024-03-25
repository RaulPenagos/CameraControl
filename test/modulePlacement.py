import numpy as np
import sys
import matplotlib.pyplot as plt
from src.Robot import Robot
from src.Table import Table

if __name__=='__main__':



    robot = Robot(30.0, 30.0, 0.1, 0.1)
    table = Table()
    
    table.generatePointsInSquare(10.0, 10.0, -20.0, 40.0)  
   
    robotMeasurements = robot.takeMeasurements(table.points)
    cameraMeasurements = robot.takeCameraMeasurements(table.points)

    fig = plt.figure(figsize = plt.figaspect(0.5))
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2)
    
    ax1.plot(table.points[:][0], table.points[:][1])
    plt.show()
    


    
    


