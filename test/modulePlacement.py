import numpy as np
import sys
import matplotlib.pyplot as plt
from src.Robot import Robot
from src.Table import Table
from src.Likelihood import MyLikelihood

if __name__=='__main__':


    cameraCoordinates = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    robot = Robot(50.0, 50.0, cameraCoordinates, 0.05, 0.05)
    
    table = Table()
    table.generatePointsInSquare(30.0, 30.0, -20.0, 30.0)  
   
    robotMeasurements = robot.takeMeasurements(table.points)
    cameraMeasurements = robot.takeCameraMeasurements(table.points, robotMeasurements)
    
    robotM = table.prepareForFit(robotMeasurements)
    robotC = table.prepareForFit(cameraMeasurements)
    
    
    l = MyLikelihood(robotM, robotC, robot)
    result = l.fit(start_params=[5, 3])
    #print('a: ', result.params[0]) 
    #print('b: ', result.params[1]) 


    fig = plt.figure(figsize = plt.figaspect(0.5))
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2)
    
    thePoints = table.toNumpy(table.points)
    theRobotMeasurements = table.toNumpy(robotMeasurements)
    theCameraMeasurements = table.toNumpy(cameraMeasurements)
    


    xReal = thePoints[:,0]
    yReal = thePoints[:,1]
    xRobot = theRobotMeasurements[:, 0]
    yRobot = theRobotMeasurements[:, 1]
    xCamera = theCameraMeasurements[:, 0]
    yCamera = theCameraMeasurements[:, 1]

    ax1.set_xlim(left = 0, right=100.0)
    ax1.set_ylim(bottom=0.0, top=100.0)
    ax1.set_xlabel('x [cm]')
    ax1.set_ylabel('y [cm]')
    ax1.plot(xReal, yReal, 'b*')
    ax1.plot(xRobot, yRobot, 'r*')

    ax2.set_xlim(left=0, right=100.0)
    ax2.set_ylim(bottom=-50.0, top=50.0)
    ax2.set_xlabel('x [cm]')
    ax2.set_ylabel('y [cm]')
    ax2.plot(xCamera, yCamera, 'r*')



    plt.show()



    
    


