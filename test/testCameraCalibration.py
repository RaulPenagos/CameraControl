import numpy as np
import sys
import matplotlib.pyplot as plt
from src.Robot import Robot
from src.Table import Table
from src.Likelihood import MyLikelihood
from src.Camera import Camera




def runTestCameraPosition(table, robot):
   
    robotMeasurements = robot.takeMeasurements(table.points)
    cameraMeasurements = robot.takeCameraMeasurements(table.points, robotMeasurements)
    
    robotM = table.prepareForFit(robotMeasurements)
    robotC = table.prepareForFit(cameraMeasurements)
    
    
    l = MyLikelihood(robotM, robotC, robot)
    result = l.fit(start_params=[5, 3])
    cov = result.cov_params()
    return result.params[0], result.params[1], cov[0][0], cov[1][1]
 

def runTestPositionDetermination(table1, table2, robot, newrobot):

    robotMeasurements = robot.takeMeasurements(table1.points)
    cameraMeasurements = robot.takeCameraMeasurements(table2.points, robotMeasurements)
    
    d2 = 0.0
    print('len')
    print(len(table2.points))
    for i in range(0, len(table2.points)):
        rPoint = robotMeasurements[i]
        cPoint = cameraMeasurements[i]
        valid, cPointGlobal = newrobot.fromCameraToGlobalSystem(rPoint, cPoint)
        d2 += (table2.points[i][0] - cPointGlobal[0])**2 + (table2.points[i][1] - cPointGlobal[1])**2
    return np.sqrt(d2/len(table2.points))
            
if __name__=='__main__':
    
    # R1 and R2 of the robot
    R1 = 30.0
    R2 = 30.0

    # Generate the table
    table = Table()
    table.generatePoints()
      
    # Generate the camera  
    camera = Camera(2.0, 0.0, 3.0, 0.0, 0.0, 0.0)

    # Grid and binning 
    sigmaRobot = np.arange(0.01, 0.1, 0.005)
    sigmaCamera = np.arange(0.01, 0.1, 0.005)
    n = len(sigmaRobot)
    m = len(sigmaCamera)
    sigmaRobot, sigmaCamera = np.meshgrid(sigmaRobot, sigmaCamera)
    sigmaX = np.copy(sigmaRobot)
    sigmaY = np.copy(sigmaRobot)
    sigmaD = np.copy(sigmaRobot)
    for i in range(0, m):
        for j in range(0, n):
            sigmaR = sigmaRobot[i][j]
            sigmaC = sigmaCamera[i][j]
            robot = Robot(R1, R2, table, camera, sigmaR, sigmaC)

            
            deltax, deltay, sigmax, sigmay = runTestCameraPosition(table, robot)
            sigmaX[i][j] = np.sqrt(sigmax)
            sigmaY[i][j] = np.sqrt(sigmay)
            newCameraCoordinates = [deltax, deltay, 0.0, 0.0, 0.0, 0.0]
            newrobot = Robot(50.0, 50.0, newCameraCoordinates, sigmaR, sigmaC)
            d = runTestPositionDetermination(table3, table2, robot, newrobot)
            print(d)
            sigmaD[i][j] = d


    fig = plt.figure(figsize = plt.figaspect(0.33))
    ax1 = fig.add_subplot(1, 3, 1)
    ax2 = fig.add_subplot(1, 3, 2)
    ax3 = fig.add_subplot(1, 3, 3)
    ax1.set_xlabel('sigma Robot [cm]')
    ax1.set_ylabel('sigma Camera [cm]')
    ax2.set_xlabel('sigma Robot [cm]')
    ax2.set_ylabel('sigma Camera [cm]')
    ax3.set_xlabel('sigma Robot [cm]')
    ax3.set_ylabel('sigma Camera [cm]')
    
    c1 = ax1.pcolormesh(sigmaRobot, sigmaCamera, sigmaX, cmap='plasma', vmin=0, vmax=0.05)
    ax1.set_title('X Uncertainty [cm]')
    ax1.axis([sigmaRobot.min(), sigmaRobot.max(), sigmaCamera.min(), sigmaCamera.max()])
    fig.colorbar(c1, ax=ax1)
    
    c2 = ax2.pcolormesh(sigmaRobot, sigmaCamera, sigmaY, cmap='plasma', vmin=0, vmax=0.05)
    ax2.set_title('Y Uncertainty [cm]')
    ax2.axis([sigmaRobot.min(), sigmaRobot.max(), sigmaCamera.min(), sigmaCamera.max()])
    fig.colorbar(c2, ax=ax2)
    
    c3 = ax3.pcolormesh(sigmaRobot, sigmaCamera, sigmaD, cmap='plasma', vmin=0, vmax=0.15)
    ax3.set_title('RMS distance [cm]')
    ax3.axis([sigmaRobot.min(), sigmaRobot.max(), sigmaCamera.min(), sigmaCamera.max()])
    fig.colorbar(c3, ax=ax3)
    
    plt.tight_layout()
    plt.savefig('colormaps.png')



    
    


