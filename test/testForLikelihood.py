import numpy as np
import matplotlib.pyplot as plt
import sys
from src.Table import Table
from src.Camera import Camera
from src.Robot import Robot
from src.innerpoint import innerpoint


def test(robot, show: bool = False):
    """
    Gets where the unitary vector of the camera points to, for every of all calibration points
    Excepting those that are close to R=60 or further.
    """
    
    point = robot.table.points[11]
    print('point:', point)

    measurements = np.asarray([])
    camera_measurements = np.asarray([])
    points = np.asarray([])
        
    # point = [point[0] + np.random.normal(0, 0.02), point[1] + np.random.normal(0, 0.02), point[2]]
    point = [point[0] + 0.05, point[1] + 0.05, point[2]]

    if robot.fromCartesianToInner(point)[0] != False and np.linalg.norm(point) < 50:

        #  Measure from different angles
        # for j in np.linspace(0, np.pi, 8): 

        # robot.cameraAim(point, j)
        robot.cameraAim(point, 0)

        x, y = robot.point3DToCameraProjection(point)

        measurements = np.append(measurements, robot.currentPos)
        camera_measurements = np.append(camera_measurements, np.array([x,y]))
        points = np.append(points, point)

        
    n = int(len(points)/3)
    points = points.reshape([n, 3]) 
    camera_measurements = camera_measurements.reshape([n, 2]) 

    if show:
        print('---Reachable Cal. Points---')
        print(f'Robot_config(J1,J2,Jz) \t||\t camera_meas(X,Y) \t')
        print(f' J1 \t \t J2 \t \t  Z \t \t JZ \t || \t X \t \t Y \t || \t x \t \t y \t \t z')
        print(n, len(points), len(measurements))
        for i in range(n):

            measure = measurements[i]
            camera = camera_measurements[i]
            print(measure.J1,measure.J2,measure.Z,measure.Jz, '\t ||', camera[0] , camera[1], '\t ||', points[i,0], points[i,1], points[i,2])
    return measurements, points, camera_measurements

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
    # table.plotTable(ax1, ax2, 'g.')

    # Generate the camera  
    camera = Camera(x = 5.0, y = 0.0, z = 20.0, psi = np.pi/150, theta =  np.pi/140, phi = np.pi/180, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)

    # Generate the robot
    robot = Robot(60.77, 38.0, 24.0, 34.0, table, camera, fig, ax1, ax2, ax3)

    measure, points, camera_measurements = test(robot, True)

    robot.MoveRobotTo(measure[0])

    robot.cameraAim(points[0])
    
    print('2ª medida:')
    print(robot.currentCartesianPos.r)
    print(robot.currentPos.Z)
    print(robot.camera.r0)
    print(robot.camera.cartesianpos.r)



    # print(robot.point3DToCameraProjection(points))



    




if __name__ == "__main__":
    main()