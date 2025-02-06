import numpy as np
import matplotlib.pyplot as plt
import sys

from src.Table import Table
from src.Camera import Camera
from src.Robot import Robot



def reachable_points(robot):
    plt.close('all')
    plt.figure(figsize = (8, 8))

    for point in robot.table.actualPoints:

        # d_point = np.sqrt(point[0]**2 + point[1]**2)
        # d_robot = robot.R1 + robot.R2
        if robot.fromCartesianToInner(point)[0] != False:  
            # Alcanzable
            plt.plot(point[0], point[1], 'og')
        else:
            plt.plot(point[0], point[1], 'or')

    plt.axhline(0, color='k', linestyle='--')
    plt.axvline(0, color='k', linestyle='--')
    plt.savefig('img/puntos.png')
    plt.show()
    print('Img Saved')


def make_measurements(robot, print_points = False):
    """
    Measures position of all calibration points
    Excepting those that are close to R=60 or further.
    """

    forbidden_points = np.asarray([])
    measurements = np.asarray([])
    points = np.asarray([])

    for n, point in enumerate(robot.table.actualPoints):
        if robot.fromCartesianToInner(point)[0] != False and np.linalg.norm(point) < 58:

            close_to_point = [i + np.random.normal(0, 0.1) for i in point]
        
            robot.cartesianMoveTo(close_to_point, 0)

            j1, j2 = robot.J1, robot.J2         

            x, y = robot.point3DToCameraProjection(point)

            points = np.append(points, point)
            measurements = np.append(measurements, [j1, j2 , x, y])
            
        else:
            # print('Punto no accesible', point)
            forbidden_points = np.append(forbidden_points, point)
            continue
    n = int(len(points)/3)
    n_f = int(len(forbidden_points)/3)
    measurements = measurements.reshape([n, 4]) 
    points = points.reshape([n, 3])
    forbidden_points = forbidden_points.reshape([n_f, 3])

    if print_points:
        print('---Reachable Points---')
        print(f'Robot_config(J1,J2) \t||\t camera_meas(X,Y) \t||\t real_pos(x,y)')
        print(f' J1 \t \t J2 \t \t  X \t \t Y \t \t  x \t \t y \t \t z')

        for i in range(n):
            print(measurements[i,0], measurements[i,1], measurements[i,2], measurements[i,3], '||', points[i,0], points[i,1], '\n')
            # print(point'\n')

        print('---Forbidden Points---')
        for point in forbidden_points:
            r = np.sqrt(point[0]**2 + point[1]**2)
            reach = robot.fromCartesianToInner(point)[0]
            print(f'{point} || {r} || {reach}')
        # Entran en juego las tolerancias

    return measurements, points


def make_measurements_camera_pointing(robot):
    """
    Gets where the unitary vector of the camera points to for every of all calibration points
    Excepting those that are close to R=60 or further.
    """
    measurements_pointing = np.asarray([])
    forbidden_points = np.asarray([])
    measurements = np.asarray([])
    points = np.asarray([])

    for n, point in enumerate(robot.table.actualPoints):
        if robot.fromCartesianToInner(point)[0] != False and np.linalg.norm(point) < 50:
        
            robot.cameraAim(point)

            j1, j2 = robot.J1, robot.J2         

            x, y = robot.point3DToCameraProjection(point)

            measurements = np.append(measurements, [j1, j2 , x, y])

            points = np.append(points, point)
            # measurements_pointing = np.append()
            
        else:
            # print('Punto no accesible', point)
            forbidden_points = np.append(forbidden_points, point)
            continue
    n = int(len(points)/3)
    n_f = int(len(forbidden_points)/3)
    measurements = measurements.reshape([n, 4]) 
    points = points.reshape([n, 3])
    forbidden_points = forbidden_points.reshape([n_f, 3])

    return measurements, points
    



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

    camera = Camera(x = 2.0, y = 0.0, z = -27.0, psi = 0.0, theta = 0.0, phi = 0.0, cx = 0.5, cy = 0.5, focaldistance = 10, sigmaCamera = 0.001)

    robot = Robot(50.0, 30.0, 30.0, 40, table, camera, fig, ax1, ax2, ax3)

    # reachable_points(robot)
    print(make_measurements(robot, False))


if __name__ == "__main__":
    main()