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


def divide_points(robot):
    """
    Divides the points on a table so you can get a test and a tarining family of points
    """
    plt.close('all')
    plt.figure(figsize = (8, 8))

    points_cal = np.array([])
    points_test = np.array([])

    number_of_test_points = 100
    x = int(len(robot.table.actualPoints)/number_of_test_points)

    for i, point in enumerate(robot.table.actualPoints):
        print(point)
        if i%x == 0:
            points_test = np.append(points_test, np.asarray([point]))
        else:
            points_cal = np.append(points_cal, np.asarray([point]))

    points_test = points_test.reshape([int(len(points_test)/3), 3]) 
    points_cal = points_cal.reshape([int(len(points_cal)/3), 3]) 

    for point in points_cal:

        if robot.fromCartesianToInner(point)[0] != False:  
            # Alcanzable
            plt.plot(point[0], point[1], 'og', label = 'Cal')
        else:
            plt.plot(point[0], point[1], 'or', label = 'Cal')


    for point in points_test:

        if robot.fromCartesianToInner(point)[0] != False:  
            # Alcanzable
            plt.plot(point[0], point[1], 'ob', label = 'Test')
        else:
            plt.plot(point[0], point[1], 'om', label = 'Test')


    plt.axhline(0, color='k', linestyle='--')
    plt.axvline(0, color='k', linestyle='--')
    plt.savefig('img/puntos_test_cal.png')
    plt.show()
    print('Img Saved')
    print(len(points_test))


def make_measurements(robot, print_points = False, print_forbidden_points = False):
    """
    CACA
    Measures position of all calibration points, taking pictures from a range of different angles.
    Excepting points that are close to R=60 or further.
    """

    forbidden_points = np.asarray([])
    measurements = np.asarray([])
    points = np.asarray([])
    camera_locations = np.asarray([])

    max_r = 0.9* (robot.R1 + robot.R2)

    for n, point in enumerate(robot.table.actualPoints[0:200]):
        if robot.fromCartesianToInner(point)[0] != False and np.linalg.norm(point) < max_r:    # Quitar en algun momento el  false y ver si puedo acceder todos

            # close_to_point = point
            close_to_point = [i + np.random.normal(0, 0.05) for i in point]

            for j in np.linspace(0, np.pi, 8):

                robot.cameraAim(close_to_point, j)

                camera_locations = np.append(camera_locations, robot.cameraPointing())

                j1, j2, jz = robot.currentPos.J1, robot.currentPos.J2, robot.currentPos.Jz     

                x, y = robot.point3DToCameraProjection(close_to_point)

                points = np.append(points, point)
                measurements = np.append(measurements, [j1, j2, jz, x, y])
            
        else:
            # print('Punto no accesible', point)
            forbidden_points = np.append(forbidden_points, point)
            continue

    n = int(len(points)/3)  # numero puntos 
    n_f = int(len(forbidden_points)/3)  # numero puntos prohibidos
    measurements = measurements.reshape([n, 5]) 
    points = points.reshape([n, 3])
    camera_locations = camera_locations.reshape([n, 3])
    forbidden_points = forbidden_points.reshape([n_f, 3])


    print('Check where camera points to: \n')
    print(f'XY Point ||| camera_locations ||| J1 J2 measurements ||| XY In Cam')

    for i, point in enumerate(points) :
        
        print(f'{point} ||| {camera_locations[i]} ||| {measurements[i][0:2]} |||{measurements[i][2:-1]}')


    if print_points:
        print('---Reachable Points---')
        print(f'Robot_config(J1,J2) \t||\t camera_meas(X,Y) \t||\t real_pos(x,y)')
        print(f' J1 \t \t J2 \t \t  Jz \t \t X \t \t Y \t \t  x \t \t y \t \t z')

        for i in range(n):
            print(measurements[i,0], measurements[i,1], measurements[i,2], measurements[i,3], 
                   measurements[i,3], '||', points[i,0], points[i,1])

        print('---Forbidden Points---')
        for point in print_forbidden_points:
            r = np.sqrt(point[0]**2 + point[1]**2)
            reach = robot.fromCartesianToInner(point)[0]  
            print(f'{point} || {r} || {reach}')
        # Entran en juego las tolerancias

    return measurements, points


def make_measurements_camera_pointing(robot, show: bool = False, N = 150):
    """
    Gets where the unitary vector of the camera points to, for every of all calibration points
    Excepting those that are close to R=60 or further.
    Input:
        N: number of points (5 measures for each)
    Output: 
        measurements: list of inner points 
        points: list of [x,y,z] for each point
        camera_measurements: [X,Y] projection of every point 
    """
    measurements = np.asarray([])
    camera_measurements = np.asarray([])
    points = np.asarray([])

    random_index = np.random.choice(np.asarray(robot.table.actualPoints).shape[0], size = N, replace=False)
    table_points = np.asarray(robot.table.actualPoints)[random_index]

    for n, point in enumerate(table_points):
        
        point = [point[0] + np.random.normal(0, 0.02), point[1] + np.random.normal(0, 0.02), point[2] + robot.camera.focusdistance]
        # point = [point[0] + 0.05, point[1] + 0.05, point[2]]

        if robot.fromCartesianToInner(point)[0] != False and np.linalg.norm(point) < 50:
            # Measure from different angles
            for j in np.linspace(0, 2*np.pi, 5): 
 
                robot.cameraAim(point, j)
                # robot.cameraAim(point, 0)

                x, y = robot.point3DToCameraProjection(point)

                measurements = np.append(measurements, robot.currentPos)

                camera_measurements = np.append(camera_measurements, np.array([x,y]))

                points = np.append(points, point)

                # print(robot.camera.cartesianpos.r[2] - point[2])
                # print('camera', robot.camera.cartesianpos.r[2])

                
        else:
            continue

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
            print(f'{measure.J1:.3f}, {measure.J2:.3f}, {measure.Z:.3f}, {measure.Jz:.3f}', '||', f'{camera[0]:.3f}, {camera[1]:.3f} ', '||', f'{points[i,0]:.2f}, {points[i,1]:.2f}, {points[i,2]:.2f}')
    return measurements, points, camera_measurements
    

def test_cameraAim(robot):

    v = [10,10,0]

    robot.cameraAim(v, 0)

    print(v)
    print(robot.cameraPointing())

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
    # camera = Camera(x = 0, y = 0.0, z = 0, psi = np.pi/150, theta =  np.pi/140, phi = np.pi/180, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)
    camera = Camera(x = 5, y = 0.0, z = 0, psi = 0, theta =  0.2, phi = 0, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)

    # Generate the robot
    robot = Robot(60.77, 38.0, 24.0, 34.0, table, camera, fig, ax1, ax2, ax3)
    


    # reachable_points(robot)

    # make_measurements(robot, False)

    # make_measurements_camera_pointing(robot, True)
    divide_points(robot)

    # test_cameraAim(robot)


if __name__ == "__main__":
    main()