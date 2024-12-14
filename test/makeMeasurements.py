import math
import numpy as np
import matplotlib.pyplot as plt
import sys

from src.Table import Table
from src.Camera import Camera
from src.Robot import Robot





if __name__ == "__main__":

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


    # Generate the table
    table = Table(0.01, 0.0)  #  (tolerance, z)

    #  Generate the camera
    camera = Camera(x = 2.0, y = 0.0, z = -27.0, psi = 0.0, theta = 0.0, phi = 0.0, cx = 0.5, cy = 0.5, focaldistance = 10, sigmaCamera = 0.001)

    # Generate the robot
    robot = Robot(50.0, 30.0, 30.0, 40, table, camera, fig, ax1, ax2, ax3)



    # #  test
    # status, j1, j2, Z = robot.fromCartesianToInner([12,-49,0]) # con 20,20,0 va bien
    # print(status, j1, j2, Z )


    def reachable_points(robot: Robot, save_fig: bool):
        """
        Plots the reachable and un-reachable points on the board 
        for a given robot
        """

        puntos_no_accesibles_x = np.array([])
        puntos_no_accesibles_y = np.array([])
        puntos_accesibles_x = np.array([])
        puntos_accesibles_y = np.array([])

        for n, point in enumerate(table.actualPoints):

            # print(point)
            if robot.fromCartesianToInner(point)[0] != False:  # quito los puntos donde el discriminate es < 0 

                puntos_accesibles_x = np.append(puntos_accesibles_x, point[0])
                puntos_accesibles_y = np.append(puntos_accesibles_y, point[1])
                # Puntos que cumplean condicion
                # print('--- OK', np.sqrt(np.sum([i**2 for i in point])))
                
            else:

                puntos_no_accesibles_x = np.append(puntos_no_accesibles_x, point[0])
                puntos_no_accesibles_y = np.append(puntos_no_accesibles_y, point[1])
                                                  
                # Puntos que no están al alcance del robot
                # if np.sqrt(np.sum([i**2 for i in point])):
                #     print('braci corto: ' np.sqrt(np.sum([i**2 for i in point])))
                    

        #  Plot puntos aceesibles/innacesibles por el brazo del robot
        # Todos los puntos innacesibles están más lejos de los 60 cm de alcance del brazo
        fig = plt.figure(figsize = (8, 8))
        plt.plot(puntos_no_accesibles_x, puntos_no_accesibles_y, 'or',  label = 'unreachable')
        plt.plot(puntos_accesibles_x, puntos_accesibles_y, 'og', label = 'reachable')
        plt.axhline(0, color='k', linestyle='--')
        plt.axvline(0, color='k', linestyle='--')
        plt.legend()
        if save_fig:
            plt.savefig('puntos.png')
        plt.show()

    # reachable_points(robot, save_fig = True)



    """
    1.- Hacer un programa que coja todos los puntos definidos en Table.py
    y que le diga al robot que vaya a esos puntos para verlos con la cámara
    y entonces tienes que imprimir por pantalla
    La configuración del robot: J1 y J2, las medidas X, Y de la cámara y también la posición real (medida) de ese punto (sacada de Table.py)
    """



    # print(f'Robot_config(J1,J2) \t||\t camera_meas(X,Y) \t||\t real_pos(x,y)')
    # print(f' J1 \t \t J2 \t \t  X \t \t Y \t \t  x \t \t y')

    # for n, point in enumerate(table.actualPoints):

    #     # quito los puntos donde el discriminate es < 0
    #     if robot.fromCartesianToInner(point)[0] != False:   
                 
    #         # Move the robot to the point 
    #         robot.cartesianMoveTo(point, 0) # 0: no modifico orientación émbolo 

    #         # Get the robot's parameters
    #         robot_config = np.asarray([robot.J1, robot.J2])

    #         # Point's position in camera CCD
    #         # camera_meas = np.asarray(robot.fromInnerToCartesian()+ robot.camera.r0)[0:2]
    #         camera_meas = robot.point3DToCameraProjection(point)[0:2]

    #         # Problemas con point3DToCameraProjection()?

    #         # The real position (measured in the lab by us)
    #         real_pos = table.points[n]

    #         robot_config_str = '\t'.join([str(i) for i in robot_config])
    #         camera_meas_str = '\t'.join([str(i) for i in camera_meas])
    #         real_pos_str = '\t'.join([str(i) for i in real_pos])

    #         # print(f'{robot_config} \t {camera_meas} \t {real_pos}')
    #         print(f'{robot_config_str} \t {camera_meas_str} \t {real_pos_str}')
    #     else:
    #         pass


    # Testing point3DToCameraProjection

    # point = np.array([25, 25, 0])

    # robot.cartesianMoveTo(point, 0)
    # print(robot.point3DToCameraProjection(point)[0:2])



    # Second testing

    for n, point in enumerate(table.actualPoints):

        if robot.fromCartesianToInner(point)[0] != False:   

            robot.cartesianMoveTo(point, 0)
            # print(robot.ux, robot.uy, robot.uz)
            print('J:', robot.J1, robot.J2)
            print(robot.point3DToCameraProjection(point)[0:2])

    