import math
import numpy as np
import matplotlib.pyplot as plt
import sys

from src.Table import Table
from src.Camera import Camera
from src.Robot import Robot
from src.Likelihood import *



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


    def reachable_points(robot: Robot, save_fig: bool):
        """
        Plots the reachable and un-reachable points on the board 
        for a given robot
        """
        puntos_no_accesibles_x, puntos_no_accesibles_y  = np.array([]), np.array([])
        puntos_accesibles_x, puntos_accesibles_y  = np.array([]), np.array([])

        for n, point in enumerate(table.actualPoints):

            # quito los puntos donde el discriminate es < 0 
            if robot.fromCartesianToInner(point)[0] != False:  

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

        # Plot puntos accesibles/innacesibles por el brazo del robot
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


    def make_measurements(robot: Robot, print: bool):
        """
        Simula la toma de medidas de la posición de los puntos de la mesa.
        Programa que coge todos los puntos definidos en la mesa, mueve el robot a cad uno
        los ve con la cámara y toma una foto (obtiene las coordenadas xy en el CCD) 
        Imprime por pantalla:
            La configuración del robot: J1 y J2
            Medidas de la cámara: X, Y 
            Posición real (por construcción en la mesa) de ese punto: x, y
        """
        if print:

            print(f'Robot_config(J1,J2) \t||\t camera_meas(X,Y) \t||\t real_pos(x,y)')
            print(f' J1 \t \t J2 \t \t  X \t \t Y \t \t  x \t \t y \t \t z')

        point_list = np.asarray([])  # save the configuration of the point in the camera and robot for every point
        real_point_list = np.asarray([])  # save the real position x,y,z of every point

        for n, point in enumerate(robot.table.actualPoints):

            # Para los puntos accesibles (discriminate es >= 0)
            if robot.fromCartesianToInner(point)[0] != False:   
                    
                # # A) Move the robot to the point 
                # robot.cartesianMoveTo(point, 0) # 0: no modifico orientación émbolo 

                # B) Move the robot close to the point 
                close_to_point = np.asarray([round(i+np.random.uniform(0,2)) for i in point])
                robot.cartesianMoveTo(close_to_point, 0) # 0: no modifico orientación émbolo 

                # Get the robot's parameters
                robot_config = np.asarray([robot.J1, robot.J2])

                # Take picture: point's position in camera CCD
                camera_meas = robot.point3DToCameraProjection(point)[0:2]

                # The real position
                real_pos = table.points[n]

                point_list = np.append(point_list, camera_meas)
                point_list = np.append(point_list, robot_config)
                real_point_list = np.append(real_point_list, real_pos)

                if print:

                    robot_config_str = '\t'.join([str(i) for i in robot_config])
                    camera_meas_str = '\t'.join([str(i) for i in camera_meas])
                    real_pos_str = '\t'.join([str(i) for i in real_pos])

                    # print(f'{robot_config} \t {camera_meas} \t {real_pos}')
                    print(f'{robot_config_str} \t {camera_meas_str} \t {real_pos_str}')

        # To be used as endog, exog
        # point list: [x1, y1, J11, J21, x2, y2, J21, J22, ....]
        return (point_list, real_point_list)
    

    make_measurements(robot)





    