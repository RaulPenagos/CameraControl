
from statsmodels.base.model import GenericLikelihoodModel
import numpy as np
import random
import copy

from src.Robot import *
from img.histograms import *
from src.Likelihoodxy import MyLikelihoodxy


"""
This Script includes a Likelihood function capable of finding the camera's parameters 
by taking pictures of the points and minimizing the positions. The parameters [x,y]_c, may 
not be the same as the real camera's ones, due to the inlcination angles. But the result will be
equivalent (totally fine with angles up to 10 º), as you will find in the bin charts used
to check simulated calibrations.

Run the calibration from Calibration.py

Cosas que se pueden configurar:
    -Parámetros del robot real, en una simulación de la realidad
    -Parametros inciales de la estimación
    -Numero de puntos para calibrar y testear
    -Numero de orientaciones para hacer approach a cada punto
    -Hacer bin charts 


Author: Raul Penagos
Date: February 12th 2025
"""


class RealitySimulation():
    """
    Simulates the real environment where you have a robot, and make measurements for 
    several points of the table. 
    """
    
    def __init__(self):
        """
        Defines and crates the real_robot(). And sets the points for calibration and test 
        with the divide_points() function
        """
        self.real_robot = self.create_real_robot()
        self.table = self.real_robot.table
        self.camera = self.real_robot.camera

        self.points_cal = np.array([])
        self.points_test = np.array([])
        # self.divide_points(20, 60)  
        self.divide_points(400, 10, True)  
        # self.divide_points(200)  # 200 test_points make good bin plots
        print(f'N Points cal: \t {len(self.points_cal)}')
        print(f'N Points test: \t {len(self.points_test)}')
    
    def create_real_robot(self):

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
        camera = Camera(x = 5.123 , y = 0.078, z = 10, psi = math.pi/180*1.7, theta = - math.pi/180*0.48, phi = math.pi/180*0.96, cx = -1, cy = -1, focaldistance = 20, sigmaCamera = 0.001)
        # camera = Camera(x = 5.0 , y = 0.0, z = 10.0, psi = 0.01, theta = 0, phi = 0, cx = 1.0, cy = 1, focaldistance = 20, sigmaCamera = 0.001)

        # Generate the robot: same size as our SCARA
        robot = Robot(60.77, 38.0, 24.0, 34.0, table, camera, fig, ax1, ax2, ax3)

        return robot
        
    def divide_points(self, number_of_test_points = 200, number_of_cal_points = 20, show: bool = False):
        """
        Args:
            number_of_cal_points:  the points you are using to calibrate the robot
            number_of_test_points:  the points you are using to test the calibration
            show: make a plot with the cal and test points on the table.
        """
        actualPoints = [point for point in self.real_robot.table.actualPoints if self.real_robot.fromCartesianToInner(point)[0] != False and np.linalg.norm(point) < 50]

        if (number_of_test_points + number_of_cal_points) > len(actualPoints):
            number_of_cal_points = int(4/5 * len(actualPoints) - 1 ) 
            number_of_test_points = int(1/5 * len(actualPoints) - 1) 

        # n random indexes to get n random total points
        random_index = np.random.choice(np.asarray(actualPoints).shape[0], size = (number_of_cal_points + number_of_test_points), replace=False)
        chop = np.asarray(actualPoints)[random_index]
        
        self.points_test, self.points_cal, _ = select_uniform_elements(chop, number_of_test_points)

        # show = True

        if show:
            # Make CoOl Graph showing Test and Cal Points, and those that can't be reached
            plt.figure(figsize = (8, 8))

            impossible_points = [point for point in self.real_robot.table.actualPoints if self.real_robot.fromCartesianToInner(point)[0] == False or np.linalg.norm(point) > 50]
            for point in impossible_points:
                plt.plot(point[0], point[1], 'ok', markersize = 10,  label = 'No alcanzable')

            for point in self.points_test: # color = 'magenta',
                plt.plot(point[0], point[1], 'or', markersize = 10,  label = 'Punto Test')

            for point in self.points_cal:
                plt.plot(point[0], point[1], 'ob', markersize = 10,  label = 'Punto Cal.')

            plt.axhline(0, color='k', linestyle='--')
            plt.axvline(0, color='k', linestyle='--')
            plt.savefig('img/likelihood_points.png')
            plt.legend()
            print('Img Saved')
     
    def make_camera_measurements(self, show: bool = False):
        """
        Goes over all cal_points, and points them with the camera of the robot, saving the 
        needed coordinates. Takes pictures of the point approaching from different angles,
        this way you get more calibration data for just one point in the desk.

        Input:
            show: show a table with the points on shell
        Output: 
            measurements: list of inner points 
            points: list of [x,y,z] for each point
            camera_measurements: [X,Y] projection of every point on cameras CCD
        """

        measurements = np.asarray([])
        camera_measurements = np.asarray([])
        points = np.asarray([])

        for n, point in enumerate(self.points_cal):
            
            # define the point where we want to move the robot (cameraAim takes into account 
            # focal distance apart from the point)
            point = [ point[0] + random.normalvariate(0, 0.01),
                     point[1] + random.normalvariate(0, 0.01),
                     point[2]]

            if self.real_robot.fromCartesianToInner(point)[0] != False and np.linalg.norm(point) < 50:

                # Take pictures of the point approaching from different angles. 5 positions.
                for j in np.linspace(0, 2*np.pi, 5): 
    
                    # robot aims the point
                    self.real_robot.cameraAim(point, 0)

                    # take picture of the point
                    x, y = self.real_robot.point3DToCameraProjection(point)

                    # save measurements: current innerposition of the robot
                    measurements = np.append(measurements, self.real_robot.currentPos)

                    # print( 'p:',self.real_robot.currentPos.J1,
                    #        self.real_robot.currentPos.J2,
                    #         self.real_robot.currentPos.Z,
                    #          self.real_robot.currentPos.Jz)

                    # save picture: save the XY of the projection of the point
                    camera_measurements = np.append(camera_measurements, np.array([x,y]))

                    # save the point: x,y,z on the table
                    points = np.append(points, point)

            else:

                continue
        
        # Total number of points
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


class Calibration():
    def __init__(self, calibrate:bool = True):
        """
        Args:
            Calibrate: wheter you want to make calibration (Likelihood) 
                       or directly use the parameters data from dictionary.
        """

        self.reality = RealitySimulation()
        self.robot1 = self.reality.real_robot  # real robot
        self.robot2 = self.create_robot_estimation(5.00, 0.0, 10.0, 0, 0, 0) # estimation robot

        # Simulo las medidas reales, con robot1 real
        measurements, points, camera_measurements = self.reality.make_camera_measurements(False)

        self.measurements = measurements
        self.camera_measurements = camera_measurements
        self.real_points = points

        self.points_cal = self.reality.points_cal
        self.points_test = self.reality.points_test
        # print(len(self.points_cal), len(self.real_points))

        n = len(self.real_points)

        self.calibration_params = {'x': None, 'y': None}

        calibrate = False
        if calibrate:
            self.calibration_params['x'], self.calibration_params['y'] = self.calibratePos(True)


    def create_robot_estimation(self, x = 5.0, y = 0.0, z = 10.0, psi = 0.00, theta =  0.00, phi = 0.00):
        """
        Args: the initial parameters for the estimation of the camera position and orientation
        """
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
        camera = Camera(x, y, z, psi, theta, phi, cx = 1, cy = 1, focaldistance = 20, sigmaCamera = 0.001)

        # Generate the robot
        robot = Robot(60.77, 38.0, 24.0, 34.0, table, camera, fig, ax1, ax2, ax3)

        return robot

    def calibratePos(self, show = True):
        print('\n Calibrating Position...\n')

        # Crear modelo likelihood
        cal = MyLikelihoodxy(self.real_points, [self.measurements, self.camera_measurements], self.robot2)
        
        # Ajustar modelo
        results = cal.fit() 

        if show:
            # Mostrar resultados de la minimizacion
            print("\n Full results summary:\n")
            print(results.summary(xname = ['x','y']))

        print('\n Calibrated Robot Parameters:\n')
        print(f'x: {cal.robot.camera.r0[0]:.4f}\t y: {cal.robot.camera.r0[1]:.4f}\t z: {cal.robot.camera.r0[2]:.4f}\n ')
        print(f'psi: {cal.robot.camera.rotation0.psi:.4f}\t theta: {cal.robot.camera.rotation0.theta:.4f}\t phi: {cal.robot.camera.rotation0.phi:.4f}\n')
        # No tienen porque ser iguales a los del robot real, pero las medidas que dan serán equivalentes 

        return [results.params[0], results.params[1]]
     
    def test_calibration(self):
        """
        Check and make some cool charts, to see if the calibrated calibration is well donde.
        Compares robot1 and robot2 over the test_points. 
        """
        
        # Actualizo robot2 con los parametros nuevos
        self.robot2.camera.r0[0] = self.calibration_params['x']
        self.robot2.camera.r0[1] = self.calibration_params['y']


        # print('\n Calibrated Robot 1:\n')
        # print(f'x: {self.robot1.camera.r0[0]:.4f}\t y: {self.robot1.camera.r0[1]:.4f}\t z: {self.robot1.camera.r0[2]:.4f}\n ')
        # print(f'psi: {self.robot1.camera.rotation0.psi:.4f}\t theta: {self.robot1.camera.rotation0.theta:.4f}\t phi: {self.robot1.camera.rotation0.phi:.4f}\n')
        # print('\n Calibrated Robot 2:\n')
        # print(f'x: {self.robot2.camera.r0[0]:.4f}\t y: {self.robot2.camera.r0[1]:.4f}\t z: {self.robot2.camera.r0[2]:.4f}\n ')
        # print(f'psi: {self.robot2.camera.rotation0.psi:.4f}\t theta: {self.robot2.camera.rotation0.theta:.4f}\t phi: {self.robot2.camera.rotation0.phi:.4f}\n')
        
    
        # Mido los puntos test utilizando el robot1
        # Guardo la proyección en la camara (X,Y), el punto (x,y,z) y la innerposition (J1,J2,Jz,Z)

        points = np.array([])  # x,y,z
        innerpoints = np.array([])  # J1,J2,Jz,Z
        pictures = np.array([])  # X,Y
        
        for point in self.points_test:

            self.robot1.cameraAim(point, 0)

            # save the point: x,y,z on the table
            points = np.append(points, point)

            # take picture of the point
            x, y = self.robot1.point3DToCameraProjection(point)
            pictures = np.append(pictures, [x, y]) 

            # save the position of the robot
            innerpoints = np.append(innerpoints, self.robot1.currentPos)

        # Reshape output
        k = int(len(innerpoints))
        points = points.reshape([k, 3]) 
        pictures = pictures.reshape([k, 2]) 

        # Go over the points with the new calibrated robot now #######################
        DELTAX = np.array([])
        DELTAY = np.array([])

        for i, inner in enumerate(innerpoints):
            # Move Robot2 to the same innerposition
            self.robot2.MoveRobotTo(inner) 

            # Take picture of the point
            x2, y2 = self.robot2.point3DToCameraProjection(points[i])
            dx = pictures[i][0] - x2 
            dy = pictures[i][1] - y2

            # dx & dy are measurements on the cameras CCD, now scale it up to the table:
            dx, dy, _ =  self.robot2.cameraProjectionToPoint3D([0, 0]) -self.robot2.cameraProjectionToPoint3D([dx, dy]) 

            DELTAX = np.append(DELTAX, dx)
            DELTAY = np.append(DELTAY, dy)

        # print(len(DELTAX), max(DELTAX), min(DELTAX), np.mean(DELTAX))
        # print(len(DELTAY), max(DELTAY), min(DELTAY), np.mean(DELTAY))

        # MAke Bin charts for the deviation DELTAX and DELTAY ########################################

        binchart_fit(DELTAX, r'$x$', 'xfit')
        binchart_fit(DELTAY, r'$y$', 'yfit')
        print('Plot Charts Done!')

def select_uniform_elements(lst, x):
    """
    Gets x uniform distributed elements from lst
    """
    L = len(lst)
    # Calculate uniformly spaced indices
    indices = [int(i * L / x) for i in range(x)]
    indices_set = set(indices)  # For faster lookup

    # Create the two lists: selected and the rest
    selected = [lst[i] for i in indices]
    rest = [lst[i] for i in range(L) if i not in indices_set]

    return selected, rest, indices


    
def main():
    cal = Calibration()
    cal.test_calibration()




if '__name__' == main():
    main()


