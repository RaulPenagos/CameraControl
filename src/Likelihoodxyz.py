from statsmodels.base.model import GenericLikelihoodModel
import numpy as np
import copy
import sys
import os
import random

# Get the root directory of the project
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# Add the test/ directory explicitly
sys.path.append(os.path.join(project_root, "test"))

# Now, import the function from makeMeasurements.py
from makeMeasurements import *
from src.Robot import *
from img.histograms import *


"""
This Script includes a Likelihood function capable of finding the camera's parameters 
by taking pictures of the points and minimizing the positions

Author: Raul Penagos
Date: February 12th 2025
"""


class MyLikelihoodxy(GenericLikelihoodModel):

    def __init__(self, endog, exog, robot, **kwds):
        """
        Optimizes x and y 
        endog (array):  reference postions [X1, Y1, Z1, X2, Y2, Z2, ...] for each point
        exog (array): measurements [innerpoint, [X, Y]]   innerpoint(J1, J2, Z, Jz) 
        robot: robot cuyos parametros quiero optimizar
        **kwds: parametros iniciales (~ reales) a partir de los cuales minimizo
        """
        self.n = int(len(endog))
        self.endog = np.asarray(endog)
        J_points, camera_points = exog 
        self.cPoints_Jpoints = J_points
        self.cPoints_CameraPoints = camera_points
        self.rPoints = np.copy(self.endog)

        #   
    
        print('N points: ', self.n)

        exog = np.asarray([])

        for i in range(self.n):
            x = J_points[i].J1, J_points[i].J2, J_points[i].Z, J_points[i].Jz, camera_points[i][0], camera_points[i][1]
            exog = np.append(exog, x)

        self.exog = exog.reshape([self.n, 6])

        self.robot = robot
        # self.sigma2 = self.robot.sigmaCamera**2 + self.robot.sigmaRobot**2

        super(MyLikelihoodxy, self).__init__(self.endog, self.exog, self.loglike, **kwds)  # self.loglike añadido


    def loglike(self, params):
        # Update camera parameters
        self.robot.camera.r0[0] = params[0]
        self.robot.camera.r0[1] = params[1]

        chi2 = 0.0

        measurements = np.asarray([]) # I measure the X Y 
        # pointings = np.asarray([]) # I measure the xyz 

        for i, Jpoint in enumerate(self.cPoints_Jpoints):

            self.robot.MoveRobotTo(Jpoint)

            X, Y = self.robot.point3DToCameraProjection(self.endog[i])

            measurements = np.append(measurements, [X,Y])
            # pointings = np.append(pointings, self.robot.cameraPointing())

        measurements = measurements.reshape([int(len(measurements)/2), 2])  
        # pointings = pointings.reshape([int(len(pointings)/3), 3])  

        for i in range(0, self.n):

            new_measure = measurements[i]
            real_measure = self.cPoints_CameraPoints[i]

            # point = self.rPoints[i][0:2]
            # pointed_point = pointings[i][0:2]

            chi2 += np.linalg.norm(new_measure-real_measure)**2 # + np.linalg.norm(point-pointed_point)**2 
    
        print(f'CHI2: {chi2}', end = "\r")

        return -chi2


    def fit(self, start_params=None, method='powell', maxiter=10000, **kwargs):
        # methods = bfgs, lbfgs, nm, newton, powell, cg, ncg, basinhopping, minimize

        if start_params is None:
            start_params = [self.robot.camera.r0[0], self.robot.camera.r0[1]]
            
        # Call the parent class's fit method

        return super(MyLikelihoodxy, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)
    

class RealitySimulation():
    """
    delta(x,y) = 0.01 mm   delta(z) = 0.004 mm
    """
    
    def __init__(self):
        self.real_robot = self.create_real_robot()
        self.table = self.real_robot.table
        self.camera = self.real_robot.camera

        self.points_cal = np.array([])
        self.points_test = np.array([])
        self.divide_points(200)
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
        camera = Camera(x = 5.0, y = 0, z = 3.6, psi = 0.003, theta =  -0.003, phi = 0.002, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)
        # camera = Camera(x = 5.0, y = 1, z = 3.6, psi = -0.017, theta =  -0.010, phi = 0.019, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)
        # camera = Camera(x = 5.0, y = 0, z = 3.6, psi = 0.03, theta =  -0.04, phi = -0.01, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)
        camera = Camera(x = 5.0, y = 2, z = 3.6, psi = math.pi/180*5, theta = - math.pi/180*4, phi = math.pi/180*3.5, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)

        # Generate the robot
        robot = Robot(60.77, 38.0, 24.0, 34.0, table, camera, fig, ax1, ax2, ax3)

        return robot
        
    def divide_points(self, number_of_test_points = 200, number_of_cal_points = 3000, show: bool = False):
        """
        Args:
            number_of_cal_points:
            number_of_test_points:
        """
        actualPoints = [point for point in self.real_robot.table.actualPoints if self.real_robot.fromCartesianToInner(point)[0] != False and np.linalg.norm(point) < 50]

        if number_of_cal_points > len(actualPoints):
            number_of_cal_points = int(len(actualPoints)) 
        
        if number_of_test_points > number_of_cal_points:
            number_of_test_points = int(number_of_cal_points / 20)

        # n random indexes to get n random total points
        random_index = np.random.choice(np.asarray(actualPoints).shape[0], size = number_of_cal_points, replace=False)
        chop = np.asarray(actualPoints)[random_index]
        
        
        self.points_test, self.points_cal, _ = select_uniform_elements(chop, number_of_test_points)


        if show:
            plt.figure(figsize = (8, 8))
            for point in self.points_cal:

                if self.real_robot.fromCartesianToInner(point)[0] != False:  
                    # Alcanzable
                    plt.plot(point[0], point[1], 'og', label = 'Cal')
                else:
                    plt.plot(point[0], point[1], 'or', label = 'Cal')


            for point in self.points_test:

                if self.real_robot.fromCartesianToInner(point)[0] != False:  
                    # Alcanzable
                    plt.plot(point[0], point[1], 'ob', label = 'Test')
                else:
                    plt.plot(point[0], point[1], 'om', label = 'Test')


            plt.axhline(0, color='k', linestyle='--')
            plt.axvline(0, color='k', linestyle='--')
            plt.savefig('img/likelihood_points.png')
            print('Img Saved')

    def make_camera_measurements(self, show: bool = False):
        """
        Gets where the unitary vector of the camera points to, for every of all calibration points
        Excepting those that are close to R=60 or further.
        Output: 
            measurements: list of inner points 
            points: list of [x,y,z] for each point
            camera_measurements: [X,Y] projection of every point 
        """
        # print('points_cal', self.points_cal)
        # print('points_test', self.points_test)
        measurements = np.asarray([])
        camera_measurements = np.asarray([])
        points = np.asarray([])

        for n, point in enumerate(self.points_cal):
            
            # define the point where we want to move the robot (cameraAim takes into account 
            # focal distance apart from the point)
            point = [point[0] + random.normalvariate(0, 0.01),
                     point[1] + random.normalvariate(0, 0.01),
                     point[2]]

            if self.real_robot.fromCartesianToInner(point)[0] != False and np.linalg.norm(point) < 50:

                # Measure from different angles
                for j in np.linspace(0, 2*np.pi, 5): 
    
                    # robot aims the point
                    self.real_robot.cameraAim(point, 0)

                    # take picture of the point
                    x, y = self.real_robot.point3DToCameraProjection(point)

                    # save measurements: current innerposition of the robot
                    measurements = np.append(measurements, self.real_robot.currentPos)

                    # save picture: save the XY of the projection of the point
                    camera_measurements = np.append(camera_measurements, np.array([x,y]))

                    # save the point: x,y,z on the table
                    points = np.append(points, point)

                    # print(robot.camera.cartesianpos.r[2] - point[2])
                    # print('camera', robot.camera.cartesianpos.r[2])

            else:
                
                continue
        
        # total number of points
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
            Calibrate: wheter make calibration or directly use data from dictionary
        """

        self.reality = RealitySimulation()
        self.robot1 = self.reality.real_robot 
        self.robot2 = self.create_robot_estimation(6, 4, 3.6, 0, 0, 0)

        # print(self.robot1.camera.rotation0.psi,self.robot1.camera.rotation0.theta, self.robot1.camera.rotation0.phi)

        # Simulo las medidas reales, con robot1 real
        measurements, points, camera_measurements = self.reality.make_camera_measurements(False)


        self.measurements = measurements
        self.camera_measurements = camera_measurements
        self.real_points = points

        self.points_cal = self.reality.points_cal
        self.points_test = self.reality.points_test
        # print(len(self.points_cal), len(self.real_points))

        n = len(self.real_points)

        self.calibration_params = {'x': 4.9920, 'y': 0.9603}

        # calibrate = False
        if calibrate:
            self.calibration_params['x'], self.calibration_params['y'] = self.calibratePos()

        # print(camera_measurements)

    def create_robot_estimation(self, x = 5.0, y = 0, z = 3.6, psi = 0.00, theta =  0.00, phi = 0.00):
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
        camera = Camera(x, y, z, psi, theta, phi, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)

        # Generate the robot
        robot = Robot(60.77, 38.0, 24.0, 34.0, table, camera, fig, ax1, ax2, ax3)

        return robot

    def calibratePos(self, show = False):
        print('\n Calibrating Position...\n')

        # Crear modelo likelihood
        # print(len(self.points_cal), len(self.measurements), len(self.camera_measurements))
        # print(self.measurements[0])
        cal = MyLikelihoodxy(self.real_points, [self.measurements, self.camera_measurements], self.robot2)
        
        # Ajustar modelo
        results = cal.fit() 

        if show:
            # Mostrar resultados
            print("\n Full results summary:\n")
            print(results.summary(xname = ['x','y']))

        print('\n Calibrated Robot:\n')
        print(f'x: {cal.robot.camera.r0[0]:.4f}\t y: {cal.robot.camera.r0[1]:.4f}\t z: {cal.robot.camera.r0[2]:.4f}\n ')
        print(f'psi: {cal.robot.camera.rotation0.psi:.4f}\t theta: {cal.robot.camera.rotation0.theta:.4f}\t phi: {cal.robot.camera.rotation0.phi:.4f}\n')


        return [results.params[0], results.params[1]]
    
    def test_calibration(self):

        # Actualizo robot2 con los parametros nuevos
        self.robot2.camera.r0[0] = self.calibration_params['x']
        self.robot2.camera.r0[1] = self.calibration_params['y']

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

        # Go over the points with the new calibrated robot #######################
        DELTAX = np.array([])
        DELTAY = np.array([])

        for i, inner in enumerate(innerpoints):
            # Move Robot2 to the same innerposition
            self.robot2.MoveRobotTo(inner) 

            # Take picture of the point
            x2, y2 = self.robot2.point3DToCameraProjection(points[i])
            dx = pictures[i][0] - x2 
            dy = pictures[i][1] - y2

            # dx & dy are measurements on the cameras CCD, now scale it up:
            dx, dy, _ = self.robot2.cameraProjectionToPoint3D([0, 0, 0]) - self.robot2.cameraProjectionToPoint3D([dx, dy, 0])

            DELTAX = np.append(DELTAX, dx)
            DELTAY = np.append(DELTAY, dy)

        
        
        # print(len(DELTAX), max(DELTAX), min(DELTAX), np.mean(DELTAX))
        # print(len(DELTAY), max(DELTAY), min(DELTAY), np.mean(DELTAY))

        # print('DELTAX', DELTAX)
        # print('DELTAY', DELTAY)

        # Evaluate with bin chart for x & y ########################################

        # binchart(DELTAX, r'$x$')
        # binchart(DELTAY, r'$y$')
        print('Plot Charts Done!')
        binchart_fit(DELTAX, r'$x$', 'xfit')
        binchart_fit(DELTAY, r'$y$', 'yfit')

   

    # binchart(num, r'$\theta$')

def select_uniform_elements(lst, x):
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
