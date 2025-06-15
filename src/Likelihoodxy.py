from statsmodels.base.model import GenericLikelihoodModel
import numpy as np
import random
import copy


from src.Robot import *
from img.histograms import *
from src.Likelihoodxy import *

"""
This Script includes a Likelihood function capable of finding camera's r0(x,y) parameters 
by taking pictures of the points and minimizing the difference in their positions. The parameters 
[x,y]_c, may not be the same as the real camera's ones, due to the pitch/roll/yaw angles. But 
the result will be equivalent (works totally fine with angles up to 10 º), as you will find 
in the bin charts used to check simulated calibrations.


Author: Raul Penagos
Date: February 12th 2025
"""


class MyLikelihoodxy(GenericLikelihoodModel):

    def __init__(self, endog, exog, robot, **kwds):
        """
        Optimizes robot's camera r0, only (x,y). To replicate the results
        taken with another 'real' robot -> Endog and Exog.
        
        Input:
            endog (array):  reference postions [[X1, Y1, Z1], [X2, Y2, Z2], ...] for each point
            exog (array): measurements with the real robot [innerpoint_1, [X, Y]_1, innerpoint_2, [X, Y]_2, ....] 
            robot: robot used as a first estimation for the parameters of the real one.
            **kwds: other params. (void)

        Example of Use:
            cal = MyLikelihoodxy(self.real_points, [self.measurements, self.camera_measurements], self.robot2)
    
            # Fit the model
            results = cal.fit() 

        """
        self.n = int(len(endog))
        self.endog = np.asarray(endog)
        J_points, camera_points = exog 
        self.cPoints_Jpoints = J_points
        self.cPoints_CameraPoints = camera_points
        self.rPoints = np.copy(self.endog)
 
    
        print('N points: ', self.n)

        exog = np.asarray([])

        for i in range(self.n):
            x = J_points[i].J1, J_points[i].J2, J_points[i].Z, J_points[i].Jz, camera_points[i][0], camera_points[i][1]
            exog = np.append(exog, x)

        self.exog = exog.reshape([self.n, 6])

        self.robot = robot

        super(MyLikelihoodxy, self).__init__(self.endog, self.exog, self.loglike, **kwds) 


    def loglike(self, params):
        # Update camera parameters on each iteration X,Y
        self.robot.camera.r0[0] = params[0]
        self.robot.camera.r0[1] = params[1]

        # Compute Log-Like value
        chi2 = 0.0

        measurements = np.asarray([]) # I measure the X Y 
        # pointings = np.asarray([]) # I measure the xyz where the camera is pointing to

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
        # Only 'powell' method converges and get low chi**2 -> Use POWELL

        if start_params is None:
            # Set initial values for the parameters to optimize
            start_params = [self.robot.camera.r0[0], self.robot.camera.r0[1]]
            
        # Call the parent class's fit method
        return super(MyLikelihoodxy, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)
    

