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


from statsmodels.stats.outliers_influence import variance_inflation_factor  # check colinearity

"""
This Script includes a Likelihood function capable of finding the camera's parameters 
by taking pictures of the points and minimizing the positions

Author: Raul Penagos
Date: February 12th 2025
"""

# xyz psi,theta,phi original (igual que la sigueinte)
class MyLikelihood(GenericLikelihoodModel):

   def __init__(self, endog, exog, robot, **kwds):
      """
      Optimizes x , y and z
      endog (array):  reference postions [[X1, Y1, Z1], [X2, Y2, Z2], ...] for each point
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
 
      print('N points: ', self.n)

      exog = np.asarray([])

      for i in range(self.n):
         x = J_points[i].J1, J_points[i].J2, J_points[i].Z, J_points[i].Jz, camera_points[i][0], camera_points[i][1]
         exog = np.append(exog, x)

      self.exog = exog.reshape([self.n, 6])

      self.robot = robot
      # self.sigma2 = self.robot.sigmaCamera**2 + self.robot.sigmaRobot**2

      super(MyLikelihood, self).__init__(self.endog, self.exog, self.loglike, **kwds)  # self.loglike añadido


   def loglike(self, params):

      self.robot.camera.r0 = np.asarray([params[0], params[1], params[2]])
      self.robot.camera.rotation0 = EulerRotation(params[3],params[4],params[5]) # pitch, roll y yaw


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
         start_params = [self.robot.camera.r0[0], self.robot.camera.r0[1], self.robot.camera.r0[2],
                         self.robot.camera.rotation0.psi, self.robot.camera.rotation0.theta,
                         self.robot.camera.rotation0.phi]
        
      # Call the parent class's fit method

      return super(MyLikelihood, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)

# xyz psi,theta,phi  
class MyLikelihoodAll(GenericLikelihoodModel):

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

      super(MyLikelihoodAll, self).__init__(self.endog, self.exog, self.loglike, **kwds)  # self.loglike añadido


   def loglike(self, params):
      # Update camera parameters
      self.robot.camera.r0[0] = params[0]
      self.robot.camera.r0[1] = params[1]
      self.robot.camera.r0[2] = params[2]
      self.robot.camera.rotation0 = EulerRotation(params[2],params[3],params[4]) 


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
         start_params = [self.robot.camera.r0[0], self.robot.camera.r0[1], self.robot.camera.r0[2],
                           self.robot.camera.rotation0.psi,
                           self.robot.camera.rotation0.theta, self.robot.camera.rotation0.phi]
         
      # Call the parent class's fit method

   #   return super(MyLikelihoodAll, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)
   


      if method == 'basinhopping':

         niter_success = 9
         step_sizes = [0.01, 0.01, 0.01, 0.001, 0.001, 0.001] 
         # take_step = CustomStep(step_sizes)
         T = 0.5   # controla la probabilidad de aceptar soluciones peores para escapar de mínimos locales (0.1 - 1)
         
         bounds = [(-5, 5), (-5, 5), (-10, 10), (-0.35, 0.35), (-0.35, 0.35), (-0.35, 0.35)]
         minimizer_kwargs = {"method": "L-BFGS-B", "bounds": bounds}

         return super(MyLikelihoodAll, self).fit(start_params=start_params, method=method, niter=maxiter, niter_success=niter_success,
                                             T=T,  stepsize=0.0, minimizer_kwargs = minimizer_kwargs, **kwargs)
      
      else:

         return super(MyLikelihoodAll, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)


# xy psi,theta,phi
class MyLikelihoodAllbutZ(GenericLikelihoodModel):

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

        super(MyLikelihoodAllbutZ, self).__init__(self.endog, self.exog, self.loglike, **kwds)  # self.loglike añadido


    def loglike(self, params):
        # Update camera parameters
        self.robot.camera.r0[0] = params[0]
        self.robot.camera.r0[1] = params[1]
        self.robot.camera.rotation0 = EulerRotation(params[2],params[3],params[4]) 


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
            start_params = [self.robot.camera.r0[0], self.robot.camera.r0[1], self.robot.camera.rotation0.psi,
                            self.robot.camera.rotation0.theta, self.robot.camera.rotation0.phi]
            
        # Call the parent class's fit method

        return super(MyLikelihoodAllbutZ, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)
     

class MyLikelihoodxyz(GenericLikelihoodModel):

   def __init__(self, endog, exog, robot, **kwds):
      """
      Optimizes x , y and z
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
 
      print('N points: ', self.n)

      exog = np.asarray([])

      for i in range(self.n):
         x = J_points[i].J1, J_points[i].J2, J_points[i].Z, J_points[i].Jz, camera_points[i][0], camera_points[i][1]
         exog = np.append(exog, x)

      self.exog = exog.reshape([self.n, 6])

      self.robot = robot
      # self.sigma2 = self.robot.sigmaCamera**2 + self.robot.sigmaRobot**2

      super(MyLikelihoodxyz, self).__init__(self.endog, self.exog, self.loglike, **kwds)  # self.loglike añadido


   def loglike(self, params):

      self.robot.camera.r0 = np.asarray([params[0], params[1], params[2]])

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
         start_params = [self.robot.camera.r0[0], self.robot.camera.r0[1], self.robot.camera.r0[2]]
        
      # Call the parent class's fit method

      return super(MyLikelihoodxyz, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)
   
# X&Y
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
    

class MyLikelihoodAngles(GenericLikelihoodModel):

   def __init__(self, endog, exog, robot, **kwds):
      """
      Optimizes psi, theta and phi.
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
 
      # print('N points: ', self.n)

      exog = np.asarray([])

      for i in range(self.n):
         x = J_points[i].J1, J_points[i].J2, J_points[i].Z, J_points[i].Jz, camera_points[i][0], camera_points[i][1]
         exog = np.append(exog, x)

      self.exog = exog.reshape([self.n, 6])

      self.robot = robot
      # self.sigma2 = self.robot.sigmaCamera**2 + self.robot.sigmaRobot**2

      super(MyLikelihoodAngles, self).__init__(self.endog, self.exog, self.loglike, **kwds)  # self.loglike añadido


   def loglike(self, params):

      self.robot.camera.rotation0 = EulerRotation(params[0],params[1],params[2]) # pitch, roll y yaw

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
         start_params =  [self.robot.camera.rotation0.psi, self.robot.camera.rotation0.theta,
                         self.robot.camera.rotation0.phi]
        
      # Call the parent class's fit method

      return super(MyLikelihoodAngles, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)
   
# theta
class MyLikelihoodxytheta(GenericLikelihoodModel):

   def __init__(self, endog, exog, robot, **kwds):
      """
      Optimizes x , y and z
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
 
      print('N points: ', self.n)

      exog = np.asarray([])

      for i in range(self.n):
         x = J_points[i].J1, J_points[i].J2, J_points[i].Z, J_points[i].Jz, camera_points[i][0], camera_points[i][1]
         exog = np.append(exog, x)

      self.exog = exog.reshape([self.n, 6])

      self.robot = robot
      # self.sigma2 = self.robot.sigmaCamera**2 + self.robot.sigmaRobot**2

      super(MyLikelihoodxytheta, self).__init__(self.endog, self.exog, self.loglike, **kwds)  # self.loglike añadido


   def loglike(self, params):

      self.robot.camera.r0[0] = params[0]
      self.robot.camera.r0[1] = params[1]
      # print('Ang 1', self.robot.camera.rotation0.psi, self.robot.camera.rotation0.theta, self.robot.camera.rotation0.phi)
      self.robot.camera.rotation0.theta = params[2] 
      # print('Ang 2', self.robot.camera.rotation0.psi, self.robot.camera.rotation0.theta, self.robot.camera.rotation0.phi)
      

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
         start_params = [self.robot.camera.r0[0], self.robot.camera.r0[1],
                         self.robot.camera.rotation0.theta]
        
      # Call the parent class's fit method

      return super(MyLikelihoodxytheta, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)
   
# psi
class MyLikelihoodxypsi(GenericLikelihoodModel):

   def __init__(self, endog, exog, robot, **kwds):
      """
      Optimizes x , y and z
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
 
      print('N points: ', self.n)

      exog = np.asarray([])

      for i in range(self.n):
         x = J_points[i].J1, J_points[i].J2, J_points[i].Z, J_points[i].Jz, camera_points[i][0], camera_points[i][1]
         exog = np.append(exog, x)

      self.exog = exog.reshape([self.n, 6])

      self.robot = robot
      # self.sigma2 = self.robot.sigmaCamera**2 + self.robot.sigmaRobot**2

      super(MyLikelihoodxypsi, self).__init__(self.endog, self.exog, self.loglike, **kwds)  # self.loglike añadido


   def loglike(self, params):

      self.robot.camera.r0[0] = params[0]
      self.robot.camera.r0[1] = params[1]
      # print('Ang 1', self.robot.camera.rotation0.psi, self.robot.camera.rotation0.theta, self.robot.camera.rotation0.phi)
      self.robot.camera.rotation0.psi = params[2] 
      # print('param2:', params[2])
      # print('Ang 2', self.robot.camera.rotation0.psi, self.robot.camera.rotation0.theta, self.robot.camera.rotation0.phi)
      

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
         start_params = [self.robot.camera.r0[0], self.robot.camera.r0[1],
                         self.robot.camera.rotation0.psi]
        
      # Call the parent class's fit method

      return super(MyLikelihoodxypsi, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)
   
# phi
class MyLikelihoodxyphi(GenericLikelihoodModel):

   def __init__(self, endog, exog, robot, **kwds):
      """
      Optimizes x , y and z
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
 
      print('N points: ', self.n)

      exog = np.asarray([])

      for i in range(self.n):
         x = J_points[i].J1, J_points[i].J2, J_points[i].Z, J_points[i].Jz, camera_points[i][0], camera_points[i][1]
         exog = np.append(exog, x)

      self.exog = exog.reshape([self.n, 6])

      self.robot = robot
      # self.sigma2 = self.robot.sigmaCamera**2 + self.robot.sigmaRobot**2

      super(MyLikelihoodxyphi, self).__init__(self.endog, self.exog, self.loglike, **kwds)  # self.loglike añadido


   def loglike(self, params):

      self.robot.camera.r0[0] = params[0]
      self.robot.camera.r0[1] = params[1]
      # print('Ang 1', self.robot.camera.rotation0.psi, self.robot.camera.rotation0.theta, self.robot.camera.rotation0.phi)
      self.robot.camera.rotation0.phi = params[2] 

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
         start_params = [self.robot.camera.r0[0], self.robot.camera.r0[1],
                         self.robot.camera.rotation0.phi]
        
      # Call the parent class's fit method

      return super(MyLikelihoodxyphi, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)
   

# psi y theta
class MyLikelihoodxyzphi1(GenericLikelihoodModel):

   def __init__(self, endog, exog, robot, **kwds):
      """
      Optimizes x , y and z
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
 
      print('N points: ', self.n)

      exog = np.asarray([])

      for i in range(self.n):
         x = J_points[i].J1, J_points[i].J2, J_points[i].Z, J_points[i].Jz, camera_points[i][0], camera_points[i][1]
         exog = np.append(exog, x)

      self.exog = exog.reshape([self.n, 6])

      self.robot = robot
      # self.sigma2 = self.robot.sigmaCamera**2 + self.robot.sigmaRobot**2

      super(MyLikelihoodxyzphi1, self).__init__(self.endog, self.exog, self.loglike, **kwds)  # self.loglike añadido


   def loglike(self, params):

      self.robot.camera.r0[0] = params[0]
      self.robot.camera.r0[1] = params[1]
      # self.robot.camera.r0[2] = params[2]
      self.robot.camera.rotation0.psi = params[2] 
      self.robot.camera.rotation0.theta = params[3] 



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
         start_params = [self.robot.camera.r0[0], self.robot.camera.r0[1],
                         self.robot.camera.rotation0.psi, self.robot.camera.rotation0.theta]
        
      # Call the parent class's fit method

      return super(MyLikelihoodxyzphi1, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)
   

class Calibration():

   def __init__(self, real_robot, simul_robot):
      self.robot = real_robot 
      self.robot2 = simul_robot
      # Simulo las medidas reales, con robot1 real
      measurements, points, camera_measurements = make_measurements_camera_pointing(self.robot, False)      
      self.measurements = measurements
      self.camera_measurements = camera_measurements
      self.real_points = points
      n = len(self.real_points)

      # print(camera_measurements)
   
   def get_loglikelihood(self,params):
      print('Loglikelihood...')

      cal = MyLikelihoodxyphi(self.real_points, [self.measurements, self.camera_measurements], self.robot2)
      
      # Ajustar modelo
      print(cal.loglike(params))

   def calibratePos(self, show = False):
      print('Calibrating Position...')

      cal = MyLikelihoodxyz(self.real_points, [self.measurements, self.camera_measurements], self.robot2)
      
      # Ajustar modelo
      results = cal.fit() 

      if show:
         # Mostrar resultados
         print("\n Full results summary:")
         print(results.summary(xname = ['x','y','z']))

      return results.params[0], results.params[1], results.params[2]

   def calibratePos_xy(self, show = False):
      print('Calibrating Position...')

      cal = MyLikelihoodxy(self.real_points, [self.measurements, self.camera_measurements], self.robot2)
      
      # Ajustar modelo
      results = cal.fit() 

      if show:
         # Mostrar resultados
         print("\n Full results summary:")
         print(results.summary(xname = ['x','y']))

      print(results.params[0], results.params[1])
      return results.params[0], results.params[1]
   
   def calibrateAngles(self, show = False): 
      print('Calibrating Angles...')

      cal = MyLikelihoodAngles(self.real_points, [self.measurements, self.camera_measurements], self.robot2)
      
      # Ajustar modelo
      results = cal.fit() 

      if show:
         # Mostrar resultados
         print("\n Full results summary:")
         print(results.summary(xname = ['psi','theta','psi']))

      return results.params[0], results.params[1], results.params[2]
 
   def calibrateAll(self, show = False): 
      print('Calibrating All...')

      cal = MyLikelihoodAll(self.real_points, [self.measurements, self.camera_measurements], self.robot2)
      
      # Ajustar modelo
      results = cal.fit() 

      if show:
         # Mostrar resultados
         print("\n Full results summary:")
         print(results.summary(xname =  ['x','y','z','psi','theta','phi']))

      return results.params[0], results.params[1], results.params[2], results.params[3], results.params[4], results.params[5]

   def calibrateAllbutZ(self, show = False): 
      print('Calibrating All...')

      cal = MyLikelihoodAllbutZ(self.real_points, [self.measurements, self.camera_measurements], self.robot2)
      
      # Ajustar modelo
      results = cal.fit() 

      if show:
         # Mostrar resultados
         print("\n Full results summary:")
         print(results.summary(xname =  ['x','y','psi','theta','phi']))

      return results.params[0], results.params[1], results.params[2], results.params[3], results.params[4]

   def calibrateAltPsi(self, show = False):
      print('Calibrating All...')

      cal = MyLikelihoodxypsi(self.real_points, [self.measurements, self.camera_measurements], self.robot2)
      
      # Ajustar modelo
      results = cal.fit() 

      if show:
         # Mostrar resultados
         print("\n Full results summary:")
         print(results.summary(xname =  ['x','y','psi']))

      return results.params[0], results.params[1], results.params[2]
 
   def calibrateAltTheta(self, show = False):
      print('Calibrating All...')

      cal = MyLikelihoodxytheta(self.real_points, [self.measurements, self.camera_measurements], self.robot2)
      
      # Ajustar modelo
      results = cal.fit() 

      if show:
         # Mostrar resultados
         print("\n Full results summary:")
         print(results.summary(xname =  ['x','y','Theta']))

      return results.params[0], results.params[1], results.params[2]
 
   def calibrateAltPhi(self, show = False):
      print('Calibrating All...')

      cal = MyLikelihoodxyphi(self.real_points, [self.measurements, self.camera_measurements], self.robot2)
      
      # Ajustar modelo
      results = cal.fit() 

      if show:
         # Mostrar resultados
         print("\n Full results summary:")
         print(results.summary(xname =  ['x','y','phi']))

      return results.params[0], results.params[1], results.params[2]
 
   def calibrateAlt1(self, show = False):
      print('Calibrating All...')

      cal = MyLikelihoodxyzphi1(self.real_points, [self.measurements, self.camera_measurements], self.robot2)
      
      # Ajustar modelo
      results = cal.fit() 

      if show:
         # Mostrar resultados
         print("\n Full results summary:")
         print(results.summary(xname =  ['x','y', 'psi', 'theta']))

      return results.params[0], results.params[1], results.params[2]
 

   def FullCal(self, epochs = 10):

      a,b,c = self.robot2.camera.r0[0], self.robot2.camera.r0[1], self.robot2.camera.r0[2]
      d,e,f = self.robot2.camera.rotation0.psi, self.robot2.camera.rotation0.theta, self.robot2.camera.rotation0.phi

      lr_pos = 0.9
      lr_angle = 0.1

      for epoch in range(epochs):
            
         # for t in np.linspace(0.2, 0, 10):

         a,b,c = self.calibratePos()
         a0, b0, c0 = self.robot2.camera.r0[0], self.robot2.camera.r0[0], self.robot2.camera.r0[0]
         da, db, dc = a - a0, b-b0, c-c0
         new_a, new_b, new_c = a0 + da * lr_pos, b0 + db * lr_pos, c0 + dc * lr_pos
         # self.robot2.camera = Camera(new_a, new_b, new_c, d, e , f, cx = -0.5, cy = -0.5, focaldistance = 20, sigmaCamera = 0.001)


         if epoch % 2 == 0:
            d,e,f = self.calibrateAngles()
            d0, e0, f0 = self.robot2.camera.rotation0.psi, self.robot2.camera.rotation0.theta, self.robot2.camera.rotation0.phi
            dd, de, df = d - d0, e-e0, f-f0
            new_d, new_e, new_f = d0 + dd * lr_angle, e0 + de * lr_angle, f0 + df * lr_angle
            # self.robot2.camera = Camera(a, b, c, lr_angle*d, lr_angle*e, lr_angle*f, cx = -0.5, cy = -0.5, focaldistance = 20, sigmaCamera = 0.001)
         else:
            new_d, new_e, new_f = self.robot2.camera.rotation0.psi, self.robot2.camera.rotation0.theta, self.robot2.camera.rotation0.phi
         self.robot2.camera = Camera(new_a, new_b, new_c, new_d, new_e, new_f, cx = -0.5, cy = -0.5, focaldistance = 20, sigmaCamera = 0.001)


         # a,b,c,d,e,f = self.calibrateAll()
         # self.robot2.camera = Camera(a, b, c, d, e, f, cx = -0.5, cy = -0.5, focaldistance = 20, sigmaCamera = 0.001)




         print(f'\n Results FullCall:  EPOCH: {epoch} ////////////////////////////////////')
         print(f'x:  {a}    y:{b}   z:{c}')
         print(f'psi:  {d}    theta:{e}   psi:{f} \n')




def main():
   """
   Two robots are defined, one is the Real robot to simulate the actual measurements, 
   and the other robot is our first estimation for the real robot's parameters.
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
   # camera = Camera(x = 5.0, y = 0, z = 3.6, psi = 0.003, theta =  0.002, phi = -0.04, cx = -0.5, cy = -0.5, focaldistance = 20, sigmaCamera = 0.001)
   # camera = Camera(x = 5.0, y = 0, z = 3.6, psi = 0.08, theta =  0.05, phi = -0.04, cx = -0.5, cy = -0.5, focaldistance = 20, sigmaCamera = 0.001)
   # camera = Camera(x = 5.0, y = 2, z = 3.6, psi = math.pi/180*5, theta = - math.pi/180*4, phi = math.pi/180*3.5, cx = -0.5, cy = -0.5, focaldistance = 20, sigmaCamera = 0.001)
   camera = Camera(5,0,3.6,0,0.2,0, cx = -0.5, cy = -0.5, focaldistance = 20, sigmaCamera = 0.001)
   camera2 = Camera(6,0.5,3.6,0,0,0, cx = -0.5, cy = -0.5, focaldistance = 20, sigmaCamera = 0.001)


   # Generate the robot
   robot = Robot(60.77, 38.0, 24.0, 34.0, table, camera, fig, ax1, ax2, ax3)

   #  My guess for the actual robot
   robot2 = copy.deepcopy(robot)
   # camera2 = Camera(6, 2, 3.6, psi = 0.00, theta = 0.0 , phi = 0.00, cx = -0.5, cy = -0.5, focaldistance = 20, sigmaCamera = 0.001)
   # camera2 = Camera(4.9, 0.57, 2.5, -0.06, -0.07 , 0.006, cx = -0.5, cy = -0.5, focaldistance = 20, sigmaCamera = 0.001)
   
   robot2.camera = camera2
   
   Cal_test = Calibration(robot, robot2)

   # Cal_test.get_loglikelihood([5,0,0.2])

   # Cal_test.calibrateAll(True)
   # Cal_test.calibrateAllbutZ(True)
   # Cal_test.calibrateAngles(True)
   # Cal_test.calibratePos(True)
   # Cal_test.calibratePos_xy(False)

   # Cal_test.calibrateAltPsi(True)
   Cal_test.calibrateAltTheta(True)
   # Cal_test.calibrateAltPhi(True)
   # Cal_test.calibrateAlt1(True)


   # Cal_test.FullCal()





    

# (4.9851, 0.4961, 22.0148, 0.01, 0.01 , 0.01
# (4.9, 0, 22, -1.5380, 0.0180, 0.0267



if __name__ == "__main__":
   main()

   
     