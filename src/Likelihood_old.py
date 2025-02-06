from statsmodels.base.model import GenericLikelihoodModel
import numpy as np
import sys
import matplotlib.pyplot as plt
import copy

sys.path.append('/CameraControl/test')
from test.makeMeasurements import *
from src.Robot import Robot
from src.EulerRotation import EulerRotation

from statsmodels.stats.outliers_influence import variance_inflation_factor  # check colinearity



class MyLikelihood(GenericLikelihoodModel):

   def __init__(self, endog, exog, robot, **kwds):
      """
      exog (array):  posiciones referencia [X1, Y1, Z1, X2, Y2, Z2, ...]
      endog (array):
      robot: robot cuyos parametros quiero optimizar
      **kwds: parametros iniciales (~ reales) a partir de los cuales minimizo
      """

      # self.n = int(len(exog)/3)
      self.n = int(len(exog))
      print(self.n)

      self.exog = np.asarray(exog)#.reshape((self.n, 3))
      self.endog = np.asarray(endog)#.reshape((self.n, 4))

      self.rPoints = np.copy(self.exog)
      self.cPoints = np.copy(self.endog)

      self.robot = robot
      # self.sigma2 = self.robot.sigmaCamera**2 + self.robot.sigmaRobot**2


      super(MyLikelihood, self).__init__(endog, exog, self.loglike, **kwds)  # self.loglike añadido


   def loglike(self, params):

      self.robot.camera.r0 = np.asarray([params[0], params[1], params[2]])
      self.robot.camera.rotation0 = EulerRotation(params[3], params[4], params[5]) # pitch, roll y yaw
      # self.robot.camera.sigmaCamera = params[6]
      # self.robot.camera.cx = params[7]
      # self.robot.camera.cy = params[8]

      chi2 = 0.0

      real_points = self.rPoints
      measured_points = np.asarray([])
      
      for point in real_points:
         # print('point', point)
         self.robot.cartesianMoveTo(point, 0)
         
         x, y = self.robot.point3DToCameraProjection(point) # obtener con robot bueno
         # print(x, y)
         measured_points = np.append(measured_points, self.robot.cameraProjectionToPoint3D([x, y])) 
         # print('proyection', self.robot2.cameraProjectionToPoint3D([x, y]))

      # cameraProjectionToPoint3D  me da un punto x,y,z
      measured_points = measured_points.reshape([self.n, 3])  

      #  si veo que alguna cosa me da fallos, meter try/excepts:

      # for n, point in enumerate(real_points):
      #    print(point, measured_points[n])


      for i in range(0, self.n):
         rPoint = real_points[i]
         cPoint = measured_points[i]
         print(rPoint, cPoint)
         
         chi2 += ((rPoint[0]-cPoint[0])**2 + (rPoint[1]-cPoint[1])**2+ (rPoint[2]-cPoint[2])**2)
      print('CHI2: ', chi2)
      return -chi2



   def fit(self, start_params=None, method='bfgs', maxiter=20000, **kwargs):
      # method = bfgs, nm, newton, powell

      if start_params is None:
         start_params = [2.1, 0.1, -26.9, 0.015, 0.008, 0.03]

        
      # Call the parent class's fit method
      return super(MyLikelihood, self).fit(start_params=start_params, method=method, maxiter=maxiter, **kwargs)
   

class Calibration():

   def __init__(self, real_robot, simul_robot):
      self.robot = real_robot 
      self.robot2 = simul_robot
      # Simulo las medidas reales, con robot1
      measures, real_points = make_measurements(self.robot)      
      self.measures = measures
      self.real_points = real_points


   def calibrate(self):

      for i in range(len(self.measures)):
         print('a',self.real_points[i])
         print(self.robot.fromInnerToCartesian(self.measures[i][0], self.measures[i][1], 40))
         print(self.robot.fromInnerToCartesian(self.measures[i][0], self.measures[i][1], 0) - self.real_points[i])

      # # Crear modelo
      # # print(f'exog: {self.real_points.shape}')
      # # print(f'endog: {self.measures.shape}')
      # cal = MyLikelihood(self.measures, self.real_points, self.robot2)
      
      # # Ajustar modelo

      # results = cal.fit()    

      # # Mostrar resultados
      # print("\n Full results summary:")
      # print(results.summary())

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
   ax3.axes.set_xlim((-1.0, 1.0))
   ax2.axes.set_xlim((-40.0, 70.0))
   ax2.axes.set_ylim((-70.0, 40.0))
   ax3.axes.set_ylim((-1.0, 1.0))


   table = Table(0.01, 0.0)

   camera = Camera(x = 2.0, y = 0, z = -27.0, psi = 0.02, theta = 0.06, phi = 0.0, cx = 0.5, cy = 0.5, focaldistance = 10, sigmaCamera = 0.001)

   robot = Robot(50.0, 30.0, 30.0, 40, table, camera, fig, ax1, ax2, ax3)
   #  [2.1, 0.1, -26.9, 0.015, 0.008, 0.03]

   robot2 = copy.deepcopy(robot)
   camera2 = Camera(2.1, 0.1, -26.9, 0.015, 0.008, 0.03, cx = 0.5, cy = 0.5, focaldistance = 10, sigmaCamera = 0.001)
   robot2.camera = camera2
   

   Cal_test = Calibration(robot, robot2)

   Cal_test.calibrate()

    






if __name__ == "__main__":
   main()

   
     