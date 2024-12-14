from statsmodels.base.model import GenericLikelihoodModel
import numpy as np
import matplotlib.pyplot as plt

from src.Robot import Robot
from src.EulerRotation import EulerRotation

class MyLikelihood(GenericLikelihoodModel):

   def __init__(self, endog, exog, robot, **kwds):

      #  Numero de puntos, endog puedo pasarlo o sacarlo 
      self.n = int(len(exog)/4)
      #rPoints_ = np.copy(endog)
      #self.rPoints = np.asmatrix(rPoints_.reshape(self.n, 3))
      #cPoints = array que contiene [x1, y1, J11, J21, x2, y2, J21, J22, ....]
      cPoints_ = np.copy(exog)
      self.cPoints = np.asmatrix(cPoints_.reshape(self.n, 4))
      self.robot = robot
      #self.sigma2 = self.robot.sigmaCamera**2 + self.robot.sigmaRobot**2
      super(MyLikelihood, self).__init__(endog, exog, **kwds)


   def loglike(self, params):

      self.robot.camera.r0[0] = params[0]
      self.robot.camera.r0[1] = params[1]
      self.robot.camera.r0[2] = params[2]
      self.robot.camera.rotation = EulerRotation(params[3], params[4], params[5])
      self.robot.camera.focaldistance = params[6]
      self.robot.camera.cx = params[7]
      self.robot.camera.cy = params[8]

      chi2 = 0.0
      for i in range(0, self.n):
         rPoint = np.asarray([self.rPoints[i, 0], self.rPoints[i, 1], self.rPoints[i, 2]])
         #Mover Robot y sacar la foto x,y del punto
         valid, cPointGlobal = self.robot.fromCameraToGlobalSystem(rPoint, cPoint)
         chi2 += ((cPoint[0]-cPointGlobal[0])**2 + (cPoint[1]-cPointGlobal[1])**2)
      return -chi2
   
     