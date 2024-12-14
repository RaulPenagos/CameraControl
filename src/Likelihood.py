from statsmodels.base.model import GenericLikelihoodModel
import numpy as np
import matplotlib.pyplot as plt

from src.Robot import Robot
from src.EulerRotation import EulerRotation



class MyLikelihood(GenericLikelihoodModel):

   def __init__(self, endog, exog, robot, **kwds):
      """
      endog (array): posiciones referencia [X1, Y1, Z1, X2, Y2, Z2, ...]
      exog (array): puntos medidos [x1, y1, J11, J21, x2, y2, J21, J22, ....]
      robot: robot cuyos parametros quiero optimizar
      **kwds: parametros iniciales ~reales

      Puedo dar las posiciones de los puntos como sus XYZ reales,
      o como los x1, y1, J11, J21 en los que encuentro el punto en la camara
      """
      # Pablo:*********************************************************************
      # #  Numero de puntos, endog puedo pasarlo o sacarlo 
      # self.n = int(len(exog)/4)
      # #rPoints_ = np.copy(endog)
      # #self.rPoints = np.asmatrix(rPoints_.reshape(self.n, 3))
      # #cPoints = array que contiene [x1, y1, J11, J21, x2, y2, J21, J22, ....]
      # cPoints_ = np.copy(exog)
      # self.cPoints = np.asmatrix(cPoints_.reshape(self.n, 4))
      # self.robot = robot
      # #self.sigma2 = self.robot.sigmaCamera**2 + self.robot.sigmaRobot**2
      # ***************************************************************************
   
      self.n = int(len(endog)/3)
      # [X1, Y1, Z1, X2, Y2, Z2, ...]
      self.endog = endog
      # [x1, y1, J11, J21, x2, y2, J21, J22, ....]
      self.exog = exog
      self.robot = robot
      cPoints_ = np.copy(exog)  # Puntos de medida
      self.cPoints = np.asmatrix(cPoints_.reshape(self.n, 4))
      rPoints_ = np.copy(endog) # puntos Coordenadas reales puntos
      self.rPoints = np.asarray(rPoints_.reshape(self.n, 3))


      super(MyLikelihood, self).__init__(endog, exog, **kwds)




   """
   Continuar revusando loglike() y busca un ejemplo para saber como ejecutar la minimizacion 
   con .fit()
   
   """


   def loglike(self, params):
      """
      Simulo la operación de mover el robot y medir el punto con los parametros estimados  
      """
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
         # Mover Robot y sacar la foto x,y del punto

         # obtengo un cPoint

         valid, cPointGlobal = self.robot.fromCameraToGlobalSystem(rPoint, cPoint)
         chi2 += ((cPoint[0]-cPointGlobal[0])**2 + (cPoint[1]-cPointGlobal[1])**2)
      return -chi2
   
     