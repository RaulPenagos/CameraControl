import numpy as np
from src.EulerRotation import EulerRotation
from src.cartesianpoint import cartesianpoint

class Camera:

    def __init__(self, x, y, z, psi, theta, phi, cx, cy, focaldistance, sigmaCamera):

        #This is the position of the camera with respect to the system of arm2
        self.r0 = np.asarray([x, y, z])
        self.rotation0 = EulerRotation(psi, theta, phi)
        
        self.sigmaCamera = sigmaCamera
        self.cx = cx
        self.cy = cy
        self.focaldistance = focaldistance
        
        self.cartesianpos = cartesianpoint(np.asarray([0.0, 0.0, 0.0]), np.asarray([1.0, 0.0, 0.0]), np.asarray([0.0, 1.0, 0.0]), np.asarray([0.0, 0.0, 1.0]))


    def setCameraGlobalInformation(self, pos):
        
        self.pos = pos
      




       