import numpy as np
from src.EulerRotation import EulerRotation
from src.cartesianpoint import cartesianpoint

from src.RealCamera import IDS_Camera



class Camera(IDS_Camera):

    def __init__(self, x, y, z, psi, theta, phi, cx, cy, focaldistance, sigmaCamera):

        super().__init__()

        #This is the position of the camera with respect to the system of arm2
        self.r0 = np.asarray([x, y, z], dtype=float)
        self.rotation0 = EulerRotation(psi, theta, phi)
        
        self.sigmaCamera = sigmaCamera
        self.cx = cx
        self.cy = cy
        self.focaldistance = focaldistance  # 20 cm
        self.focusdistance = 3.6  # cm
        # r0 global, uxglobal, uyglobal, uz global ?
        self.cartesianpos = cartesianpoint(np.asarray([0.0, 0.0, 0.0]), np.asarray([1.0, 0.0, 0.0]), np.asarray([0.0, 1.0, 0.0]), np.asarray([0.0, 0.0, 1.0]))

    def setCameraGlobalInformation(self, pos):
        #  No se usa
        self.pos = pos # does not update self.cartesianpos?
      