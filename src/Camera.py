import numpy as np

from src.EulerRotation import EulerRotation

class Camera:

    def __init__(self, x, y, z, psi, theta, phi):

        self.r = np.asarray([x, y, z])
        self.rotation = EulerRotation(psi, theta, phi)
       