import numpy as np
from src.EulerRotation import EulerRotation

class Camera:

    def __init__(self, x, y, z, psi, theta, phi, sigmaCamera):

        self.r0 = np.asarray([x, y, z])
        self.rotation0 = EulerRotation(psi, theta, phi)
        self.sigmaCamera = sigmaCamera
        self.r = self.r0
        self.rotation = self.rotation0

    def rotate(self, Jz):

        rotation = EulerRotation(Jz, 0.0, 0.0)
        self.r = rotation.apply(self.r0)
        self.rotation = UPDATERotation



       