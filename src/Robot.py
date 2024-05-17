import numpy as np
import sys

from src.Camera import Camera
from src.EulerRotation import EulerRotation
from src.Table import Table



class Robot:

    def __init__(self, R1, R2, table, camera, sigmaRobot, sigmaCamera):

        self.R1 = R1
        self.R2 = R2
        self.tol = 1e-8
        self.sigmaRobot = sigmaRobot
        self.sigmaCamera = sigmaCamera
        self.camera = camera
        self.table = table
        self.realTable = self.makeRealTable()    

    # This produces a real table according to the tolerances
    def makeRealTable(self):

        table = Table()
        for point in self.table.points:
            dx = np.random.normal(p[0], self.sigmaRobot)
            dy = np.random.normal(p[1], self.sigmaRobot)
            table.addReferencePoint(point[0] + dx, point[1] + dy, point[2])

    #Auxiliary function
    def angleFromSineCosine(self, s, c):
        if s >= 0:
            return np.arccos(c)
        else:
            return -np.arccos(c)

    #Auxiliary function to check whether two points are the same
    def checkValidConversion(self, v, j):

        x = self.R1 * np.cos(j[0]) + self.R2 * np.cos(j[1])
        y = self.R1 * np.sin(j[0]) + self.R2 * np.sin(j[1])
        if (x-v[0])**2 + (y-v[1])**2 < 1e-5:
            return True
  
    def fromRotationToPosition(self, J1, J2, Z):

        x = self.R1 * np.cos(J1) + self.R2 * np.cos(J2+J1)
        y = self.R1 * np.sin(J1) + self.R2 * np.sin(J2+J1)
        z = Z
        return np.asarray([x, y, z])
    
    def fromPositionToRotation(self, v):

        x = v[0]
        y = v[1]
        z = v[2]
        Delta = (x**2 + y**2 + self.R1**2 - self.R2**2)/(2.0*self.R1)
        a = (x**2 + y**2)
        b = -2.0 * Delta * x
        c = Delta**2 - y**2
        if b**2-4.0*a*c < 0.0:
            return False, 0, 0, 0
    
        cosj1_p = (-b + np.sqrt(b**2-4.0*a*c))/(2.0*a)
        cosj2_p = (x - self.R1 * cosj1_p) / self.R2
        sinj1_pp = np.sqrt(1.0 - cosj1_p**2)
        sinj2_pp = (y - self.R1 * sinj1_pp) / self.R2
        sinj1_pm = -np.sqrt(1.0 - cosj1_p**2)
        sinj2_pm = (y - self.R1 * sinj1_pm) / self.R2

        cosj1_m = (-b - np.sqrt(b**2-4.0*a*c))/(2.0*a)
        cosj2_m = (x - self.R1 * cosj1_m) / self.R2
        sinj1_mp = np.sqrt(1.0 - cosj1_m**2)
        sinj2_mp = (y - self.R1 * sinj1_mp) / self.R2
        sinj1_mm = -np.sqrt(1.0 - cosj1_m**2)
        sinj2_mm = (y - self.R1 * sinj1_mm) / self.R2

        J1pp = self.angleFromSineCosine(sinj1_pp, cosj1_p)
        J1pm = self.angleFromSineCosine(sinj1_pm, cosj1_p)
        J1mp = self.angleFromSineCosine(sinj1_mp, cosj1_m)
        J1mm = self.angleFromSineCosine(sinj1_mm, cosj1_m)
        J2pp = self.angleFromSineCosine(sinj2_pp, cosj2_p)
        J2pm = self.angleFromSineCosine(sinj2_pm, cosj2_p)
        J2mp = self.angleFromSineCosine(sinj2_mp, cosj2_m)
        J2mm = self.angleFromSineCosine(sinj2_mm, cosj2_m)

        pairs = [[J1pp, J2pp], [J1pm, J2pm], [J1mp, J2mp], [J1mm, J2mm]]

        index = -1
        j1 = 1000.0
        for i, j in enumerate(pairs):
            if self.checkValidConversion(v, j):
                if j[0] >= 0 and j[0] < j1:
                    index = i
                    j1 = j[0]
        if index == -1:
            return False, 0, 0, 0
        else:
            return True, pairs[index][0], pairs[index][1]-pairs[index][0], z


    def fromGlobalToArmSystem(self, r, x):

        valid, J1, J2, z = self.fromPositionToRotation(r)
        J = (J1 + J2)
        if not valid:
            return False, np.asarray([0, 0, 0])
        else:
            delta = np.asarray([self.R1*np.cos(J1), self.R1*np.sin(J1), 0.0])
            v = x - delta
            rot_ = [[np.cos(-J), -np.sin(-J), 0.0],
                    [np.sin(-J), np.cos(-J), 0.0],
                    [0.0, 0.0, 1.0]]
            rot = np.asmatrix(rot_)
            newv = np.asarray(rot.dot(v))[0]
            return True, newv

    def fromArmToGlobalSystem(self, r, x):

        valid, J1, J2, z = self.fromPositionToRotation(r)
        J = (J1 + J2)
        if not valid:
            return False, np.asarray([0, 0, 0])
        else:
            delta = np.asarray([self.R1*np.cos(J1), self.R1*np.sin(J1), 0.0])
            rot_ = [[np.cos(J), -np.sin(J), 0.0],
                    [np.sin(J), np.cos(J), 0.0],
                    [0.0, 0.0, 1.0]]
            rot = np.asmatrix(rot_)
            newv = np.asarray(rot.dot(x))[0]
            newx = newv + delta
            return True, newx

    def fromArmToCameraSystem(self, x):

        v = x - self.camera.r
        rotnom = self.camera.rotation.rot
        newv = np.asarray(rotnom.dot(v))[0]
        return True, newv

    def fromCameraToArmSystem(self, x):

        delta = self.camera.r 
        rotnom = self.camera.rotation.rot
        v = np.asarray(rotnom.dot(x))[0]
        newv = v + delta
        return True, newv

    def fromGlobalToCameraSystem(self, x, r):

        valid, plocalarm = self.fromGlobalToArmSystem(x, r)
        if not valid:
            return False, np.asarray([0,0,0])
        valid, pcamera = self.fromArmToCameraSystem(plocalarm)
        if not valid:
            return False, np.asarray([0,0,0])
        return True, pcamera

    def fromCameraToGlobalSystem(self, x, r):
        
        valid, plocalarm = self.fromCameraToArmSystem(r)
        if not valid:
            return False, np.asarray([0,0,0])
        valid, pglobal = self.fromArmToGlobalSystem(x, plocalarm)
        if not valid:
            return False, np.asarray([0,0,0])
        return True, pglobal


    def takeCameraMeasurements(self, refPoints, robotPoints):
        measurements = []
        for i, p in enumerate(refPoints):
            valid, ppoint = self.fromGlobalToCameraSystem(robotPoints[i], p)
            if not valid:
                print("Reference point out of range")
                sys.exit()
            x = np.random.normal(ppoint[0], self.sigmaCamera)
            y = np.random.normal(ppoint[1], self.sigmaCamera)
            z = ppoint[2]
            measurements.append(np.asarray([x, y, z]))
        return measurements