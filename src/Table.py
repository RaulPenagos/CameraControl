import numpy as np


class Table:

    def __init__(self):

        self.points = []

    def addReferencePoint(self, x, y, z):
        self.points.append(np.asarray([x,y,z]))
    
    def generatePointsInSquare(self, x0, y0, z0, L):

        x1 = x0 
        y1 = y0 
        z1 = z0
        x2 = x0 + L 
        y2 = y0 
        z2 = z0
        x3 = x0 
        y3 = y0 + L 
        z3 = z0
        x4 = x0 + L 
        y4 = y0 + L 
        z4 = z0
        self.addReferencePoint(x1, y1, z1)
        self.addReferencePoint(x2, y2, z2)
        self.addReferencePoint(x3, y3, z3)
        self.addReferencePoint(x4, y4, z4)

        