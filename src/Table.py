import numpy as np


class Table:

    def __init__(self):

        self.points = []

    def addReferencePoint(self, x, y, z):
        self.points.append(np.asarray([x,y,z]))
    
    def toNumpy(self, p):
        return np.asmatrix(p)
    
    def prepareForFit(self, r):

        x = self.toNumpy(r)
        y = x.flatten()
        y = np.asarray(y)
        return y[0,:]
    
    def generatePointsInSquare(self, x0, y0, z0, L):

        #Network 1
        x = [-37.5, -25.0, -12.5, 0.0, 12.5, 25.0, 37.5, 50.0]
        y = [-50.0, -37.5, -25.0, -12.5, 0.0, 12.5, 25.0]
        for ix in x:
            for iy in y:
                if iy > -25.1:
                    continue
                self.addReferencePoint(ix, iy, 0.0)
        for iy in y:
            self.addReferencePoint(50.0, iy)
        for ix in x:
            self.addReferencePoint(ix, 25.0)
        for ix in x:
            if ix > -25.1:
                continue
            for iy in y:
                self.addReferencePoint(ix, iy)
        #Horizontal Line 1
        self.addReferencePoint(0, -60.0)
        #Vertical Line Right
        y2 = [-60.0, -50.0, -32.0, 24.0]
        for iy in y2:
            self.addReferencePoint(60.0, iy)
        #Horizontal Line 2
        self.addReferencePoint(-14.0, -19.0, 0.0)
        self.addReferencePoint(14.0, -19.0, 0.0)
        #Horizontal Line 3
        self.addReferencePoint(-19.0, -14.0, 0.0)
        self.addReferencePoint(14.0, -14.0, 0.0)



        self.addReferencePoint(-37.5, -50.0, 0.0)
        self.addReferencePoint(-37.5, -37.5, 0.0)
        self.addReferencePoint(-37.5, 0.0, 0.0)
        self.addReferencePoint(-37.5, 0.0, 0.0)
        self.addReferencePoint(-37.5, 0.0, 0.0)
        self.addReferencePoint(-37.5, 0.0, 0.0)
        self.addReferencePoint(-37.5, 0.0, 0.0)
        self.addReferencePoint(-37.5, 0.0, 0.0)
        self.addReferencePoint(-37.5, 0.0, 0.0)

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

        