import numpy as np


class Table:

    def __init__(self):

        self.points = []
        self.pLL = np.asarray([-38.0, -65.0, 0])
        self.pUL = np.asarray([-38.0, 36.0, 0])
        self.pLR = np.asarray([61.0, -65.0, 0])
        self.pUR = np.asarray([61.0, 36.0, 0])
        
    def addReferencePoint(self, x, y, z):
        self.points.append(np.asarray([x,y,z]))

    def toNumpy(self, p):
        return np.asmatrix(p)
    
    def prepareForFit(self, r):

        x = self.toNumpy(r)
        y = x.flatten()
        y = np.asarray(y)
        return y[0,:]
    
    def generatePoints(self):

        #Network 1
        x = [-37.5, -25.0, -12.5, 0.0, 12.5, 25.0, 37.5, 50.0]
        y = [-50.0, -37.5, -25.0, -12.5, 0.0, 12.5, 25.0]
        for ix in x:
            for iy in y:
                if iy > -24.9:
                    continue
                self.addReferencePoint(ix, iy, 0.0)
        for iy in y:
            self.addReferencePoint(50.0, iy, 0.0)
        for ix in x:
            self.addReferencePoint(ix, 25.0, 0.0)
        for ix in x:
            if ix > -25.1:
                continue
            for iy in y:
                self.addReferencePoint(ix, iy, 0.0)
        #Horizontal Line 1
        self.addReferencePoint(0, -60.0, 0.0)
        self.addReferencePoint(50.0, -60.0, 0.0)

        #Vertical Line Right
        y2 = [-60.0, -50.0, -32.0, 24.0]
        for iy in y2:
            self.addReferencePoint(60.0, iy, 0.0)
        #Horizontal Line 2
        self.addReferencePoint(-14.0, -19.0, 0.0)
        self.addReferencePoint(14.0, -19.0, 0.0)
        #Horizontal Line 3
        self.addReferencePoint(-19.0, -14.0, 0.0)
        self.addReferencePoint(19.0, -14.0, 0.0)
        #Horizontal Line 4
        self.addReferencePoint(-19.0, 14.0, 0.0)
        self.addReferencePoint(19.0, 14.0, 0.0)
        #Horizontal Line 4
        self.addReferencePoint(-14.0, 19.0, 0.0)
        self.addReferencePoint(14.0, 19.0, 0.0)

    def plotTable(self, ax1, ax2, t, alpha=0.0):

        x_start = [self.pLL[0], self.pLR[0], self.pUR[0], self.pUL[0], self.pLL[0]]
        y_start = [self.pLL[1], self.pLR[1], self.pUR[1], self.pUL[1], self.pLL[1]]
        z_start = [self.pLL[2], self.pLR[2], self.pUR[2], self.pUL[2], self.pLL[2]]
        ax1.plot3D(x_start , y_start, z_start, 'b')
        ax2.plot(x_start, y_start, 'b')
        ax1.plot3D(0, 0, 0, 'r*')
        ax2.plot(0, 0, 'r*')

        for p in self.points:
            ax1.plot3D(p[0], p[1], p[2], t, alpha)
            ax2.plot(p[0], p[1], t, alpha)

    