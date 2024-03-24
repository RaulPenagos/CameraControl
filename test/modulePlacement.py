import numpy as np
import sys




def generatePoints(n):

    points = []
    x0 = 10
    y0 = 10
    z0 = 0
    L = 30
    for i in range(0, n):
        x1 = x0 + i * L/2.0
        y1 = y0 + i * L/2.0
        z1 = -20
        x2 = x0 + L + i * L/2.0
        y2 = y0 + i * L/2.0
        z2 = -20
        x3 = x0 + i * L/2.0
        y3 = y0 + L + i * L/2.0
        z3 = -20
        x4 = x0 + L + i * L/2.0
        y4 = y0 + L + i * L/2.0
        z4 = -20
        r1 = np.asarray([x1,y1,z1])
        r2 = np.asarray([x2,y2,z2])
        r3 = np.asarray([x3,y3,z3])
        r4 = np.asarray([x4,y4,z4])
        points.append(r1)
        points.append(r2)
        points.append(r3)
        points.append(r4)


if __name__=='__main__':


   
    realPoints = generatePoints(1)
    realPointsCameraView = []
    for p in realPoints:
        pcamera = fromGlobalToCameraSystem(params, p, p)


    params = [10, 0, 0, 0, 0, 0]

    J1 = np.pi/6.0
    J2 = 0.0
    z = -5
    r = fromRotationToPosition(J1, J2, z)
    x = np.asarray([50 * np.cos(np.pi/4.0), 50 * np.sin(np.pi/4.0), 0])
    valid, v = fromGlobalToArmSystem(r, x)
    if valid:
        print ('Punto en globales:', x)
        print ('Punto en locales:', v)
    valid, v2 = fromArmToGlobalSystem(r, v)
    if valid:
        print ('Punto en locales:', v)
        print ('Punto en globales:', v2)
    



    
    


