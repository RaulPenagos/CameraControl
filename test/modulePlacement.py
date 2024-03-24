import numpy as np


R1 = 20
R2 = 30


def angleFromSineCosine(s, c):

    if s >= 0:
        return np.arccos(c)
    else:
        return -np.arccos(c)

def fromRotationToPosition(J1, J2, Z):

    x = R1 * np.cos(J1) + R2 * np.cos(J2)
    y = R1 * np.sin(J1) + R2 * np.sin(J2)
    z = Z
    return np.asarray([x, y, z])
    
def checkValid(v, j):

    x = R1 * np.cos(j[0]) + R2 * np.cos(j[1])
    y = R1 * np.sin(j[0]) + R2 * np.sin(j[1])
    if (x-v[0])**2 + (y-v[1])**2 < 1e-5:
        return True

def fromPositionToRotation(v):

    x = v[0]
    y = v[1]
    z = v[2]
    Delta = (x**2 + y**2 + R1**2 - R2**2)/(2.0*R1)
    a = (x**2 + y**2)
    b = -2.0 * Delta * x
    c = Delta**2 - y**2
    if b**2-4.0*a*c < 0.0:
        print('The vector is out of range')
        return False, 0, 0, 0
    
    cosj1_p = (-b + np.sqrt(b**2-4.0*a*c))/(2.0*a)
    cosj2_p = (x - R1 * cosj1_p) / R2
    sinj1_pp = np.sqrt(1.0 - cosj1_p**2)
    sinj2_pp = (y - R1 * sinj1_pp) / R2
    sinj1_pm = -np.sqrt(1.0 - cosj1_p**2)
    sinj2_pm = (y - R1 * sinj1_pm) / R2
       

    cosj1_m = (-b - np.sqrt(b**2-4.0*a*c))/(2.0*a)
    cosj2_m = (x - R1 * cosj1_m) / R2
    sinj1_mp = np.sqrt(1.0 - cosj1_m**2)
    sinj2_mp = (y - R1 * sinj1_mp) / R2
    sinj1_mm = -np.sqrt(1.0 - cosj1_m**2)
    sinj2_mm = (y - R1 * sinj1_mm) / R2

    J1pp = angleFromSineCosine(sinj1_pp, cosj1_p)
    J1pm = angleFromSineCosine(sinj1_pm, cosj1_p)
    J1mp = angleFromSineCosine(sinj1_mp, cosj1_m)
    J1mm = angleFromSineCosine(sinj1_mm, cosj1_m)
    
    J2pp = angleFromSineCosine(sinj2_pp, cosj2_p)
    J2pm = angleFromSineCosine(sinj2_pm, cosj2_p)
    J2mp = angleFromSineCosine(sinj2_mp, cosj2_m)
    J2mm = angleFromSineCosine(sinj2_mm, cosj2_m)

    pairs = [[J1pp, J2pp], [J1pm, J2pm], [J1mp, J2mp], [J1mm, J2mm]]

    index = -1
    j1 = 1000.0
    for i, j in enumerate(pairs):
        if checkValid(v, j):
            if j[0] >= 0 and j[0] < j1:
                index = i
                j1 = j[0]
    if index == -1:
        return False, 0, 0, 0
    else:
        return True, pairs[index][0], pairs[index][1], z


def fromGlobalToArmSystem(r, x):

    valid, J1, J2, z = fromPositionToRotation(r)
    if not valid:
        return False, np.asarray([0, 0, 0])
    else:
        delta = np.asarray([R1*np.cos(J1), R1*np.sin(J1), z])
        v = x - delta
        rot_ = [[np.cos(-J1), -np.sin(-J1), 0.0],
                [np.sin(-J1), np.cos(-J1), 0.0],
                [0.0, 0.0, 1.0]]
        rot = np.asmatrix(rot_)
        newv = np.asarray(rot.dot(v))[0]
        return True, newv

def fromArmToGlobalSystem(r, x):

    valid, J1, J2, z = fromPositionToRotation(r)
    if not valid:
        return False, np.asarray([0, 0, 0])
    else:
        delta = np.asarray([R1*np.cos(J1), R1*np.sin(J1), z])
        rot_ = [[np.cos(J1), -np.sin(J1), 0.0],
                [np.sin(J1), np.cos(J1), 0.0],
                [0.0, 0.0, 1.0]]
        rot = np.asmatrix(rot_)
        newv = np.asarray(rot.dot(x))[0]
        newx = newv + delta
        return True, newx


def fromArmToCameraSystem(param, x):

    delta = np.asarray([param[0], param[1], param[2]])
    v = x - delta
    phix = param[3]
    phiy = param[4]
    phiz = param[5]
    #RotX
    matxnom_ = [[1.0, 0, 0],
                [0.0, np.cos(-phix), -np.sin(-phix)],
                [0.0, np.sin(-phix), np.cos(-phix)]]
    matxnom = np.asmatrix(matxnom_)
    #RotY
    matynom_ = [[np.cos(-phiy), 0.0, -np.sin(-phiy)],
                [0.0, 1.0, 0.0],
                [np.sin(-phiy), 0.0, np.cos(-phiy)]]
    matynom = np.asmatrix(matynom_)
    #RotZ
    matznom_ = [[np.cos(-phiz), -np.sin(-phiz), 0.0],
                [np.sin(-phiz), np.cos(-phiz), 0.0],
                [0.0, 0.0, 1.0]]
    matznom = np.asmatrix(matznom_)                                                                                                                                                                         15,1           7%
    rotnom = matxnom.dot(matynom.dot(matznom))
    newv = np.asarray(rotnom.dot(v))[0]
    return True, newv


def fromCameraToArmSystem(param, x):

    delta = np.asarray([param[0], param[1], param[2]])
    phix = param[3]
    phiy = param[4]
    phiz = param[5]
    #RotX
    matxnom_ = [[1.0, 0, 0],
                [0.0, np.cos(phix), -np.sin(phix)],
                [0.0, np.sin(phix), np.cos(phix)]]
    matxnom = np.asmatrix(matxnom_)
    #RotY
    matynom_ = [[np.cos(phiy), 0.0, -np.sin(phiy)],
                [0.0, 1.0, 0.0],
                [np.sin(phiy), 0.0, np.cos(phiy)]]
    matynom = np.asmatrix(matynom_)
    #RotZ
    matznom_ = [[np.cos(phiz), -np.sin(phiz), 0.0],
                [np.sin(phiz), np.cos(phiz), 0.0],
                [0.0, 0.0, 1.0]]
    matznom = np.asmatrix(matznom_)                                                                                                                                                                         15,1           7%
    rotnom = matxnom.dot(matynom.dot(matznom))
    v = np.asarray(rotnom.dot(x))[0]
    newv = v + delta
    return True, newv


if __name__=='__main__':

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
    



    
    


