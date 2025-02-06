from statsmodels.base.model import GenericLikelihoodModel
import numpy as np
import sys
import matplotlib.pyplot as plt

sys.path.append('/CameraControl/test')
from test.makeMeasurements import *
from src.Robot import Robot
from src.EulerRotation import EulerRotation


def find_bad_points(robot, R):
    for n, point in enumerate(robot.table.actualPoints):
        if np.linalg.norm(point) >= R:
            print(n, point, np.linalg.norm(point))

def test_capture_point(robot):
    # Voy a simular que muevo el robot a un punto, para ver que ocurre con la camara
    # A donde apunta?

    P = [20,10,0]

    robot.cartesianMoveTo(P, 0)

    print(P)

    print(robot.fromInnerToCartesian(robot.J1, robot.J2, P[2]))

    x,y = robot.point3DToCameraProjection(P)

    print(x,y)

    print(robot.cameraProjectionToPoint3D([x,y]))
    



def test_z(robot):
    points = [[10,10,0],[-30,-15,0],[20,-8,0]]

    for point in points:
        
        robot.cartesianMoveTo(point, 0)

        # print('r0:', robot.camera.r0global)
        # print(robot.camera.uxglobal, robot.camera.uyglobal, robot.camera.uzglobal)
        print(point)
        print('camera points to:', robot.cameraPointing())
        print(':)',robot.camera.r0global)
        print(':)',robot.camera.r0global)
        print('robot', robot.fromInnerToCartesian(robot.J1, robot.J2, robot.Z))
    # Sí, uz varia en función de los angulos

def test_camera_aim(robot):
    P = [20, 10, 0]

    robot.cameraAim(P)
    print(robot.cameraPointing())
    robot.cartesianMoveTo(P, 0)
    print(robot.cameraPointing())

def main():
    fig = plt.figure(figsize = (16, 8), layout="constrained")
    gs0 = fig.add_gridspec(1, 2, width_ratios=[2, 1])
    ax1 = fig.add_subplot(gs0[0], projection = '3d')
    gs1 = gs0[1].subgridspec(2,1)
    ax2 = fig.add_subplot(gs1[0])
    ax3 = fig.add_subplot(gs1[1])
    ax1.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax1.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax1.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax1.set_xlabel('x [cm]')
    ax1.set_ylabel('y [cm]')
    ax1.set_zlabel('z [cm]')
    ax2.set_xlabel('x [cm]')
    ax2.set_ylabel('y [cm]')
    ax3.set_xlabel('z [cm]')
    ax3.set_ylabel('y [cm]')
    ax1.axes.set_xlim3d(left=-70, right=70.0) 
    ax1.axes.set_ylim3d(bottom=-70, top=70.0)   
    ax2.axes.set_xlim((-40.0, 70.0))
    ax2.axes.set_ylim((-70.0, 40.0))
    ax3.axes.set_xlim((-1.0, 1.0))
    ax3.axes.set_ylim((-1.0, 1.0))


    table = Table(0.01, 0.0)


    camera = Camera(x = 2, y = 3, z = -27.0, psi = .05, theta = 0.08, phi = 0.07, cx = 0.5, cy = 0.5, focaldistance = 10, sigmaCamera = 0.001)

    # print(camera.r0)
    robot = Robot(50.0, 30.0, 30.0, 40, table, camera, fig, ax1, ax2, ax3)
    # print(robot.camera.r0)
    # robot.cartesianMoveTo([5,5,0],0)
    # print(robot.camera.r0)

    # find_bad_points(robot, 58)

    # test_z(robot)

    test_camera_aim(robot)
    # print('patata')

    



if __name__ == "__main__":
    main()