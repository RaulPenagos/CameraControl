from src.Robot import *
import matplotlib.pyplot as plt

def main():
    """
    Two robots are defined, one is the Real robot to simulate the actual measurements, 
    and the other robot is our first estimation for the real robot's parameters.
    """
    
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

    # Generate the camera  
    camera = Camera(x = 5.0, y = 1, z = 3.6, psi = 0.00, theta =  0.0, phi = 0.00, cx = -0.5, cy = -0.5, focaldistance = 10, sigmaCamera = 0.001)

    # Generate the robot
    robot = Robot(60.77, 38.0, 24.0, 34.0, table, camera, fig, ax1, ax2, ax3)

#     print('\n Robot Data:\n')
#     print(f'x: {robot.camera.r0[0]:.4f}\t y: {robot.camera.r0[1]:.4f}\t z: {robot.camera.r0[2]:.4f}\n ')
#     print(f'psi: {robot.camera.rotation0.psi:.4f}\t theta: {robot.camera.rotation0.theta:.4f}\t phi: {robot.camera.rotation0.phi:.4f}\n')
    
        
#     print('Ang 1', robot.camera.rotation0.psi, robot.camera.rotation0.theta, robot.camera.rotation0.phi)
#     robot.camera.rotation0.theta = 0.00101220703125
#     robot.camera.r0[0], robot.camera.r0[1] = 5.4 , 1.4
#     robot.cartesianMoveTo([25,49,0], 0)
#     print('Ang 2', robot.camera.rotation0.psi, robot.camera.rotation0.theta, robot.camera.rotation0.phi)
#     robot.camera.rotation0.psi =  0.0010244140625
#     robot.cartesianMoveTo([20,36,0], 0)
#     print('Ang 2', robot.camera.rotation0.psi, robot.camera.rotation0.theta, robot.camera.rotation0.phi)
#     robot.camera.rotation0.theta = 0.1
#     robot.cartesianMoveTo([20,40,0], 0)
#     print('Ang 2', robot.camera.rotation0.psi, robot.camera.rotation0.theta, robot.camera.rotation0.phi)
#     robot.camera.rotation0.theta, robot.camera.rotation0.psi = 0, 0
#     print('Ang 2', robot.camera.rotation0.psi, robot.camera.rotation0.theta, robot.camera.rotation0.phi)
      
      
      



# if __name__ == "__main__":
#    main()

   

