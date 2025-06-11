# CameraControl

___

## Pablo Martínez Ruiz del Árbol, Raúl Penagos Solórzano
___

CameraControl is a Python-based simulation that models the geometry and physics of a 2-axis SCARA robot, ensabled to an industrial IDS-Camera. It is used in order to communicate with the robot and take pictures, also, it can simulate its motion and may be used in order to calibrate the position of the camera.
___



## Features

- Simulates motion of the robot and camera
- Simulates areal table with calibration points
- Simulates a real robot, and then calibrates its parameters
- Creates Animation.gif of the movement of the robot


## Installation

The main required dependencies are numpy, matplotlib, random and sys. Also logging, statsmodels and copy  All can be installed using pip:

```bash
pip install numpy matplotlib random sys logging statsmodels copy
```

## Usage


### Some definitions                                                 
- The robot pointer has two systems of coordinates:                
 -Cartesian: (x, y, z) and JZ = position of the pointer and JZ     
 -Inner: (J1, J2, Z) and JZ = rotations of the axis                
 -The robot pointer position z is Z0 - Z                           

- The robot has also two imporant systems of reference:            
 -The absolute one with respect to the table (x, y, z)         
 -The robot has also two imporant systems of reference:       
 -And the one associated to the second arm of the robot        
 -with the center at the robot pointer                        


It is important to notice that the program asumes all data introduced is in the CGS system of units:
- Position in centimeter
- Angles in radians 

### Generate robot, table and camera

The simulation requires a `robot` class object with a `table` and a `camera`. Then the robot can be operated to reach points in the table or focus them using the camera. Pictures can be taken using a basic (but precise) pinhole proyection model. Additionaly, a script includes the necesary functions to simulate the calibration of a robot using a Likelihood Function.



The main scripts are:

- `cartesianpoint.py` : `cartesianpoint` class defines cartesian points given (r, ux, uy, uz), being r the vector (x,y,z)
- `innerpoint.py` : `innerpoint` class defines the robot's position in th inner system of coordinates (J1, J2, Z, Jz)
- `EulerRotation.py` : `EulerRotation` class defines rotations (psi, theta, phi) using Euler's Angles. This rotations are applied as a rotation in `z` axis `phi`, a rotation on `x` axis `theta` and another rotation on `z` axis `psi`.
 
- `Robot.py` : `Robot` class defines a Scara robot with its attributes. It includes functions to transform between both coordinate systems, operate the robot and it's camera and make an animation of its movement.
- `Table.py` : `Table` class defines a table and points over it.
- `Camera.py` : `Camera` class defines a camera linked to the robot. `IDS_Camera` is a class to connect and operate the actual IDS Camera.
- `Likelihoodxy.py` : `MyLikelihoodxy` defines a functional likelihood. Given the measurements made with a 'real robot', it optimizes its 'robot' in order to optimize it's camera's $(x,y)_0$ in order to make it equivalent to the real one.
- `Calibration.py` : This script simulates a real scenario, implements the `MyLikelihoodxy` class, and calibrates a estimation camera.
  
- `Image.py` : This class is used by `IDS_Camera` class and enables the user to edit and calibrate images od fiducials using several methods.

### Basic Usage

The code is meant to be run on Linux Shell (if the user is in a windows machine: WSL Linux Shell).
Before runing any script, in every session, it is needed to source the program:

```bash
    $ cd CameraControl
    $ source setup.sh
```

To create a basic script and run the program the user can create a robot object, view example: 

```python
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
# Uncomment in case you are making an animation
# table.plotTable(ax1, ax2, 'g.')

# Generate the camera  
camera = Camera(x = 5 , y = 0, z = 10, psi =0, theta = 0, phi = 0, cx = -0.5, cy = -0.5, focaldistance = 20, sigmaCamera = 0.001)

# Generate the robot
robot = Robot(60.77, 38.0, 24.0, 34.0, table, camera, fig, ax1, ax2, ax3)
```

Afterwards, the robot can be moved using its methods.

In case the user wants to calibrate or make an animation, this can be donde from `Calibration.py` or `moveRobot.py` scripts, that can be executed as follows:

```bash
$ python3 src/Calibration.py
```

```bash
$ python3 test/moveRobot.py
```



## Code Structure
1. `Robot.py` - Defines the `Robot` class, with the following attributes that describe the SCARA robot:

- h: height of the robot arms
- R1: lenght of the first arm 
- R2: lenght of the second arm 
- Z0: initial reference height of the robots pointer
- table: `table` where the robot is located
- camera: `camera` linked to the robot
- fig, ax1, ax2, ax3: `plt` figure and axis to make a plot or animation of the robot

It's main methods are:

Read the classto check its methods, they are self explanatory.

2. `Camera.py` - Defines `Camera` class, cameras liked to a `Robot` with the following parameters. The camera is described with a pinhole model. In order to get a realistic proyection of the points in the camera's CCD, the camera must be like:

      |<--a-->|<-----------b----------->|
      O-------o-------------------------|
  3D Point  Pinhole              Plane of proyection
        (camera position)

a: focaldistance = 20 cm
b: focusdistance = 3.6 cm

Basically, make sure that the camera is 3.6 cm above a point when taking a picture of it.

Attributes:

- r0:  `(xyz)` position of the camera with respect to the robots pointer
- rotation0:  `EulerRotation(psi, theta, phi)` of the camera in its position
- sigmaCamera: not used
- cx: aumento lateral
- cy: aumento lateral
- focaldistance: focal distance of the microscopes lens (f = 200 mm). Distance
- focusdistance: working distance of the objective (f ~= 36 mm). The distance the camera must be over a point in order to 'focus it' if it were to be the real camera.
- cartesianpos: camera's current position



3. `Table.py`- Defines `Table` class.

- tolerance:  not used
- z: level height of the table (z=0)

Cool part is that a matrxi of reference points can be generated.


