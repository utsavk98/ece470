# Semester Robot Simulation Project
ECE 470: Introduction to Robotics

Utsav Kawrani

Molly Sturgis

5 March 2018

The overall goal of this semester project is to simulate a robot doing a task. For our team's project, we are simulating the filling of a pill organizer with medications. This specific READ ME file covers the installation of a robot simulation software (V-REP) and executing a basic scene where the robot articulates each joint.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

There are a few prerequisite knowledge and programs that will be required in order to accomplish the required steps. Our code which interacts with V-REP is python. Note, these are directions and prerequisites for a windows operating system. 

```
Python 3
Anaconda Prompt
Jupyter Lab
```

### Installing & Downloading

This is a ste-by-step guide to installing and running our team's example scene. 

Step 1. Install V-REP

```
You can find the download at (http://www.coppeliarobotics.com/downloads.html). Then chose the appropriate operating system.
```

Step 2. Follow Prompted Instructions for Installation and Set-Up of V-REP

Step 3. Organize and Set-Up Folders and Files
```
This will enable more seemless working later. First, create a folder for this project, labelling it whatever fills your heart with joy. 

Download our team's source code into this folder. It is called "RobotArmTry1.ttt", for forward kinematics download "project.ipynb" or "ForwardKinematics.ipynb". The first file ("project.ipynb" was our code as of the checkpoint, "ForwardKinematics.ipynb" is a corrected version of the code for forward kinematics.

Download our team's scene into this folder. It is called "UR3.py".
'
In this folder, there are three other files which must also be included. 
*v-rep.py - located: vrep/programming/remoteApiBindings/python/python/vrep.py
*vrepConst.py - located: vrep/programming/remoteApiBindings/python/python/vrepConst.py
*remoteApi.dylib -located: vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib, this depends on your operating system, follow the file system for your operating system
```


## Running the Simulation

These directions are for running the initial demo simulation. Directions for running the forward kinematics simulation are below.
Step 1. Launch V-REP
```
This can be done by terminal via launching the program directly.
```

Step 2. Load Our Scene
```
Go to the file menu, then "Open scene..." then navigate to RobotArmTry1.ttt. 
```
Step 3. Run the Python Code
```
Open an Anaconda prompt in the folder containing your scene and .py executable. 
In your prompt type down 'python UR3.py' and the press enter.

```
Step 4. Watch the Magic

```
Switch to your simulator (V-REP) and watch the robot move.
```
If you desire to view the simulation for forward kinematics, see the below directions:

Step 1. Launch V-REP
```
This can be done by terminal via launching the program directly.
```

Step 2. Load Our Scene
```
Go to the file menu, then "Open scene..." then navigate to RobotArmTry1.ttt. 

Step 3. Open "Project.ipynb" in JupyterLab - this represents the code as we submitted for the checkpoint time. A correct code is found in "ForwardKin.ipynb"
```
One can do this by opening JupyterLab and navigating to the location where "Project.ipynb" or "ForwardKinematics.ipynb" was saved. Press the play button which is located on the toolbar directly below the tab with the name of the file. 

```
Step 4. Watch the Magic

## Authors

* **Utsav Kawrani** 
* **Molly Sturgis**


## Acknowledgments

* Thank you to moms, specifically ours
