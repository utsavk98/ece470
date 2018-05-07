# Semester Robot Simulation Project
ECE 470: Introduction to Robotics

Utsav Kawrani

Molly Sturgis

5 March 2018

The overall goal of this semester project is to simulate a robot doing a task. For our team's project, we are simulating a firefighting robot.  This READ ME file covers the installation of a robot simulation software (V-REP) and executing the simulation.

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

Download our team's source code into this folder. The scene is called "Final_Scene_water.ttt" and the source code is "project.ipynb".
'
In this folder, there are three other files which must also be included. 
*v-rep.py - located: vrep/programming/remoteApiBindings/python/python/vrep.py
*vrepConst.py - located: vrep/programming/remoteApiBindings/python/python/vrepConst.py
*remoteApi.dylib -located: vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib, this depends on your operating system, follow the file system for your operating system
```


## Running the Simulation


Step 1. Launch V-REP
```
This can be done by terminal via launching the program directly.
```

Step 2. Load Our Scene
```
Go to the file menu, then "Open scene..." then navigate to Final_Scene_Water.ttt

Step 3. Open "Project.ipynb" in JupyterLab 
```
One can do this by opening JupyterLab and navigating to the location where "Project.ipynb" was saved. Press the run button which is located on the toolbar directly below the tab with the name of the file. 

```
Step 4. Watch the Magic

## Authors

* **Utsav Kawrani** 
* **Molly Sturgis**


## Acknowledgments

* Thank you to moms, specifically ours
* Dads are cool too
