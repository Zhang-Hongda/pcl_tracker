README
==============================

****
	
|Author|ZhangHongda|
|---|---
|E-mail|2210010444@qq.com


****
# pcl_tracker
## ZhangHongda, Harbin Institute of Technology

An ROS package for tracking 3D-printed maker. The pose information of the marker is published through tf message.
### Installation
Step 1: clone the repository into your own workspace
```
cd ${PATH_TO YOUR_WORKSPACE_FOLDER}/src
git clone https://github.com/Zhang-Hongda/pcl_tracker
```
Step 2: building
```
catkin_make
```
Step 3: activate the workspace
```
source ${PATH_TO YOUR_WORKSPACE_FOLDER}/devel/setup.bash
```
### Preparations
__Build your own marker with a 3D-printer.__
___NOTE:___
    The models of the marker can be found in the "models" folder.    
    The color of the spheres on the marker should be the same, and it should be different from the color of the working platform.    
    Users can build there own marker on 3D modeling softwares such as Solidworks, Pro/E, UG, CATIA, etc.    
    The diameter of the colored spheres should be greater than 30 millimeters (preferably between 30 mm and 40 mm).
    The length of the two sticks that connect the spheres should be different. The difference between the shorter rod and the longer one shoud be about 15 milliters, but both of them shoud be longer than 10mm.    
    The length of the rod under the center sphere can be customized by modifying the "tool_length" parameters in "para/config.yaml".
### Usage 
To launch the program, run
```
roslaunch pcl_tracker demo.launch
```

