pcl_tracker
==============================

****
|Author|Zhang Hongda|
|------|-----------------|
|E-mail|2210010444@qq.com|
|Institute|Harbin Institute of Technology|
****

### Description
An ROS package for tracking a 3D-printed maker. The pose information of the marker is published through tf message. A Kinectv2 sensor is required.
### Dependencies
- ROS kinetic
- PCL1.7
### Preparations
* __Build your own marker with a 3D-printer.__  
    1. The models of the marker can be found in the "[models](./models)" folder. Users can also build there own marker on 3D modeling softwares such as Solidworks, Pro/E, UG, CATIA, etc.   
    2. The `color` of the spheres on the marker should be the same, and it should be different from the color of the working platform (green is recommended).  
    3. The `diameter` of the colored spheres should be greater than 30 millimeters (preferably between 30 mm and 40 mm).
    4. The `length` of the two sticks that connect the spheres should be different. The difference between the shorter rod and the longer one shoud be about 15 milliters, but both of them shoud be longer than 10mm.  

* __Install drivers for Kinectv2__  
    * Install [libfreenect2](https://github.com/OpenKinect/libfreenect2) and [iai_kinect2](https://github.com/code-iai/iai_kinect2).
    
 __NOTE:__  
 * The length of the rod below the center sphere can be customized by modifying the "tool_length" parameter in "[param](./param)/config.yaml".  

![finalresult.PNG](./models/finalresult_.PNG "marker")

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

### Usage 
To launch the program, run:
```
roslaunch pcl_tracker marker_tracker.launch
```
__NOTE:__  

* By pressing the "Shift" on the keyboard and clicking the marker's point cloud in the cloud viewer, users can specify the color of the customized marker (default color for tracking is green).
* The program displays colored point clouds of the working platform by default. To visualize the result of point clouds segmentation and clustering, please modifiy the corresponding parameters in "[param](./param)/config.yaml".
* By default, the program subscribes to "kinect2_qhd_points" published by kinect2_bridge package. The name of the topic can be modified in the launch file or in the command line.
```
roslaunch pcl_tracker demo.launch topic:=/kinect2/sd/points
```

### Result
To get the tracking result, uses can create a tf subscriber to get the position of the marker in the camera frame.
```
rosrun tf tf_echo /kinect2_link /marker
```
The position of the end of the marker can also be acquired.
```
rosrun tf tf_echo /kinect2_link /tool0
```
The output of the program should look like this:
![markertracker.gif](./gif/markertracker.gif "program output")
### Video
[![marker tracker viedo](https://img.youtube.com/vi/WdobYXdcNEA/0.jpg)](https://www.youtube.com/watch?v=WdobYXdcNEA)
