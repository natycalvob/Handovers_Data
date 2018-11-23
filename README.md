# Handovers_Data
This repository contains the data set collected for human-robot object handovers. 

## Prerequisites

- Python<br />
In Ubuntu python 3.4 can be installed via terminal with the command given below: 
```
sudo apt-get install python3
```
- Matlab<br />
- ROS<br /> 
In order to succesfully run the code, you should have installed <a href="http://wiki.ros.org/kinetic/Installation/Ubuntu">ROS</a>

## Installation 
Do not forget to install the necessary libraries described in the Requirements paragraph.
Clone the repository in the source folder of your catkin workspace.

```
git clone https://github.com/natycalvob/Handovers_Data.git
```

## Description of Dataset

### Data Collection

The dataset collection took place on the [EmaroLab](https://github.com/EmaroLab) at the University of Genoa
We used a ROS-Node to collect data and save it into ROS-bag files. 
We collected x,y and z positions of 15 body joints: *left foot, left knee, left hand, left elbow, left shoulder, right foot, right knee, right hip, 
right hand, right elbow, right shoulder, torso, neck, head* <br />
<br />
Each experiment has its own bag file, using a Python script the data was converted intoCSV fileles, 
where the rows contain the timestamp and the columns contain 15 Kinect 3D coordinates data. *(e.g.  head_x, head_y, head_z)*   

### Data Processing 

Data preparation, processing and analysis were done using MATLAB. Dataset has x,y and z coodinates of the upper body joints. 

## Authors

* [Natalia Calvo](https://github.com/natycalvob), e-mail: nata.calvob@hotmail.com
