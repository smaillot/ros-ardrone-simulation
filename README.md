# Introduction to ROS

[240AR060] - Syllabus - Introduction to Ros

[![N|Solid](https://mar.masters.upc.edu/capcalera.jpg)](https://mar.masters.upc.edu/es)

The content of this master lecture is the following :

* ROS introduction

> What is ROS. Why, when and where to use ROS. Introduction to basic concepts of software processes, system
and design of robotic architectures. ROS community. Pros and cons of using ROS in a robot. Overview of current
famous robots using ROS.

* ROS environment configuration in a Linux O.S. 

> Installation of ROS in a Linux environment (Ubuntu).

* Introduction to ROS tools. Visualisation, analysis
and debug. Practical application (using ?.bag?
files).

> Introduction to ROS developer tools. Command line tools and GUIs

* Introduction to ROS nodes and communications

> Description of ROS communications scheme. ROS package (node) creation

* Configuration and use of a simulation environment.

> Presentation of the simulated environment to be used during the subject. Introduction to Gazebo. Overview of a
simulated robot.

# ROS Installation

1. Setup sources.list
```sh
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
2. Setup Keys
```sh
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```
3. Debian packages update
```sh
$ sudo apt-get update 
```
4. ROS-Desktop full and other packages installation
```sh
$ sudo apt-get install ros-kinetic-desktop-full ros-kinetic-ardrone-autonomy ros-kinetic-hector-*
```

### Used services


| Service | link |
| ------ | ------ |
| Ubuntu 16.04 | [Ubuntu Xenial Xerus] |
| ROS Kinetic | [ROS kinetic Wiki] |
| BitBucket | [Our BitBucket repo] |

   [240AR060]: <http://www.upc.edu/estudispdf/guia_docent.php?codi=240AR060&lang=ing>
   [Ubuntu Xenial Xerus]: <http://releases.ubuntu.com/16.04/>
   [ROS kinetic wiki]: <http://wiki.ros.org/kinetic>
   [Our BitBucket repo]: https://bitbucket.org/alb_mol/intros17_gamma