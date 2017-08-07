# Pepper Obstacle avoidance

## Prerequisites
* ViSP
* [QuadProgpp](https://github.com/liuq/QuadProgpp)

## Installation
* Clone and build QuadProgpp in $QUADPROG_HOME
* Set environment variable $QUADPROG_HOME, pointing to the install folder of QuadProgg  
* Clone the repo in your `catkin/src`   
 `$ cd ~/catking_ws/src`   
 `$ git clone https://github.com/lagadic/pepper_obs_avoid.git`   
* Build the package:   
 `$ cd ~/catkin_ws`   
 `$ catkin_make -Dvisp_naoqi_DIR=/path/to/visp_naoqi/build -Dvisp_DIR=/path_to_build_visp `   
 



