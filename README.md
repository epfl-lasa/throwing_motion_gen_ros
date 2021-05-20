# throwing_motion_gen_ros
This package contains ds based motion generator for throwing task. The algorithm generate motion such that the robot end-effector will pass through the release position with the desired release velocity

# Installation
Go in the `src` directory of your catkin workspace and clone this package:
```sh
git clone  https://github.com/epfl-lasa/throwing_motion_gen_ros.git
```
# Dependencies
The main dependencies are the following ones:

 - **ROS**: Robot operating system (indigo distribution)
 - **CMake**: Build system
 - **Eigen**: A library for linear algebra

# File hierarchy

The file system is divided in several subfolders:
 - `config`: contains _.yaml_ used by launch files and .txt files containing GMM parameters
 - `include`: contains class header files
 - `launch`: contains _.launch_ files
 - `src`: contains class implentations and source files


# running the motion generator

```sh
roslaunch roslaunch throwing_ds_ros load_throwingDS.launch
```