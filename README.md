# ITD
Intensity Triangle Descriptor Constructed from High-Resolution Spinning LiDAR Intensity Image for Loop Closure Detection

# 1. Introduction

**The paper is currently under review, and the core code will be available after the paper is accepted. 
At present, we provide a dynamic library (lib/libintensity_td.so) for the core code to ensure that you can test it in advance.**

ITD has strong environmental adaptability, and experiments show that ITD can achieve high recall and accuracy loop detection tasks in indoor, urban, geometrically degraded corridors, and unstructured suburban roads.

# 2. Prerequisites

* Ubuntu18.04 or higher.
* ROS >= Melodic.
* EIGEN >= 3.4.0
* PCL >= 1.8
* OPENCV >= 3.2.0

# 3. Build
Clone the repository and catkin_make:
```shell
mkdir -p ~/$A_ROS_DIR$/src
cd ~/$A_ROS_DIR$/src
git clone https://github.com/RoboFeng/ITD.git
cd ..
catkin_make
source devel/setup.bash
```

# 4. Prepare Sample Datasets
* [STheReO dataset](https://sites.google.com/view/rpmsthereo/download).
* [Fusionportable dataset](https://fusionportable.github.io/dataset/fusionportable/).
* We provide [trajectory files](demo/demo_traj/README.md) and usage instructions for the datasets.

# 5. Run ITD
* **Run STheReO dataset**
  
You need to modify the root directory where the sequence is located in [config/demo_sthereo.yaml](config/demo_sthereo.yaml),and ensure that the trajectory file exists in a specific folder under the root directory.
```shell
roslaunch intensity_td run_sthereo_opt.launch
```

* **Run Fusionportable dataset**

You need to modify the rosbag path and trajectory file path of the sequence in [config/demo_fusionportable.yaml](config/demo_fusionportable.yaml).
```shell
roslaunch intensity_td run_fusionportable_opt.launch
```

* **Run Online Demo**

At present, we only tested the Ouster-OS1-128 LiDAR. You need to modify the rostopic names of the LiDAR point cloud and odometer in [config/demo_online.yaml](config/demo_online.yaml). Please note that the original complete point cloud must be entered.

```shell
roslaunch intensity_td run_online_opt.launch
```