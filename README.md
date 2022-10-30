# galaxy_camera

ROS wrapper for the galaxy camera made by Daheng Imaging.

Forked from QiayuanLiao/git@github.com:QiayuanLiao/galaxy_camera.git

Dependencies:

- ROS Melodic
- gxiapi

ONLY TESTED ON MER2-302!!!

# Getting started

## Install dependencies

- ROS - http://wiki.ros.org/ROS
- gxiapi: - download `Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.1911.9122` from
  http://gb.daheng-imaging.com/CN/Software and install

## Download and build code

1. git clone
2. Make in your workspace

```
catkin_make
source devel/setup.bash
```

## Test

1. Connect the camera by Gige, run:

```
roslaunch galaxy_camera MER-132GC.launch
or 
roslaunch galaxy_camera MER-132GC_R_L.launch
```

2. Adjust the params in launch file.
3. Calibrate:

```
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.030 image:=/galaxy_camera/image_raw camera:=/galaxy_camera
```

4. More information:
   http://wiki.ros.org/image_pipeline

# TODO

- Multi-camera support
- nodelet support
- test on other device

# 数据信息

相机型号：MER-200-20GC
IP地址：192.168.29.1 和 192.168.29.2
