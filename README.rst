ecto_corrector
========================================

ecto_corrector contains Ecto (http://ecto.willowgarage.com/) bindings for the
pose_corrector ROS package (http://www.ros.org/wiki/pose_corrector). The main
purpose of ecto_corrector is to provide pose refinement functionality when
matching a 3D mesh model into range data. 

Installation:
----------------------------------------

ecto_corrector currently requires the following ROS packages to be installed
and in your ROS_PACKAGE_PATH:
 - pose_corrector
 - sensor_msgs
 - geometry_msgs
 - tabletop_object_detector

You must also have ecto (https://github.com/plasmodic/ecto) and ecto_pcl
(https://github.com/plasmodic/ecto_pcl). The recommended method for getting 
these dependencies is through ecto_kitchen (https://github.com/plasmodic/ecto_kitchen).

| Example instructions for building ecto_corrector:
|  cd ecto_corrector
|  mkdir build
|  cd build
|  cmake .. -Decto_DIR=~/ecto_kitchen/build -Decto_pcl_INCLUDES=~/ecto_kitchen/pcl/include
|  make -j4

Running example programs in the scripts directory:
----------------------------------------

As with other ecto Python scripts, make sure you have your PYTHONPATH set up 
properly. Use the python_path.sh script in ecto_corrector/build as well as 
the python_path.sh scripts for other required ecto packages prior
to running the script.

