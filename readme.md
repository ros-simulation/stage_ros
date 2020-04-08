# Introduction
This Fork is to fix error **Invalid argument passed to lookupTransform argument source_frame in tf2** which cased by **migration from tf to tf2**

This issues was described [here](https://github.com/rst-tu-dortmund/teb_local_planner_tutorials/issues/9)
# Using
- Create catkit workspace:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
- Build only stage_ros and it's dependencies:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/AMRobots/stage_ros.git
$ cd ../
catkin_make --only-pkg-with-deps stage_ros
```
