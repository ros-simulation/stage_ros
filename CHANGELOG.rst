^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package stage_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.7.2 (2013-09-19)
------------------
* Changed default GUI window size to 600x400
* Added velocity to ground truth odometry
* Fixed tf (yaw component) for the base_link->camera transform.
* Fixed ground truth pose coordinate system

1.7.1 (2013-08-30)
------------------
* Fixing warnings
* Small fixes
* Added RGB+3D-sensor interface (Primesense/Kinect/Xtion).
  * Publishes CameraInfo, depth image, RGBA image, tf (takes world-file pantilt paremeter into account)
  * Supports the "old" configuration (laser+odom) as well as camera+odom, laser+camera+odom and odom-only.
  Fixed laser transform height (previously was hardcoded at 0.15, now it takes robot height into account).
* Introduced changes from https://github.com/rtv/Stage/issues/34 with some changes (does not require lasers to be present and works without cameras).

1.7.0 (2013-06-27 18:15:07 -0700)
---------------------------------
- Initial move over from old repository: https://code.ros.org/svn/ros-pkg/stacks/stage
- Catkinized
- Stage itself is released as a third party package now
- Had to disable velocities in the output odometry as Stage no longer implements it internally.
- Updated rostest
- Updated rviz configurations
