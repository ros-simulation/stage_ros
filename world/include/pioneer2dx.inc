# A pioneer robot optionally equipped with a laser range sensor and/or a QR-code reader

define pioneer_base position 
(
  color "red"                   # Default color.
  drive "diff"                  # Differential steering model.
  gui_nose 1                    # Draw a nose on the robot so we can see which way it points
  obstacle_return 1             # Can hit things.
  ranger_return 0.5             # reflects sonar beams
  blob_return 1                 # Seen by blobfinders  
  fiducial_return 1             # Seen as "1" fiducial finders

  localization "gps"             
  localization_origin [0 0 0 0]   # Start odometry at (0, 0, 0).

  # alternative odometric localization with simple error model
  # localization "odom"              # Change to "gps" to have impossibly perfect, global odometry
  # odom_error [ 0.05 0.05 0.1 ]    # Odometry error or slip in X, Y and Theta
                                     # (Uniform random distribution)   

  # four DOF kinematics limits
  # [ xmin xmax ymin ymax zmin zmax amin amax ]        
  velocity_bounds [-1 1 0 0 0 0 -45.0 45.0 ]          
  acceleration_bounds [-0.5 0.5 0 0 0 0 -45 45.0 ]
)

define pioneer2dx_base_no_sonar pioneer_base
(
  # actual size
  size [0.44 0.38 0.22] # sizes from MobileRobots' web site

  # the pioneer's center of rotation is offset from its center of area
  origin [-0.04 0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 0

  # estimated mass in KG
  mass 23.0 
)

# as above, but with front sonar only
define pioneer2dx_no_sonar pioneer2dx_base_no_sonar
(  
  # simplified Body shape:
  block( 
    points 8
    point[0] [-0.2 0.12]
    point[1] [-0.2 -0.12]
    point[2] [-0.12 -0.2555]
    point[3] [0.12 -0.2555]
    point[4] [0.2 -0.12]
    point[5] [0.2 0.12]
    point[6] [0.12 0.2555]
    point[7] [-0.12 0.2555]
    z [0 0.22]
  )
)

define qrcodereader fiducial
(
  fiducial_key 1 # fiducial_key values of model and fiducial detector must match

  # fiducial properties
  range_min 0.0
  range_max 8.0
  range_max_id 5.0
  fov 270.0
  ignore_zloc 1  #  When set to 1, the fiducial finder ignores the z component when checking a fiducial

  model
  (
    # model properties
    size [ 0.1 0.1 0.1 ]
    color "green"
  )
)
