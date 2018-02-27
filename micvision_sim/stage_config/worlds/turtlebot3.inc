include "rplidar.inc"

define turtlebot3 position
(
  color "red"          # Default color.
  drive "diff"         # Differential steering model.
  gui_nose 1           # Draw a nose on the robot so we can see which way it points
  obstacle_return 1    # Can hit things.
  laser_return 1                # reflects sonar beams


  # alternative odometric localization with simple error model
  localization "odom"                 # Change to "gps" to have impossibly perfect, global odometry
#  odom_error [ 0.05 0.05 0.1 0 ]      # Odometry error or slip in X, Y and Theta (Uniform random distribution)   
  odom_error [ 0 0 0 0 ]

  # four DOF kinematics limits
  # [ xmin xmax ymin ymax zmin zmax amin amax ]        
  velocity_bounds     [-0.75 0.75 0 0 0 0 -90.0 90.0 ]          
  acceleration_bounds [-0.5 0.5 0 0 0 0 -90.0 90.0 ]

  # Actual size
  size [0.36 0.36 0.35]

  # The pioneer's center of rotation is offset from its center of area
  origin [0 0 0 0]

  # Estimated mass in KG
  mass 40.0 

  # Body shape:
  # radius = 0.18m
  block
  ( 
    points 25
    point[0]  [-1.0000 -0.0000]
    point[1]  [-0.9659 -0.2588]
    point[2]  [-0.8660 -0.5000]
    point[3]  [-0.7071 -0.7071]
    point[4]  [-0.5000 -0.8660]
    point[5]  [-0.2588 -0.9659]
    point[6]  [ 0.0000 -1.0000]
    point[7]  [ 0.2588 -0.9659]
    point[8]  [ 0.5000 -0.8660]
    point[9]  [ 0.7071 -0.7071]
    point[10] [ 0.8660 -0.5000]
    point[11] [ 0.9659 -0.2588]
    point[12] [ 0.5000  0.0000]
    point[13] [ 0.9659  0.2588]
    point[14] [ 0.8660  0.5000]
    point[15] [ 0.7071  0.7071]
    point[16] [ 0.5000  0.8660]
    point[17] [ 0.2588  0.9659]
    point[18] [ 0.0000  1.0000]
    point[19] [-0.2588  0.9659]
    point[20] [-0.5000  0.8660]
    point[21] [-0.7071  0.7071]
    point[22] [-0.8660  0.5000]
    point[23] [-0.9659  0.2588]
    point[24] [-1.0000  0.0000]
    z [0 1]
  )

  rplidar( pose [0.0 0 -0.2 0 ])
)

