# Sphero_robot_stage_ros_simple_simulation

A package that simulates SpheroSPRK robot using stage_ros simulation. More information on stage_ros available [here](http://wiki.ros.org/stage_ros)

How to use:

1. Build the package into an existing catkin workspace. More information on creating a catkin workspace available [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

2. Start ROS in your first terminal.

3. Type _roslaunch sphero_formation simple_setup_sim.launch_ in a new terminal.

If you want to set a specific linear velocity and orientation for the robot follow these steps:

4. a) Type _rosrun sphero_formation simple_controller.py_ in a new terminal.

5. a) To set a certain linear velocity and orientation send geometry_msgs/Twist messages to a topic named /sphero_referenca.
Only linear.x and angular.z fields of the message matter. Set the desired linear velocity by changing linear.x value of the message.
Note that you can set the velocity to be anything between 0 and 0.5. If you set the velocity to a number outside this range, the robot won't
change the current velocity. Type the desired orientation into the angular.z field. The value must be between -pi and pi. If you set the orientation
outside of this range, the robot will keep its current orientation. Note that although this field usually represents angular velocity,
in this case it represents orientation, and NOT angular velocity of the robot.

If you want to set a specific point in x-y space for the robot to go to follow these steps:

4. b) Type _rosrun sphero_formation xy_controller.py_ in a new terminal.

5. b) To set a point for a robot to go to, send geometry_msgs/Point messages to a topic named /sphero_referentna_tocka.
Only x and y fields in the message matter, z field won't be read in the program.
