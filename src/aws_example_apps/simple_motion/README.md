# simple_motion ROS package

This ROS package contains a ROS node that allows for a simple translation and rotation based on velocities and distance/rotation, and for stop if there is anything immediately in front of it, based on laser scanner.

## Requirements
- ROS -- tested on Melodic, but other versions may work.
- colcon -- used for building the application. 

## Build
Once cloned in a ROS workspace, e.g., `ros_workspace/src/`, run the following commands to build it:

	cd ros_workspace
	colcon build
	
## Run
Run first the robot nodes or simulator. 
Then, source and use the launch file:

	source ros_workspace/install/setup.sh
	roslaunch simple_motion simple_motion.launch

## Attribution & Licensing

Materials substantially authored by Alberto Quattrini Li. Copyright 2020 by Amazon.com, Inc. or its affiliates. Licensed MIT-0 - See LICENSE for further information
