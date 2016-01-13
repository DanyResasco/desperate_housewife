# desperate_housewife
This repo contains  the code to run the demo desperate housewife developed in CP.
To use this repo execute the following steps:

	- 'git clone https://github.com/CentroEPiaggio/calibration.git'
	- 'git clone --recursive https://github.com/CentroEPiaggio/vito-robot.git
	- 'cd vito-robot'
	- move to trash the pkg ros_control
	- 'cd ..'
	- 'cd kuka_hw '
	- 'git checkout manueld_devel '

Compile doing 'catkin_make --only_pkg-with-deps desperate_housewife' in your workspace
The command to launch the robot in simulation is : roslaunch desperate_housewife desperate_robot.launch


TEST FILE
To test field: 
	-roslaunch test_send_obst.launch
		** For this code sends a ros message  rostopic pub  -1 /send_obst std_msgs/Float64MultiArray " data: [Pos_ost_1_x, Pos_ost_1_y, Ps_ost_1_z, radius_1, height_1, Pos_ost_2_x, Pos_ost_2_y, Pos_ost_2_z, radius_2, height_2] "

	-roslaunch desperate_housewife grid.launch
		** This code draws the repulsive fields like an arrow around he obstacle. For this code sends a ros message rostopic pub -1 /gridspace std_msgs/Float64MultiArray 'data: [x_min, x_max, x_resolution, y_min, y_max, y_resolution, z_min, z_max, z_resolution]'

	-roslaunch desperate_housewife ball.launch
		**This code simulate the movements of a ball that moves within the field. For this code sends a ros message   rostopic pub -1 /SphereInfo std_msgs/Float64MultiArray 'data: [Pos_x_init, Pos_y_init, Pos_z_init, radius, mass, Pos_x_des, Pos_y_des, Pos_z_des]'



