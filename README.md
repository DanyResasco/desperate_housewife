# desperate_housewife
This repo contains  the code to run the demo desperate housewife developed in CP.
To use this repo execute the following steps:

	- 'git clone https://github.com/CentroEPiaggio/calibration.git'
	- 'git clone --recursive https://github.com/CentroEPiaggio/vito-robot.git
	- 'cd vito-robot'
	- move to trash the pkg ros_control
	- move to trash the pkg kuka-lwr
	- 'git clone https://github.com/manuelbonilla/kuka-lwr.git'
	- cd kuka-lwr
	- 'git checkout ros-i-specs'
	- 'cd ../..'
	- 'git clone https://github.com/manuelbonilla/desperate_housewife.git'
	- 'cd desperate_housewife'
	- 'git checkout tvito_torque_control_devel '
	- 'cd ..'

Compile doing 'catkin_make --only_pkg-with-deps desperate_housewife' in your workspace
The command to launch the robot in simulation is : roslaunch desperate_housewife run_vito.launch
The demo doesn't start until does not receive the message : rostopic pub -1 /right_arm/PotentialFieldControl/start_control std_msgs/Bool 'True'
If you want use this demo for one arm the command to launch is: roslaunch desperate_housewife run_vito.launch use_both_arm:=false 
and select the arm to use with left_arm_enabled:=true or right_arm_enabled:=true


DESPERATE MIND DEMO

With this code it's possible to make three different demo changing one param, called demo, into config file desperate_demo_hand_pose_param.
If set this param to_
	- 0 starts the demo that take the first graspable object with obstacles avoidance
	- 1 start the demo tat take the first graspable object and the obstacle will be push down to the table 
The third demo is not properly a demo because works always at background and starts when on the table there are more than user definition number of obstalces. In this case the robot overtun the table precisely Desperate_housewife. This number is setting into config file desperate_demo_hand_pose_param. This param is called Number_obj. There is another param to set, is the time of interpolation. This time is setting into config file vito_controllers. It's must be setting in both arm. At config param, this time is called time_interp_desired.
Lets a look of all code:  
	- Desperate mind is logic of a system.
	- Generate hand pose decide wich arm to use, if cylinder is a graspable object or it's a obstacle and decide the relative poses.
	- Fit geometries make a cluster of scene point cloud and fittig a cylinder
	- Scene filtering make a filter of camera point cloud, erasing the table and the environment


TEST FILE

To test potential field: 

	-roslaunch test_send_obst.launch  (sends only upright cylinder)
	
	This code sends a ros message  to 

	-To sends only upright cylinder: rostopic pub  -1 /send_obst std_msgs/Float64MultiArray " data: [Pos_ost_1_x, Pos_ost_1_y, Ps_ost_1_z, radius_1, height_1, Pos_ost_2_x, Pos_ost_2_y, Pos_ost_2_z, radius_2, height_2] "

	-To any tipe of cylinder sends: rostopic pub  -1 /PotentialFieldControl/obstacle_list desperate_housewife/fittedGeometriesArray 

	-roslaunch desperate_housewife grid.launch
		** This code draws the repulsive fields like an arrow around the obstacle. For this code sends a ros message:
		 rostopic pub -1 /gridspace std_msgs/Float64MultiArray 'data: [x_min, x_max, x_resolution, y_min, y_max, y_resolution, z_min, z_max, z_resolution]'

	-roslaunch desperate_housewife ball.launch
		**This code simulate the movements of a ball that moves within the field. For this code sends a ros message:
		  rostopic pub -1 /SphereInfo std_msgs/Float64MultiArray 'data: [Pos_x_init, Pos_y_init, Pos_z_init, radius, mass, Pos_x_des, Pos_y_des, Pos_z_des]'
