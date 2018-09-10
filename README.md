# Multi sensorized quadcopter for PX4 Gazebo simulator 

This repository contains all necessary files to run the PX4 firmware on ROS-Gazebo simulation, usisng an IRIS based quadrotor equipped with a camera, a depth camera, a laser scanner and a sonsar sensor. 

To start the simulation clone the repository including all the submodules:
		
        $ git clone https://github.com/jocacace/Firmware --recursive
        
4. Compile and launch it!		
		
        $ cd px4/Firmware && make posix_sitl_default gazebo -j4

    During the installation you should install some additional dependencies (follow the instruction in the compilation shell):
    
        $ sudo apt-get install python-pip
        $ pip install --upgrade pip
        $ sudo apt-get install python-jinja2
        $ sudo pip install numpy toml
        $ sudo apt-get install protobuf-c*


5. To launch the Iris version equipped with the camera, you should load the gazebo configuration. You could use the __load_sitl_conf.sh__ script placed in the sitl_gazebo folder

		$ cd Tools/sitl_gazebo && source load_sitl_conf.sh
        $ roslaunch px4 iris_with_camera_sitl.launch 
        
    

### Setup the environment
In ordert to speed up the configuration process could be convienent to add the following link to your .bashrc file:

		export sitl_location='/PATH/TO/PX4/Firmware/Tools/sitl_gazebo'
		
You can now configure the environment in a fast way:
	
		cd $sitl_location
		source load_sitl_conf.sh
		cd    

### Install Gazebo for MAVLink SITL

You can refer to the official [PX4 Simulation](https://dev.px4.io/en/simulation/) documentation to properly install and run the Gazebo Mavlink simulator.

### Use a camera enabled quadrotor with Mavros
We can use Mavros package to [MAVROS](http://wiki.ros.org/mavros) ROS package to control the camera enabled IRIS quadrotor.

##### Launch the simualtion scene
After properly configured the environment, launch the simulation use this command:

		roslaunch px4 iris_camera.launch

You can check the active topics and services to test the correct working of MAVROS bridge. 

##### Arm the quadrotor

To arm the quadrotor we can use the MAVROS service:

		rosservice  call  /mavros/cmd/arming "value: true"
 
##### Take-off
We will use the same Geo-location specified in the load_sitl_conf.sh file to take off:

		rosservice call  /mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 44.4928130, longitude: 11.3299817, altitude: 2.0}" 

In this simulation the Radio controller is not considered. For this reason, if you want to fly in the scene, you should change the default bahviour for the RC loss failsafe. You can use qgroundcontrol ground station to set the default value of NAV_RCL_ACT parameter to: Disabled.

##### Visualize ROS camera stream

To visualize the camera stream, you can use the _rqt_image_view_ tool from ROS selecting the correct camera topic: _/iris_with_camera/camera_raw/image_raw_ 

### Main differences with the Main PX4 repo:
The main differences with the original Firmware repository lie in the configuration files:
		export sitl_location='/PATH/TO/PX4/Firmware/Tools/sitl_gazebo'

		Firmware/posix-configs
		Firmware/Tools/sitl-gazebo/models
		Firmware/Tools/sitl-gazebo/launch



