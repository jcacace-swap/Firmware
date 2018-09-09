# PX4 Gazebo simulator

This repository contains all necessary files to run the PX4 firmware on ROS-Gazebo simulation. To start the simulation clone the repository including all the submodules:
		
        $ git clone https://jocacace@bitbucket.org/sar_drone/px4.git --recursive
        
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

		export sitl_location='/PATH/TO/px4/Firmware/Tools/sitl_gazebo'
		cd $sitl_location
		source load_sitl_conf.sh
		cd    

### Install Gazebo for MAVLink SITL

You can refer to the official [PX4 Simulation](https://dev.px4.io/en/simulation/) documentation to properly install and run the Gazebo Mavlink simulator.
 


