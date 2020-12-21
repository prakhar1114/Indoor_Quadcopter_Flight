# Launch_ConfigFiles
This folder contains the modified launch and configuration files which will help in experimental indoor quadcopter flight. Only a few parameters have been modified, to know those changes compare with the original files.
# pub_waypoints
ROS package that contains the a python node to set waypoints to the ros topic /mavros/setpoint_position/local initialized by the ros package mavros. The package sends these waypoints to hardware or the simulation connected to the mavros node and makes the vehicle (eg. quadcopter) to follow these waypoints. 
