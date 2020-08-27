# Drone_Control repository
This repository contains source codes for the following dissertation: "Automatic Drone Control to a Moving Target Point"     
credited by **Lee, Ho Jae and Yeo, Seung Won**    
   
Here we can think of the waypoints as delivery destinations and the moving target as a delivery truck.

### GPS.py
> Move the drone to the moving target by only GPS-based control

### mission_GPS.py
> Move the drone to several waypoints and finish at the stationary target

### mission_GPS_vision.py 
> Move the drone to several waypoints and come back to the moving target with GPS-based and Vision-based Control

### GPS_topic
> This directory contains source codes for a ROS node at the target. This sends GPS information of the target

### targetDetection_topic
> This directory contains source codes for a ROS node at the Drone's RaspberryPi Camera. This sends coordinate information to which the drone should move.
