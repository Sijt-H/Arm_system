# Arm_system

## Vision
`image_publisher.py` opens the webcam connection, creates a `\webcam` node and publishes the image on the `image` topic.
The image publisher asks for a camera number to select. On a device with multiple cameras (eg. forward-facing and external USB) each camera has a number. 
cd to `~/Arm_sytem/` and run `python3 camera_selection.py`. This will output the numer of the connected cameras. 
Number 0 is reserved for the default camera, which would be the front-facing camera for most laptops.

## Detection


## Running with ROS
```
cd to ~Arm_system\agrobot_ws\
catkin_make
roscore
rosrun vision image_publisher.py
rosrun detection detection.py
```



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
   
