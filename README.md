#DISCLAIMER
I am not responsible for any damages or injury as a result of using this system. 
The PSU has the potential to be lethal if misused.

Double check your work and proceed with care!




# Arm_system
System: Ubuntu 20.04 (running as dualboot not in a VM), ROS Noetic
## Packages
For every sub-system a ROS package has been created. These packages are interconnected by publishing and subscribing to the relevant topics. 

### Vision
`image_publisher.py` opens the webcam connection, creates a `\webcam` node and publishes the image on the `image` topic.
The image publisher asks for a camera number to select. On a device with multiple cameras (eg. forward-facing and external USB), each camera has a number. 

cd to `~/Arm_sytem/support_scripts/` and run `python3 camera_selection.py`. This will output the numbers of the connected cameras you can choose from. 

Number 0 is reserved for the default camera, which would be the front-facing camera for most laptops. After a restart the last used camera is set to be default.

### Detection
The scripts `detection.py` subscribes to `image` topic. The image received is a ROS type image and needs to be converted to an OpenCV type image to be usable.
Using Hough Circle Transform the crop is detected and the coordinates of the center point of the circle are published to be mapped.

The detection package also contains `mapping.py`, this script takes the image coordinates that are detected and maps them to the real world coordinates. 

Some setup needs to be done to use the mapping script:

#### Mapping setup
Calibrate the camera
Measure world points and corresponsing image points
put the image points, world points in the ... script.
Correct the load directory `loaddir` in `mapping.py`
#### Mapping
The mapping calculates the optimal scaling factor "s" for the image and world points that are measured. Using the intrinsic camera parameters: camera matrix and distortion matrix, the image coordinates can be mapped to world coordinates. 

## Arduino
The Arduino Mega is connected to the stepper driver (A4988) and controls it using two outputs, STEP and DIR.

## Running with ROS
Complete the following commands to run the system using ROS.

```
cd to ~Arm_system\agrobot_ws\
catkin_make *only after changes have been made to any packages*
roscore
rosrun vision image_publisher.py
rosrun detection detection.py
rosrun detection mapping.py
rosrun rosserial_python serial_node.py /dev/ttyACM0
```


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
   
