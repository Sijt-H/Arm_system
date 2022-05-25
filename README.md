# Arm_system
## Packages
For every sub-system a ROS package has been created. These packages are interconnected by publishing and subscribing to the relevant topics.

### Vision
`image_publisher.py` opens the webcam connection, creates a `\webcam` node and publishes the image on the `image` topic.
The image publisher asks for a camera number to select. On a device with multiple cameras (eg. forward-facing and external USB), each camera has a number. 

cd to `~/Arm_sytem/support_scripts/` and run `python3 camera_selection.py`. This will output the number of the connected cameras. 

Number 0 is reserved for the default camera, which would be the front-facing camera for most laptops. After a restart the last used camera is set to be default.

### Detection
Subscribes to `image` topic. The image received is a ROS type image and needs to be converted to an OpenCV type image to be usable.
The detection package also contains `mapping.py`, this script takes the image coordinates that are detected and maps them to the world coordinates. 
Some setup needs to be done in order to get accurate results.
#### Mapping setup
Calibrate the camera
Measure world points and corresponsing image points
put the image points, world points in the ... script.
Correct the load directory `loaddir` in `mapping.py`
#### Mapping
The mapping calculates the optimal scaling factor "s" for the image and world points that are measured. Using the intrinsic camera parameters: camera matrix and distortion matrix, the image coordinates can be mapped to world coordinates. 

## Running with ROS
Complete the following commands to run the system using ROS.

```
cd to ~Arm_system\agrobot_ws\
catkin_make *only after changes have been made to any packages*
roscore
rosrun vision image_publisher.py
rosrun detection detection.py
rosrun detection mapping.py
```



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
   
