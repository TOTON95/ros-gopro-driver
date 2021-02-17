# ros-gopro-driver

This is an interface between ROS and a GOPRO camera (GOPRO 5 and above), this driver transports images from the camera using WiFi towards the ROS environment, publishing it as a ROS image topic, leaving it usable for any further process. 

## Instructions

- Connect to WIFI capable GOPRO with a wireless adapter and select the configuration for your camera.
- gp_shutter: topic that activates the GOPRO shutter/start video command
- gp_stop: topic that stops the GOPRO recording


**Note:** Currently, this interface is only capable of work with GOPRO 5. More cameras with their respective configuration files are going to be added in the future. 

**Known issues:** There is some delay between the camera and the image being published, this driver is built with the best way found to reduce it and sacrificing the absence of some image errors, so far it can achieve a performance similar to the stock application from GOPRO itself, hopefully this is not the lowest delay possible. So it is recommendable to use it as a video logging tool, unless your application does not depend on real-time video feed or your strategy is robust enough to tackle the delay drawbacks.


