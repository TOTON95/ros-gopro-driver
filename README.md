# ros-gopro-driver

This is an interface between ROS and a GOPRO camera (GOPRO 4 and above), this driver transports images from the camera using WiFi towards the ROS environment, publishing it as a ROS image topic, leaving it usable for any further process. 

## Quickstart (Linux)

These steps describe the process of compiling the ROS driver, connecting the GoPro, and viewing the video stream over a ROS topic.
Ubuntu 18.04 (Bionic Beaver) was used to write this, but the steps are expected to be fairly general. 
These steps are confirmed for GOPRO Hero versions 4 and 5. 

**Install ROS**

If you do not already have ROS installed, follow the [ROS officials insructions](http://wiki.ros.org/ROS/Installation). 

Install the `rospkg` python module.

Python (python 2.7) is almost certainly installed by default, but the installation steps for ROS might not include the `rospkg` module. 

    pip install rospkg

**Setup catkin workspace for compiling ROS packages.**

If you continued the sequence of ROS tutorials past installation, you already setup a catkin workspace.
Otherwise, follow [ROS officials insructions](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

The typical path to your catkin workspace path is `~/catkin_ws`. 
Since you can setup catkin anywhere, the following steps will use `$CATKIN_PATH` to refer to your catkin workspace path. 

**Download and install `ros-gopro-driver`.**

    cd $CATKIN_PATH/src/
    git clone https://github.com/TOTON95/ros-gopro-driver.git
    cd ..
    catkin_make

**Connect computer's WiFi to GoPro.**

On the GOPRO, enable WiFi connections in settings. There will be a menu item to find the GOPRO's WiFi network ID and password. 
On the computer, connect to that WiFi network. 

**Launch the `ros-gopro-driver` node.**

    roslaunch ros-gopro-driver gopro_5.launch

If the camera is connected over WiFi, the driver should connect after a few seconds, and the console output show that the frames are being recieved. 
   
_Example output:_

    [ INFO] [1613796372.141513528]: Frame: 10039 Bytes decoded 7023 check 1
   
    [ INFO] [1613796372.182556570]: Frame: 10040 Bytes decoded 6028 check 1
   
    [ INFO] [1613796372.228475522]: Frame: 10041 Bytes decoded 8986 check 1
   
**View the stream using `rqt_image_view`.**

      rqt_image_view /gopro_5/gopro_out

If there are no issues, you will see the GOPRO stream!

## Node's ROS topics 

- gopro_out: topic that contains the streaming GOPRO frames
- gp_shutter: topic that activates the GOPRO shutter/start video command
- gp_stop: topic that stops the GOPRO recording


**Note:** Currently, this interface is only capable of work with GOPRO 4 and 5. More cameras with their respective configuration files are going to be added in the future. 

**Known issues:** There is some delay between the camera and the image being published, this driver is built with the best way found to reduce it and sacrificing the absence of some image errors, so far it can achieve a performance similar to the stock application from GOPRO itself, hopefully this is not the lowest delay possible. So it is recommendable to use it as a video logging tool, unless your application does not depend on real-time video feed or your strategy is robust enough to tackle the delay drawbacks.


