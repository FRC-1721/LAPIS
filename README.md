# LAPIS
![Image Owned by Mojang](https://gamepedia.cursecdn.com/minecraft_gamepedia/archive/9/9f/20190403173427%21Lapis_Lazuli.png?version=333534eee995063b4191a3abe9f86113)

# Hungry Toaster
![Image Owned by Travis aka boinary aka dirdraggo](https://cdn.discordapp.com/attachments/516760208366764038/647066532861575168/Hungry_Toaster.png)

## Installation Notes
Using the rosinstall file, copy the rosinstall file into your catkin workspace
```
wget https://raw.githubusercontent.com/FRC-1721/LAPIS/master/kinetic.rosinstall
```
Then run the update
```
cp kinetic.rosinstall src/.rosinstall
wstool update -t src
```

Newer versions of pynetworktables no longer support Python 2.7, unfortunately
ROS doesn't really support Python 3 even with the latest Melodic release. To
get a version of pynetworktables that works with Python 2.7:

    sudo pip install pynetworktables==2018.2.1

## Ports and other important information
| Type | Description | Number | Our Use |
| :--- | :----: | :----: | ---: |
| Camera Data | For cameras and video | UDP/TCP 1180 | Available |
| Camera Data | For cameras and video | UDP/TCP 1181 | Available |
| Camera Data | For cameras and video | UDP/TCP 1182 | Available |
| Camera Data | For cameras and video | UDP/TCP 1183 | Available |
| Camera Data | For cameras and video | UDP/TCP 1184 | Available |
| Camera Data | For cameras and video | UDP/TCP 1185 | Available |
| Camera Data | For cameras and video | UDP/TCP 1186 | Available |
| Camera Data | For cameras and video | UDP/TCP 1187 | Available |
| Camera Data | For cameras and video | UDP/TCP 1188 | Available |
| Camera Data | For cameras and video | UDP/TCP 1189 | Available |
| Camera Data | For cameras and video | UDP/TCP 1190 | Available |
| SmartDashboard | Unsure if this also includes other networktables tables | TCP 1735 | Can't use |
| DS-Robot Comms | Contains the data from the DS to the robot | UDP 1130 | Can't use |
| Robot to DS | Unsure what this contans | UDP 1140 | Can't use |
| Camera/Web interface | Unsecure web interface connection | HTTP 80 | Available |
| Camera/Web interface | Secure web interface connection | HTTP 443 | Available |
| h.264 Streaming | Real time streaming | UDP/TCP 554 | Available |
| Team Use | Team Use | UDP/TCP 5800 | Available |
| Team Use | Team Use | UDP/TCP 5801 | Available |
| Team Use | Team Use | UDP/TCP 5802 | Available |
| Team Use | Team Use | UDP/TCP 5803 | Available |
| Team Use | Team Use | UDP/TCP 5804 | Available |
| Team Use | Team Use | UDP/TCP 5805 | Available |
| Team Use | Team Use | UDP/TCP 5806 | Available |
| Team Use | Team Use | UDP/TCP 5807 | Available |
| Team Use | Team Use | UDP/TCP 5808 | Available |
| Team Use | Team Use | UDP/TCP 5809 | Available |
| Team Use | Team Use | UDP/TCP 5810 | Available |

## Communicating with the Robot

We do not want to rely on DNS for communications. Therefore, we need to tell
everything to use IP addresses. On the robot computer, make sure to run the
following command before running any launch files:

    export ROS_IP=X.Y.Z.1

Make sure to use the IP address for the actual interface that outside computers
can access.

On your computer, you need to set both ROS_IP to your computer's IP address,
and set the ROS_MASTER_URI to the robot's computer IP address:

	export ROS_IP=X.Y.Z.2
	export ROS_MASTER_URI=http://X.Y.Z.1:11311

# Running the Code

To run everything on the real robot (laser drivers, odometry, URDF):

	export ROS_IP=10.17.21.106
	roslaunch hungry_toaster hungry_toaster.launch

To simuate network tables for odometry:

	rosrun tidalforce_odom simulated_rio.py

To just view the URDF/meshes:

	roslaunch hungry_toaster view_urdf.launch
