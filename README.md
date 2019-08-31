# LAPIS

## Installation Notes

Newer versions of pynetworktables no longer support Python 2.7, unfortunately
ROS doesn't really support Python 3 even with the latest Melodic release. To
get a version of pynetworktables that works with Python 2.7:

    sudo pip install pynetworktables==2018.2.1

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

# Testing
Launch `fakeOdom` or `flakeOdom` first for testing, it will provide
networktables data in place of the rio data and will simulate changing encoder
values. The tidalforceodom node will generate /odom and when pointed toward
127.0.0.1 it will read the fake data from the encoders
