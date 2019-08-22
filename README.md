# LAPIS

## Installation Notes

    sudo pip install pynetworktables==2018.2.1

# Testing
Launch `fakeOdom` or `flakeOdom` first for testing, it will provide networktables data in place of the rio data and will simulate changing encoder values
tidalforceodom will generate /odom and when pointed toward 127.0.0.1 it will read the fake data from the encoders

