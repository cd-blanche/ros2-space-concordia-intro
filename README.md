# ros2-space-concordia-intro

Github repository for introduction task to Space Concordia

In this project, a ros2 workspace will be created alongside a custom package that will hold the publisher and subscriber nodes. The publisher node will be written in C++ while the subscriber node will be written in Python.

### What it does

This package allows you to set a custom parameter string that will be built by a virtual player letter by letter. The process can then be subscribed to.

## Installation

### Requirements

- ros2
- colcon
- a cup of wine or alternatively your favourite choice of beverage

#### Build the package

You will need to build the package by running the following command at the root of the directory:

`$ colcon build --packages-select build_my_string`

#### Install dependencies

To make sure you have the required dependencies, run the following command while still in the root of the directory:

`$ rosdep install -i --from-path src --rosdistro humble -y`

#### Source the setup files

For both the publisher and subscriber nodes, run each of these commands in a new terminal to source the setup files:

` $ source /opt/ros/humble/setup.bash`

Then in both terminals, navigate into the root directory of the project and run this command:

`$ source install/setup.bash`

#### Publisher node

Now run the publisher node in one terminal:

`$ ros2 run build_my_string publisher`

#### Subscriber node

And run the subscriber node in the other terminal:

`$ ros2 run build_my_string py_subscriber.py`

## Usage

There are three available parameters that you can set once the publisher node is running:

- message
- base_w (base width)
- base_h (base heigth)

To set any of these parameters, open a new terminal and run any of the following commands.

##### Message

Message allows you to change the string that is being built. Make sure to add double quotes `""` when inputing a new string.

`$ ros2 param set /message_publisher message "<your_string_here>"`

##### base_w

This will allow you to adjust the width of the virtual base on which the player lives on. The value must me an integer. Default set to 20.

`$ ros2 param set /message_publisher base_w <int>`

##### base_h

This will allow you to adjust the height of the virtual base on which the player lives on. The value must me an integer. Default set to 40.

`$ ros2 param set /message_publisher base_h <int>`

### _Enjoy the show!_

Now just sit back, relax, and watch as your new little friend takes the time of his day to do your bidding.

###### _By Jacques Martinez_
