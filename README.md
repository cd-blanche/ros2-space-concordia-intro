# ros2-space-concordia-intro

Github repository for introduction task to Space Concordia

In this project, a ros2 workspace will be created alongside a custom package that will hold the publisher and subscriber nodes. The subscriber node will be written in C++ while the publisher node will be written in Python.

### What it does

This package allows you to set a custom parameter string that will be published letter by letter to the console.

### Installation

#### Build the package

You will need to build the package by running the following command at the root of the directory:

`$ colcon build --packages-select <pkg_name>`

#### Install dependencies

To make sure you have therequired dependencies, run the following command:

`$ rosdep install -i --from-path src --rosdistro humble -y`

#### Source the setup files

For both the publisher and subscriber nodes, run each of these commands in a new terminal to source the setup files:

` $ source /opt/ros/humble/setup.bash`

Then navigate into the root directory of the project and run:

`$ source install/setup.bash`

#### Publisher node

Now run the publisher node in one terminal:

`$ ros2 run <package_name> <publisher_executable_name>`

#### Subscriber node

And run the subscriber node in the other terminal:

`$ ros2 run <package_name> <subscriber_executable_name>`

###### _By Jacques Martinez_
