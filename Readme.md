### Description

Simple ROS package to convert odometry topics to different frames. Tested in ROS Noetic.

### Usage

You can launch the node using the provided launch file. It takes parameters to configure the frame transformations, input, and output topics:

roslaunch odom_transform odom_transform.launch

Parameters

The following parameters can be set via the launch file or from the parameter server:

    target_frame (string, default: odom): The frame to which the odometry data should be transformed.
    input_topic (string, default: /input_odom): The topic to subscribe to for the input odometry messages.
    output_topic (string, default: /output_odom): The topic where the transformed odometry messages will be published.
    output_child_frame (string, default: base_link): The child frame of the transformed odometry data.


