#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Vector3Stamped, PointStamped, PoseStamped
import tf2_ros
import tf2_geometry_msgs

class OdomTransformNode:
    def __init__(self):
        rospy.init_node('odom_transform_node')

        self.target_frame = rospy.get_param('~target_frame', 'odom')
        self.output_child_frame = rospy.get_param('~output_child_frame', 'base_link')
        self.input_topic = rospy.get_param('~input_topic', '/input_odom')
        self.output_topic = rospy.get_param('~output_topic', '/output_odom')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.odom_sub = rospy.Subscriber(self.input_topic, Odometry, self.odom_callback)
        self.odom_pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)

    def odom_callback(self, msg):
        try:
            # Transform the odometry message to the target frame
            transform_to_target = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, msg.header.stamp, rospy.Duration(1.0))
            
            transformed_odom = Odometry()
            transformed_odom.header = msg.header
            transformed_odom.header.frame_id = self.target_frame
            
            # Transform position
            pos_stamped = PointStamped()
            pos_stamped.header = msg.header
            pos_stamped.point = msg.pose.pose.position
            transformed_pos = tf2_geometry_msgs.do_transform_point(pos_stamped, transform_to_target)
            transformed_odom.pose.pose.position = transformed_pos.point

            # Transform orientation
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform_to_target)
            transformed_odom.pose.pose.orientation = transformed_pose.pose.orientation
            
            # Transform velocity
            velocity_transform = self.tf_buffer.lookup_transform(self.output_child_frame, msg.child_frame_id, msg.header.stamp, rospy.Duration(1.0))
            
            # Create a Vector3Stamped for linear velocity
            linear_vel = Vector3Stamped()
            linear_vel.vector = msg.twist.twist.linear
            linear_vel.header.frame_id = msg.child_frame_id
            linear_vel.header.stamp = msg.header.stamp
            
            # Transform linear velocity
            transformed_linear_vel = tf2_geometry_msgs.do_transform_vector3(linear_vel, velocity_transform)
            
            # Create a Vector3Stamped for angular velocity
            angular_vel = Vector3Stamped()
            angular_vel.vector = msg.twist.twist.angular
            angular_vel.header.frame_id = msg.child_frame_id
            angular_vel.header.stamp = msg.header.stamp
            
            # Transform angular velocity
            transformed_angular_vel = tf2_geometry_msgs.do_transform_vector3(angular_vel, velocity_transform)
            
            # Set the transformed velocities
            transformed_odom.twist.twist.linear = transformed_linear_vel.vector
            transformed_odom.twist.twist.angular = transformed_angular_vel.vector
            
            transformed_odom.child_frame_id = self.output_child_frame
            
            self.odom_pub.publish(transformed_odom)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF2 exception: {e}")

if __name__ == '__main__':
    try:
        node = OdomTransformNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
