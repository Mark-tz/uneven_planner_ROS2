#!/usr/bin/env python3

import os
import numpy as np
from math import cos, sin, atan2

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
# 修正消息类型为 ROS2 的 ModelStates
from gazebo_msgs.msg import ModelStates

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

def yaw_from_quaternion(q):
    # 纯计算方式，无需 tf_transformations 依赖
    return atan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

class TrueStatePub(Node):
    def __init__(self):
        super().__init__('true_state_pub')

        self.rear_pose_pub = self.create_publisher(PoseStamped, '/racebot/true_state/rear_pose', 1)
        self.center_pose_pub = self.create_publisher(PoseStamped, '/racebot/true_state/center_pose', 1)
        self.vel_pub = self.create_publisher(TwistStamped, '/racebot/true_state/velocity', 1)
        self.true_odom_pub = self.create_publisher(Odometry, '/racebot/true_state/odom', 1)

        # 订阅类型修正为 ModelStates
        self.subscription = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_cb, 1)

    def model_cb(self,data):
        try:
            vehicle_model_index = data.name.index("racebot")
        except:
            return
        vehicle_position = data.pose[vehicle_model_index]
        vehicle_velocity = data.twist[vehicle_model_index]
        orientation = vehicle_position.orientation
        # 用本地函数计算 yaw，去除 ROS1 的 tf.transformations
        yaw = yaw_from_quaternion(orientation)
        # 用 ROS2 的时钟生成时间戳
        time_stamp = self.get_clock().now().to_msg()

        # vehicle center position
        center_pose = PoseStamped()
        center_pose.header.frame_id = 'world'
        center_pose.header.stamp = time_stamp
        center_pose.pose.position = vehicle_position.position
        center_pose.pose.orientation = vehicle_position.orientation
        self.center_pose_pub.publish(center_pose)

        # vehicle rear axle position
        rear_pose = PoseStamped()
        rear_pose.header.frame_id = 'world'
        rear_pose.header.stamp = time_stamp
        center_x = vehicle_position.position.x
        center_y = vehicle_position.position.y
        rear_x = center_x - cos(yaw) * 0.13
        rear_y = center_y - sin(yaw) * 0.13
        rear_pose.pose.position.x = rear_x
        rear_pose.pose.position.y = rear_y
        rear_pose.pose.orientation = vehicle_position.orientation
        self.rear_pose_pub.publish(rear_pose)

        # vehicle velocity
        velocity = TwistStamped()
        velocity.header.frame_id = ''
        velocity.header.stamp = time_stamp
        velocity.twist.linear = vehicle_velocity.linear
        velocity.twist.angular = vehicle_velocity.angular
        self.vel_pub.publish(velocity)

        # true odom
        true_odom = Odometry()
        true_odom.header.frame_id = 'world'
        true_odom.header.stamp = time_stamp
        true_odom.pose.pose = rear_pose.pose
        true_odom.twist.twist = velocity.twist
        self.true_odom_pub.publish(true_odom)

if __name__ == "__main__":
    rclpy.init()
    try:
        node = TrueStatePub()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Cannot start vehicle pose and velocity updater: {e}")
    finally:
        rclpy.shutdown()