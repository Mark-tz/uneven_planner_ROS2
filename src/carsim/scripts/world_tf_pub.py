#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped

class WorldTfPub(Node):
	def __init__(self):
		super().__init__('world_tf_pub')

		self.last_published_timestamp = self.get_clock().now()
		self.tf_broadcaster = TransformBroadcaster(self)

		self.subscription = self.create_subscription(
			PoseStamped, '/racebot/true_state/center_pose', self.pose_cb, 1000)

	def pose_cb(self, msg):
		pose = msg.pose.position
		orientation = msg.pose.orientation
		
		current_time = self.get_clock().now()
		if self.last_published_timestamp != current_time:
			self.last_published_timestamp = current_time
			
			t = TransformStamped()
			t.header.stamp = current_time.to_msg()
			t.header.frame_id = 'world'
			t.child_frame_id = 'base_footprint'
			t.transform.translation.x = pose.x
			t.transform.translation.y = pose.y
			t.transform.translation.z = pose.z
			t.transform.rotation.x = orientation.x
			t.transform.rotation.y = orientation.y
			t.transform.rotation.z = orientation.z
			t.transform.rotation.w = orientation.w
			
			self.tf_broadcaster.sendTransform(t)

if __name__ == "__main__":
	rclpy.init()
	try:
		node = WorldTfPub()
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	except Exception as e:
		print(f"Cannot start transform publisher: {e}")
	finally:
		rclpy.shutdown()