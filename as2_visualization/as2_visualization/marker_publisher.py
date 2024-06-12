""" A ROS 2 node to publish markers for visualization in RViz """

import random
from collections import deque
from geometry_msgs.msg import PoseStamped, TwistStamped
from visualization_msgs.msg import Marker
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
import rclpy
from rclpy.duration import Duration

# TODO: choose btter names


class MarkerPublisherNode(Node):
    """ A ROS node to publish markers for visualization in RViz """
    MARKERS_FREQ = 0.1
    MARKERS_LIFETIME = 5  # seconds
    PATH_LIFETIME = 5  # seconds

    def __init__(self) -> None:
        """ Initialize the node """
        super().__init__('marker_publisher_node')

        self.declare_parameter("namespace", 'drone0')
        self.namespace = self.get_parameter(
            "namespace").get_parameter_value().string_value

        self.declare_parameter("record_length", 500)
        record_length = self.get_parameter(
            "record_length").get_parameter_value().integer_value

        self.get_logger().info(f"Using drone namespace: {self.namespace}")

        self.pose_sub = self.create_subscription(
            PoseStamped, f'/{self.namespace}/self_localization/pose',
            self.pose_callback, qos_profile_sensor_data)
        self.twist_sub = self.create_subscription(
            TwistStamped, f'/{self.namespace}/self_localization/twist',
            self.twist_callback, qos_profile_sensor_data)
        self.pose_ref_sub = self.create_subscription(
            PoseStamped, f'/{self.namespace}/motion_reference/pose',
            self.ref_pose_callback, qos_profile_sensor_data)

        self.marker_pub = self.create_publisher(
            Marker, f'/viz/{self.namespace}/reference_pose', qos_profile_system_default
        )
        self.vel_pub = self.create_publisher(
            Marker, f'/viz/{self.namespace}/vel', qos_profile_system_default
        )
        self.path_pub = self.create_publisher(
            Marker, f'/viz/{self.namespace}/last_poses', qos_profile_system_default
        )

        self.poses_record: deque[PoseStamped] = deque(
            maxlen=record_length)

        self.color = [random.random(), random.random(), random.random()]
        self.marker = Marker()
        self.marker.ns = 'am'
        self.marker.id = 33
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.lifetime = Duration(seconds=self.MARKERS_LIFETIME).to_msg()
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.r = self.color[0]
        self.marker.color.g = self.color[1]
        self.marker.color.b = self.color[2]
        self.marker.color.a = 1.0

        self.path = Marker()
        self.path.ns = 'am'
        self.path.id = 34
        self.path.type = Marker.POINTS
        self.path.action = Marker.ADD
        self.path.lifetime = Duration(seconds=self.PATH_LIFETIME).to_msg()
        self.path.scale.x = 0.01
        self.path.scale.y = 0.01
        self.path.color.r = self.color[0]
        self.path.color.g = self.color[1]
        self.path.color.b = self.color[2]
        self.path.color.a = 1.0

        self.timer = self.create_timer(self.MARKERS_FREQ, self.publish_markers)

    def twist_callback(self, msg: TwistStamped) -> None:
        """Twist callback"""
        vel = Marker()

        vel.type = Marker.ARROW
        vel.action = Marker.ADD

        vel.scale.x = 0.1
        vel.scale.y = 0.1
        vel.scale.z = 0.1
        vel.color.r = self.color[0]
        vel.color.g = self.color[1]
        vel.color.b = self.color[2]
        vel.color.a = 1.0
        vel.header = msg.header
        vel.pose.position.x = msg.twist.linear.x
        vel.pose.position.y = msg.twist.linear.y
        vel.pose.position.z = msg.twist.linear.z

        self.vel_pub.publish(vel)

    def pose_callback(self, msg: PoseStamped) -> None:
        """Pose callback"""
        self.poses_record.append(msg)

    def ref_pose_callback(self, msg: PoseStamped) -> None:
        """Motion reference pose callback"""
        self.marker.header = msg.header
        self.marker.pose = msg.pose

        self.marker_pub.publish(self.marker)

    def publish_markers(self) -> None:
        """ Publish the markers """

        if self.poses_record:
            self.path.header = self.poses_record[0].header
            self.path.points = [pose.pose.position for pose in self.poses_record]
            self.path_pub.publish(self.path)


def main(args=None):
    """ Main function """

    rclpy.init(args=args)
    marker_publisher_node = MarkerPublisherNode()
    rclpy.spin(marker_publisher_node)
    marker_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
