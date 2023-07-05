""" A ROS 2 node to publish markers for visualization in RViz """

from collections import deque
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TwistStamped
from visualization_msgs.msg import Marker
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import rclpy
from rclpy.duration import Duration


class MarkerPublisherNode(Node):
    """ A ROS node to publish markers for visualization in RViz """
    MARKERS_FREQ = 0.1
    MARKERS_LIFETIME = 5  # seconds
    RECORD_LENGTH = 200

    def __init__(self, namespace: str, use_sim_time: bool = False) -> None:
        """ Initialize the node """
        super().__init__('marker_publisher_node')

        self.param_use_sim_time = Parameter(
            'use_sim_time', Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([self.param_use_sim_time])

        self.pose_sub = self.create_subscription(
            PoseStamped, f'/{namespace}/self_localization/pose',
            self.pose_callback, qos_profile_system_default)
        self.twist_sub = self.create_subscription(
            TwistStamped, f'/{namespace}/self_localization/twist',
            self.twist_callback, qos_profile_system_default)
        self.pose_ref_sub = self.create_subscription(
            PoseStamped, f'/{namespace}/motion_reference/pose',
            self.ref_pose_callback, qos_profile_system_default)

        self.marker_pub = self.create_publisher(
            Marker, 'viz/reference_pose', qos_profile_system_default
        )
        self.vel_pub = self.create_publisher(
            Marker, 'viz/vel', qos_profile_system_default
        )
        self.traj_pub = self.create_publisher(
            Path, 'viz/last_poses', qos_profile_system_default
        )

        self.poses_record: deque[PoseStamped] = deque(maxlen=200)

        self.marker = Marker()
        self.marker.ns = 'am'
        self.marker.id = 33
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.lifetime = Duration(seconds=self.MARKERS_LIFETIME).to_msg()
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.g = 1.0
        self.marker.color.a = 1.0

        self.timer = self.create_timer(self.MARKERS_FREQ, self.publish_markers)

    def twist_callback(self, msg: TwistStamped) -> None:
        """Twist callback"""
        vel = Marker()

        vel.type = Marker.ARROW
        vel.action = Marker.ADD

        vel.scale.x = 0.1
        vel.scale.y = 0.1
        vel.scale.z = 0.1
        vel.color.g = 1.0
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
            last_poses = Path()
            last_poses.header = self.poses_record[0].header
            last_poses.poses = list(self.poses_record)

            self.traj_pub.publish(last_poses)


def main(args=None):
    """ Main function """

    rclpy.init(args=args)
    marker_publisher_node = MarkerPublisherNode('drone0', True)
    rclpy.spin(marker_publisher_node)
    marker_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
