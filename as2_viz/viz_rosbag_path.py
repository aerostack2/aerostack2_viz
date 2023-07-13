""" A ROS 2 node to publish markers for visualization in RViz """

from typing import List
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from visualization_msgs.msg import Marker
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import rclpy
import rclpy.logging
from as2_viz.tools.utils_ros2 import readRosbag


class RosbagPathPublisherNode(Node):
    """ A ROS node to publish markers for visualization in RViz """

    def __init__(self) -> None:
        """ Initialize the node """
        super().__init__('rosbag_path_publisher_node')

        self.declare_parameter("namespace", 'viz')
        self.namespace = self.get_parameter(
            "namespace").get_parameter_value().string_value

        self.declare_parameter("rosbag_path", "")
        rosbag_path = self.get_parameter(
            "rosbag_path").get_parameter_value().string_value
        if rosbag_path == "":
            self.get_logger().error(f"Rosbag path is empty")
            return

        self.declare_parameter("topic_name", "")
        topic_name = self.get_parameter(
            "topic_name").get_parameter_value().string_value
        if topic_name == "":
            self.get_logger().error(f"Topic name is empty")
            return

        self.declare_parameter("marker.topic_name", "")
        marker_topic_name = self.get_parameter(
            "marker.topic_name").get_parameter_value().string_value
        if marker_topic_name == "":
            marker_topic_name = f'/{self.namespace}{topic_name}'

        self.get_logger().info(f"From rosbag : {rosbag_path}")
        self.get_logger().info(f"Get topic : {topic_name}")
        self.get_logger().info(f"And publish on : {marker_topic_name}")

        self.marker = Marker()
        self.marker.type = Marker.LINE_STRIP
        self.marker.ns = "as2_viz"

        self.marker.color.a = 1.0
        self.declare_parameter("marker.color.r", 0.0)
        self.marker.color.r = self.get_parameter(
            "marker.color.r").get_parameter_value().double_value
        self.declare_parameter("marker.color.g", 1.0)
        self.marker.color.g = self.get_parameter(
            "marker.color.g").get_parameter_value().double_value
        self.declare_parameter("marker.color.b", 0.0)
        self.marker.color.b = self.get_parameter(
            "marker.color.b").get_parameter_value().double_value

        self.declare_parameter("marker.scale.x", 0.1)
        self.marker.scale.x = self.get_parameter(
            "marker.scale.x").get_parameter_value().double_value
        self.declare_parameter("marker.scale.y", 0.1)
        self.marker.scale.y = self.get_parameter(
            "marker.scale.y").get_parameter_value().double_value
        self.declare_parameter("marker.scale.z", 0.1)
        self.marker.scale.z = self.get_parameter(
            "marker.scale.z").get_parameter_value().double_value

        self.marker.points, frame_id = RosbagPathPublisherNode.get_rosbag_data(
            rosbag_path, topic_name)
        if self.marker.points is None:
            return

        if frame_id is None:
            frame_id = "earth"
        self.marker.header.frame_id = frame_id

        self.marker_pub = self.create_publisher(
            Marker, marker_topic_name, qos_profile_system_default
        )

        self.get_logger().info(f"Read {len(self.marker.points)} points")

        self.declare_parameter("publish_rate", 1.0)
        publish_rate = self.get_parameter(
            "publish_rate").get_parameter_value().double_value

        self.get_logger().info(f"Start publishing at {publish_rate} Hz")
        self.timer = self.create_timer(publish_rate, self.publish_marker)

    @staticmethod
    def get_rosbag_data(rosbag_path, topic_name) -> List[Point]:
        """ Get the data from the rosbag """
        topic_msgs, time = readRosbag(
            rosbag_path,
            wanted_topics=[topic_name],
        )
        rclpy.logging.get_logger('rosbag_path_publisher_node').info(
            f"Rosbag time : {time}")

        if len(topic_msgs[0].msgs) == 0:
            rclpy.logging.get_logger('rosbag_path_publisher_node').error(
                "No message found in the rosbag")
            return None

        frame_id = None
        point_list = []
        for point_msgs in topic_msgs[0].msgs:
            point = Point()
            if isinstance(point_msgs, PoseStamped):
                frame_id = point_msgs.header.frame_id
                point.x = point_msgs.pose.position.x
                point.y = point_msgs.pose.position.y
                point.z = point_msgs.pose.position.z
            elif isinstance(point_msgs, PointStamped):
                frame_id = point_msgs.header.frame_id
                point = point_msgs.point
            elif isinstance(point_msgs, Point):
                point = point_msgs
            else:
                rclpy.logging.get_logger('rosbag_path_publisher_node').error(
                    f"Unknown message type : {type(point_msgs)}")
                return None
            point_list.append(point)
        return point_list, frame_id

    def publish_marker(self) -> None:
        """ Publish the path """
        self.get_logger().info("Publishing marker")
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.marker)


def main(args=None):
    """ Main function """

    rclpy.init(args=args)
    marker_publisher_node = RosbagPathPublisherNode()
    rclpy.spin(marker_publisher_node)
    marker_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
