""" A ROS 2 node to publish markers for visualization in RViz """

from visualization_msgs.msg import Marker
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import rclpy
import rclpy.logging


class ModelDaePublisherNode(Node):
    """ A ROS node to publish markers for visualization in RViz """

    def __init__(self) -> None:
        """ Initialize the node """
        super().__init__('rosbag_path_publisher_node')

        self.declare_parameter("namespace", 'viz')
        self.namespace = self.get_parameter(
            "namespace").get_parameter_value().string_value

        self.declare_parameter("marker.topic_name", "")
        marker_topic_name = self.get_parameter(
            "marker.topic_name").get_parameter_value().string_value
        if marker_topic_name == "":
            self.get_logger().error("Marker topic name is empty")
            return

        self.marker = Marker()
        self.marker.type = Marker.MESH_RESOURCE
        self.marker.mesh_use_embedded_materials = True
        self.marker.ns = "as2_viz"

        self.declare_parameter("marker.id", 0)
        self.marker.id = self.get_parameter(
            "marker.id").get_parameter_value().integer_value

        self.declare_parameter("marker.mesh_resource", "")
        self.marker.mesh_resource = self.get_parameter(
            "marker.mesh_resource").get_parameter_value().string_value
        if self.marker.mesh_resource == "":
            self.get_logger().error("Mesh resource path is empty")
            return

        self.declare_parameter("marker.frame_id", "")
        self.marker.header.frame_id = self.get_parameter(
            "marker.frame_id").get_parameter_value().string_value
        if self.marker.header.frame_id  == "":
            self.get_logger().error("Marker frame id is empty")
            return

        self.declare_parameter("marker.position.x", 0.0)
        self.marker.pose.position.x = self.get_parameter(
            "marker.position.x").get_parameter_value().double_value
        self.declare_parameter("marker.position.y", 0.0)
        self.marker.pose.position.y = self.get_parameter(
            "marker.position.y").get_parameter_value().double_value
        self.declare_parameter("marker.position.z", 0.0)
        self.marker.pose.position.z = self.get_parameter(
            "marker.position.z").get_parameter_value().double_value

        self.declare_parameter("marker.orientation.w", 1.0)
        self.marker.pose.orientation.w = self.get_parameter(
            "marker.orientation.w").get_parameter_value().double_value
        self.declare_parameter("marker.orientation.x", 0.0)
        self.marker.pose.orientation.x = self.get_parameter(
            "marker.orientation.x").get_parameter_value().double_value
        self.declare_parameter("marker.orientation.y", 0.0)
        self.marker.pose.orientation.y = self.get_parameter(
            "marker.orientation.y").get_parameter_value().double_value
        self.declare_parameter("marker.orientation.z", 0.0)
        self.marker.pose.orientation.z = self.get_parameter(
            "marker.orientation.z").get_parameter_value().double_value

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

        self.marker_pub = self.create_publisher(
            Marker, marker_topic_name, qos_profile_system_default
        )

        self.declare_parameter("publish_rate", 1.0)
        publish_rate = self.get_parameter(
            "publish_rate").get_parameter_value().double_value

        self.get_logger().info(f"Start publishing at {publish_rate} Hz")
        self.timer = self.create_timer(publish_rate, self.publish_marker)

    def publish_marker(self) -> None:
        """ Publish the path """
        self.get_logger().info("Publishing marker")
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.marker)


def main(args=None):
    """ Main function """

    rclpy.init(args=args)
    marker_publisher_node = ModelDaePublisherNode()
    rclpy.spin(marker_publisher_node)
    marker_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
