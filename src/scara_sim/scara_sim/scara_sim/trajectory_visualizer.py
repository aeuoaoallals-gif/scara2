import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        self.publisher = self.create_publisher(Marker, '/trajectory_marker', 10)
        self.declare_parameter('publish_frequency', 10.0)
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.timer = self.create_timer(1/self.publish_frequency, self.publish_marker)  # публикуем раз в секунду (latching не используем)
        self.publish_marker()  # сразу публикуем

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'reference_trajectory'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.01  # толщина линии
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Вершины ломаной из ТЗ
        points = [
            (-0.25, -0.25, 0.15),
            (-0.25, 0.25, 0.15),
            (0.25, 0.25, 0.15),
            (0.25, -0.25, 0.15),
        ]
        for x, y, z in points:
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            marker.points.append(p)

        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()