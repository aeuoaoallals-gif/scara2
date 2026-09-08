import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf2_ros
from tf2_ros import TransformException

class RealTrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('real_trajectory_visualizer')
        
        # Публикуем маркер для RViz
        self.publisher_ = self.create_publisher(Marker, '/real_trajectory_marker', 10)
        
        # Инициализируем буфер и слушатель TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Таймер для периодического опроса TF (20 Гц)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Инициализация маркера LINE_STRIP
        self.marker = Marker()
        self.marker.header.frame_id = 'world'  # Фиксированный фрейм из вашего URDF
        self.marker.ns = 'real_trajectory'
        self.marker.id = 0
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        
        # Внешний вид линии (зеленый цвет для отличия от желтой референсной траектории)
        self.marker.scale.x = 0.005  # Толщина линии
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0    
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        
        self.max_points = 20000  # Ограничение на количество точек для экономии памяти (скользящее окно)

    def timer_callback(self):
        try:
            # Получаем трансформ из 'world' в 'tool0' (инструментальная точка робота)
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'world', 'tool0', now, rclpy.duration.Duration(seconds=0.1)
            )
            
            # Создаем новую точку
            p = Point()
            p.x = trans.transform.translation.x
            p.y = trans.transform.translation.y
            p.z = trans.transform.translation.z
            
            # Добавляем точку в маркер
            self.marker.points.append(p)
            
            # Удаляем самые старые точки, если превышен лимит
            if len(self.marker.points) > self.max_points:
                self.marker.points.pop(0)
                
            # Обновляем штамп времени
            self.marker.header.stamp = self.get_clock().now().to_msg()
            
            # Публикуем маркер
            self.publisher_.publish(self.marker)
            
        except TransformException:
            # Трансформ пока недоступен (например, робот еще не заспавнен), просто пропускаем такт
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RealTrajectoryVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

