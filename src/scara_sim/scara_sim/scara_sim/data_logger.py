import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import tf2_ros
import csv
import threading
import queue
import os
from datetime import datetime

class ScaraDataLogger(Node):
    def __init__(self):
        super().__init__('scara_data_logger')
        
        self.declare_parameter('csv_filename', 'scara_data.csv')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'tool0')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('ideal_state_topic', '/joint_trajectory_controller/state')
        # Частота записи в Гц (1000 Гц = каждые 0.001 секунды)
        self.declare_parameter('record_frequency', 1000.0)
        
        self.filename = self.get_parameter('csv_filename').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.record_frequency = self.get_parameter('record_frequency').value
        
        # Очередь увеличена для высокой частоты
        self.data_queue = queue.Queue(maxsize=10000)
        
        self.joint_sub = self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self.joint_callback,
            10
        )
        
        self.ideal_sub = self.create_subscription(
            JointTrajectoryControllerState,
            self.get_parameter('ideal_state_topic').value,
            self.ideal_callback,
            10
        )
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.current_js = None
        self.ideal_js = None
        
        # === ТАЙМЕР для записи каждые 0.001 секунды ===
        period = 1.0 / self.record_frequency
        self.timer = self.create_timer(period, self._timer_callback)
        
        self.is_running = True
        self.writer_thread = threading.Thread(target=self._background_writer, daemon=True)
        self.writer_thread.start()
        
        self._init_csv()
        self.get_logger().info(f'🟢 Логгер запущен. Запись идет ПОСТОЯННО с частотой {self.record_frequency} Гц')
        self.get_logger().info(f'Файл: {os.path.abspath(self.filename)}')

    def _init_csv(self):
        if not os.path.isfile(self.filename):
            with open(self.filename, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'Time',
                    'Curr_X', 'Curr_Y', 'Curr_Z',
                    'J1_Angle', 'J2_Angle',
                    'J1_Torque', 'J2_Torque',
                    'Ideal_J1_Angle', 'Ideal_J2_Angle',
                    'Ideal_J1_Torque', 'Ideal_J2_Torque',
                    'Desired_J1_Torque', 'Desired_J2_Torque'
                ])

    def joint_callback(self, msg: JointState):
        self.current_js = msg

    def ideal_callback(self, msg: JointTrajectoryControllerState):
        self.ideal_js = msg

    def _timer_callback(self):
        """Вызывается каждые 0.001 секунды. Пишет данные БЕЗ фильтров."""
        if self.current_js is None:
            return  # Ждем самых первых данных от робота
        
        # 1. Текущие XYZ через TF
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time()
            )
            cx = transform.transform.translation.x
            cy = transform.transform.translation.y
            cz = transform.transform.translation.z
        except Exception:
            cx, cy, cz = 'N/A', 'N/A', 'N/A'

        # 2. Текущие углы и моменты (2 сустава)
        n = min(2, len(self.current_js.position))
        angles = [self.current_js.position[i] for i in range(n)] + [None]*(2-n)
        torques = [self.current_js.effort[i] for i in range(min(2, len(self.current_js.effort)))] + [None]*(2-n)
        
        # 3. Идеальные углы и моменты
        ideal_angles = [None, None]
        ideal_torques = [None, None]
        desired_torques = [None, None]
        
        if self.ideal_js is not None:
            ref_pos = getattr(self.ideal_js.reference, 'positions', [])
            for i in range(min(2, len(ref_pos))):
                ideal_angles[i] = ref_pos[i]
            
            ref_eff = getattr(self.ideal_js.reference, 'effort', [])
            for i in range(min(2, len(ref_eff))):
                ideal_torques[i] = ref_eff[i]
            
            desired = getattr(self.ideal_js, 'desired', None)
            if desired is not None:
                des_eff = getattr(desired, 'effort', [])
                for i in range(min(2, len(des_eff))):
                    desired_torques[i] = des_eff[i]

        record = [
            datetime.now().strftime('%H:%M:%S.%f')[:-3],
            cx, cy, cz,
            *angles,
            *torques,
            *ideal_angles,
            *ideal_torques,
            *desired_torques
        ]
        
        try:
            self.data_queue.put_nowait(record)
        except queue.Full:
            pass  # При очень высокой частоте просто пропускаем кадр, чтобы не было лагов

    def _background_writer(self):
        batch = []
        while self.is_running:
            try:
                record = self.data_queue.get(timeout=0.5)
                batch.append(record)
                
                # Пакетная запись каждые 200 строк для высокой частоты
                if len(batch) >= 200:
                    self._write_batch(batch)
                    batch.clear()
            except queue.Empty:
                if batch:
                    self._write_batch(batch)
                    batch.clear()
            except Exception as e:
                self.get_logger().error(f'Ошибка потока записи: {e}')

    def _write_batch(self, batch):
        try:
            with open(self.filename, mode='a', newline='', encoding='utf-8') as f:
                csv.writer(f).writerows(batch)
        except Exception as e:
            self.get_logger().error(f'Ошибка записи в файл: {e}')

    def destroy_node(self):
        self.is_running = False
        self.writer_thread.join(timeout=2.0)
        batch = []
        while not self.data_queue.empty():
            batch.append(self.data_queue.get_nowait())
        if batch:
            self._write_batch(batch)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ScaraDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()