import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import csv
import os
from collections import deque
from ament_index_python.packages import get_package_share_directory

class EffortControllerWithFeedback(Node):
    def __init__(self):
        super().__init__('effort_controller_with_feedback')
        
        # ═══════════════════════════════════════════════════════
        # ПАРАМЕТРЫ РОБОТА
        # ═══════════════════════════════════════════════════════
        self.L1 = 0.35
        self.L2 = 0.25
        self.m1 = 1.924
        self.m2 = 1.374
        self.l1 = self.L1 / 2.0
        self.l2 = self.L2 / 2.0
        self.I1zz = (1/12) * self.m1 * (3*(0.025)**2 + self.L1**2)
        self.I2zz = (1/12) * self.m2 * (3*(0.025)**2 + self.L2**2)
        self.b1 = 0.1
        self.b2 = 0.1
        
        # ═══════════════════════════════════════════════════════
        # КОЭФФИЦИЕНТЫ И ДОПУСК
        # ═══════════════════════════════════════════════════════
        self.Kp_cartesian = 50.0
        self.Kd_cartesian = 10.0
        self.tolerance = 0.01  # 1 см допуск
        
        # ═══════════════════════════════════════════════════════
        # СГЛАЖИВАНИЕ
        # ═══════════════════════════════════════════════════════
        self.smoothing_window = 50  # окно сглаживания (количество точек)
        self.max_tau_rate = 2.0  # максимальное изменение момента за шаг (Н·м)
        self.prev_tau = np.array([0.0, 0.0])
        self.tau_buffer = deque(maxlen=self.smoothing_window)
        
        # ═══════════════════════════════════════════════════════
        # ПОДПИСКА И ПУБЛИКАЦИЯ
        # ═══════════════════════════════════════════════════════
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.torque_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller/commands', 10)
        
        # ═══════════════════════════════════════════════════════
        # ЗАГРУЗКА И СГЛАЖИВАНИЕ ТРАЕКТОРИИ
        # ═══════════════════════════════════════════════════════
        self.trajectory = self.load_and_smooth_trajectory()
        self.start_time = None
        
        # ═══════════════════════════════════════════════════════
        # СОСТОЯНИЕ
        # ══════════════════════════════════════════════════════
        self.q = np.array([0.0, 0.0])
        self.dq = np.array([0.0, 0.0])
        self.has_state = False
        
        self.get_logger().info("=" * 70)
        self.get_logger().info("Effort Controller с коррекцией и сглаживанием")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"Loaded {len(self.trajectory)} trajectory points")
        self.get_logger().info(f"Smoothing window: {self.smoothing_window} points")
        self.get_logger().info(f"Max torque rate: {self.max_tau_rate} N·m/step")
        self.get_logger().info(f"Tolerance: {self.tolerance*100:.1f} cm")
        if self.trajectory:
            self.get_logger().info(f"First point: t={self.trajectory[0]['t']:.3f}s, pos=({self.trajectory[0]['x']:.3f}, {self.trajectory[0]['y']:.3f})")
            self.get_logger().info(f"Last point:  t={self.trajectory[-1]['t']:.3f}s, pos=({self.trajectory[-1]['x']:.3f}, {self.trajectory[-1]['y']:.3f})")
        
        # Таймер 1000 Гц
        self.timer = self.create_timer(0.001, self.control_loop)

    def joint_state_callback(self, msg):
        self.q[0] = msg.position[0]
        self.q[1] = msg.position[1]
        self.dq[0] = msg.velocity[0]
        self.dq[1] = msg.velocity[1]
        self.has_state = True

    def load_and_smooth_trajectory(self):
        """Загрузка и сглаживание траектории"""
        try:
            pkg_path = get_package_share_directory('scara_sim')
            csv_path = os.path.join(pkg_path, 'data', 'moments.csv')
            
            trajectory = []
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)
                next(reader)
                for row in reader:
                    trajectory.append({
                        't': float(row[0]),
                        'x': float(row[1]),
                        'y': float(row[2]),
                        'q1': float(row[3]),
                        'q2': float(row[4]),
                        'q1_dot': float(row[5]),
                        'q2_dot': float(row[6]),
                        'q1_ddot': float(row[7]),
                        'q2_ddot': float(row[8]),
                        'tau1': float(row[9]),
                        'tau2': float(row[10])
                    })
            
            # Сглаживание моментов скользящим средним
            if len(trajectory) > self.smoothing_window:
                self.get_logger().info("Applying smoothing to trajectory...")
                for i in range(len(trajectory)):
                    start_idx = max(0, i - self.smoothing_window // 2)
                    end_idx = min(len(trajectory), i + self.smoothing_window // 2)
                    
                    window = trajectory[start_idx:end_idx]
                    avg_tau1 = sum(p['tau1'] for p in window) / len(window)
                    avg_tau2 = sum(p['tau2'] for p in window) / len(window)
                    
                    trajectory[i]['tau1'] = avg_tau1
                    trajectory[i]['tau2'] = avg_tau2
            
            return trajectory
        except Exception as e:
            self.get_logger().error(f"Failed to load CSV: {e}")
            return []

    def forward_kinematics(self, q1, q2):
        x = self.L1 * np.cos(q1) + self.L2 * np.cos(q1 + q2)
        y = self.L1 * np.sin(q1) + self.L2 * np.sin(q1 + q2)
        return x, y

    def compute_jacobian(self, q1, q2):
        s1 = np.sin(q1)
        c1 = np.cos(q1)
        s12 = np.sin(q1 + q2)
        c12 = np.cos(q1 + q2)
        
        J = np.array([
            [-self.L1*s1 - self.L2*s12, -self.L2*s12],
            [ self.L1*c1 + self.L2*c12,  self.L2*c12]
        ])
        return J

    def apply_rate_limiting(self, tau_desired):
        """Ограничение скорости изменения моментов"""
        tau_change = tau_desired - self.prev_tau
        
        # Ограничиваем изменение
        if np.any(np.abs(tau_change) > self.max_tau_rate):
            tau_change = np.clip(tau_change, -self.max_tau_rate, self.max_tau_rate)
            tau_smoothed = self.prev_tau + tau_change
        else:
            tau_smoothed = tau_desired
        
        self.prev_tau = tau_smoothed.copy()
        return tau_smoothed

    def control_loop(self):
        if not self.has_state:
            return
        
        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info("Starting control with smoothing!")
        
        t_sim = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # ═══════════════════════════════════════════════════════
        # 1. НАЙТИ ЖЕЛАЕМУЮ ТОЧКУ ТРАЕКТОРИИ
        # ═══════════════════════════════════════════════════════
        if not self.trajectory or t_sim > self.trajectory[-1]['t']:
            if self.trajectory:
                ref = self.trajectory[-1]
            else:
                return
        else:
            idx = min(range(len(self.trajectory)), 
                     key=lambda i: abs(self.trajectory[i]['t'] - t_sim))
            ref = self.trajectory[idx]
        
        # ═══════════════════════════════════════════════════════
        # 2. ВЫЧИСЛИТЬ ТЕКУЩЕЕ ПОЛОЖЕНИЕ КОНЦА
        # ═══════════════════════════════════════════════════════
        x_curr, y_curr = self.forward_kinematics(self.q[0], self.q[1])
        
        # ═══════════════════════════════════════════════════════
        # 3. ОШИБКА ПОЛОЖЕНИЯ
        # ═══════════════════════════════════════════════════════
        pos_error_x = ref['x'] - x_curr
        pos_error_y = ref['y'] - y_curr
        pos_error = np.sqrt(pos_error_x**2 + pos_error_y**2)
        
        # ═══════════════════════════════════════════════════════
        # 4. ПД-РЕГУЛЯТОР С ДОПУСКОМ
        # ═══════════════════════════════════════════════════════
        if pos_error < self.tolerance:
            tau_correction = np.array([0.0, 0.0])
        else:
            Fx = self.Kp_cartesian * pos_error_x
            Fy = self.Kp_cartesian * pos_error_y
            
            J = self.compute_jacobian(self.q[0], self.q[1])
            tau_correction = J.T @ np.array([Fx, Fy])
        
        # ═══════════════════════════════════════════════════════
        # 5. БАЗОВЫЕ МОМЕНТЫ ИЗ CSV (уже сглаженные)
        # ═══════════════════════════════════════════════════════
        tau_ff = np.array([ref['tau1'], ref['tau2']])
        
        # ═══════════════════════════════════════════════════════
        # 6. ИТОГОВЫЕ МОМЕНТЫ
        # ═══════════════════════════════════════════════════════
        tau_total = tau_ff + tau_correction
        tau_total = np.clip(tau_total, -50.0, 50.0)
        
        # ═══════════════════════════════════════════════════════
        # 7. СГЛАЖИВАНИЕ МОМЕНТОВ (RATE LIMITING)
        # ═══════════════════════════════════════════════════════
        tau_smoothed = self.apply_rate_limiting(tau_total)
        
        # Публикация
        msg = Float64MultiArray()
        msg.data = [float(tau_smoothed[0]), float(tau_smoothed[1])]
        self.torque_pub.publish(msg)
        
        # Логирование
        if not hasattr(self, 'log_counter'):
            self.log_counter = 0
        self.log_counter += 1
        
        if self.log_counter % 1000 == 0:
            correction_status = "NO CORRECTION" if pos_error < self.tolerance else "CORRECTING"
            self.get_logger().info(
                f"t={t_sim:.2f}s | "
                f"pos=({x_curr:.3f},{y_curr:.3f}) | "
                f"ref=({ref['x']:.3f},{ref['y']:.3f}) | "
                f"err={pos_error*1000:.1f}mm [{correction_status}] | "
                f"tau_raw=[{tau_total[0]:.2f},{tau_total[1]:.2f}] | "
                f"tau_smooth=[{tau_smoothed[0]:.2f},{tau_smoothed[1]:.2f}]"
            )

def main(args=None):
    rclpy.init(args=args)
    node = EffortControllerWithFeedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()