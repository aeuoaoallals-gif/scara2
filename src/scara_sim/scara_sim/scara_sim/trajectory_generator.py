import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import List, Tuple
import csv
from ament_index_python.packages import get_package_share_directory
import os

# ------------------------- Параметры робота -------------------------
L1 = 0.35          # м
L2 = 0.25          # м
rho = 2800.0       # кг/м³
d = 0.05           # м
m1 = rho * np.pi * (d/2)**2 * L1   # ≈ 1.924 кг
m2 = rho * np.pi * (d/2)**2 * L2   # ≈ 1.374 кг
l1 = L1/2
l2 = L2/2
I1zz = (1/12) * m1 * (3*(d/2)**2 + L1**2)
I2zz = (1/12) * m2 * (3*(d/2)**2 + L2**2)
b1 = b2 = 0.1      # вязкое трение

# ------------------------- Dataclasses -------------------------
@dataclass
class TrajectoryPoint:
    t: float               # время от начала движения, с
    x: float               # декартовы координаты конца второго звена
    y: float
    q1: float              # обобщённые координаты
    q2: float
    q1_dot: float          # скорости
    q2_dot: float
    q1_ddot: float         # ускорения
    q2_ddot: float
    tau1: float            # моменты в сочленениях
    tau2: float

@dataclass
class Segment:
    start: Tuple[float, float]   # начальная точка (x, y)
    end: Tuple[float, float]     # конечная точка (x, y)
    duration: float              # время движения по сегменту, с

# ------------------------- Функции IK и динамики -------------------------
def ik(x: float, y: float, q_prev: Tuple[float, float]) -> Tuple[float, float]:
    """Аналитическое решение обратной кинематики с выбором ветви по непрерывности."""
    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    D = np.clip(D, -1.0, 1.0)
    q2_candidates = [
        np.arctan2(np.sqrt(1 - D**2), D),
        np.arctan2(-np.sqrt(1 - D**2), D)
    ]
    # Выбираем кандидата, ближайшего к предыдущему q2
    q2_new = min(q2_candidates, key=lambda q: abs(q - q_prev[1]))
    q1_new = np.arctan2(y, x) - np.arctan2(L2 * np.sin(q2_new), L1 + L2 * np.cos(q2_new))
    return q1_new, q2_new

def compute_dynamics(q1: float, q2: float, q1_dot: float, q2_dot: float,
                     q1_ddot: float, q2_ddot: float) -> Tuple[float, float]:
    """Вычисление моментов по уравнению обратной динамики (без импульсов)."""
    M11 = I1zz + I2zz + m1*l1**2 + m2*(L1**2 + l2**2 + 2*L1*l2*np.cos(q2))
    M12 = I2zz + m2*(l2**2 + L1*l2*np.cos(q2))
    M22 = I2zz + m2*l2**2

    c11 = -m2 * L1 * l2 * np.sin(q2) * q2_dot
    c12 = -m2 * L1 * l2 * np.sin(q2) * (q1_dot + q2_dot)
    c21 = m2 * L1 * l2 * np.sin(q2) * q1_dot
    c22 = 0.0

    tau1 = M11*q1_ddot + M12*q2_ddot + c11*q1_dot + c12*q2_dot + b1*q1_dot
    tau2 = M12*q1_ddot + M22*q2_ddot + c21*q1_dot + c22*q2_dot + b2*q2_dot
    return tau1, tau2

def generate_trajectory(segments: List[Segment], dt: float = 0.001) -> List[TrajectoryPoint]:
    """
    Генерация точек траектории с расчётом кинематики и динамики.
    """
    points: List[TrajectoryPoint] = []
    current_t = 0.0

    # Начальные значения: предполагаем, что робот уже находится в начале первого сегмента
    start_x, start_y = segments[0].start
    q1, q2 = ik(start_x, start_y, (0.0, 0.0))  # начальное приближение
    q1_dot = q2_dot = 0.0
    # Переменные для хранения скоростей в конце предыдущего шага (для импульсов)
    q1_dot_old = 0.0
    q2_dot_old = 0.0

    # Для каждого сегмента
    for seg_idx, seg in enumerate(segments):
        x_start, y_start = seg.start
        x_end, y_end = seg.end
        T = seg.duration
        L = np.hypot(x_end - x_start, y_end - y_start)
        v = L / T if T > 0 else 0.0
        # Единичный вектор направления
        if L > 1e-12:
            ux = (x_end - x_start) / L
            uy = (y_end - y_start) / L
        else:
            ux = uy = 0.0

        # Вектор декартовой скорости на сегменте
        xdot = v * ux
        ydot = v * uy

        # Число шагов на сегменте
        n_steps = max(1, int(round(T / dt)))
        t_seg = np.linspace(0, T, n_steps + 1)  # включая конечную точку

        for i, tau in enumerate(t_seg):
            t_abs = current_t + tau
            # Координаты в данный момент
            x = x_start + xdot * tau
            y = y_start + ydot * tau

            # IK
            q_prev = (q1, q2)
            q1, q2 = ik(x, y, q_prev)

            # Якобиан и его производная
            s1, c1 = np.sin(q1), np.cos(q1)
            s12, c12 = np.sin(q1+q2), np.cos(q1+q2)
            J = np.array([
                [-L1*s1 - L2*s12, -L2*s12],
                [ L1*c1 + L2*c12,  L2*c12]
            ])
            # Скорости в суставах
            xy_dot = np.array([xdot, ydot])
            q_dot = np.linalg.solve(J, xy_dot)
            q1_dot_new, q2_dot_new = q_dot

            # Ускорение в суставах (для постоянной декартовой скорости)
            dJ = np.array([
                [-L1*c1*q1_dot_new - L2*c12*(q1_dot_new+q2_dot_new),
                 -L2*c12*(q1_dot_new+q2_dot_new)],
                [-L1*s1*q1_dot_new - L2*s12*(q1_dot_new+q2_dot_new),
                 -L2*s12*(q1_dot_new+q2_dot_new)]
            ])
            q_ddot = -np.linalg.solve(J, dJ @ q_dot)
            q1_ddot, q2_ddot = q_ddot

            # Обработка импульсного момента на стыке сегментов (включая первый сегмент)
            if i == 0:  # начало любого сегмента
                delta_q_dot = np.array([q1_dot_new - q1_dot_old,
                                        q2_dot_new - q2_dot_old])
                # Матрица инерции для текущей конфигурации
                M11 = I1zz + I2zz + m1*l1**2 + m2*(L1**2 + l2**2 + 2*L1*l2*np.cos(q2))
                M12 = I2zz + m2*(l2**2 + L1*l2*np.cos(q2))
                M22 = I2zz + m2*l2**2
                M = np.array([[M11, M12], [M12, M22]])
                impulse = M @ delta_q_dot
                tau_imp = impulse / dt
                # Заменяем моменты на импульсные
                tau1, tau2 = tau_imp[0], tau_imp[1]
            else:
                # Обычный расчёт моментов
                tau1, tau2 = compute_dynamics(q1, q2, q1_dot_new, q2_dot_new,
                                             q1_ddot, q2_ddot)

            # Сохраняем точку
            points.append(TrajectoryPoint(
                t=t_abs,
                x=x, y=y,
                q1=q1, q2=q2,
                q1_dot=q1_dot_new, q2_dot=q2_dot_new,
                q1_ddot=q1_ddot, q2_ddot=q2_ddot,
                tau1=tau1, tau2=tau2
            ))

            # Обновляем сохранённые скорости для следующего шага
            q1_dot_old = q1_dot_new
            q2_dot_old = q2_dot_new
            q1_dot, q2_dot = q1_dot_new, q2_dot_new

        # Переход к следующему сегменту
        current_t += T

    # Добавляем конечный импульс (остановка)
    if points:
        last = points[-1]
        delta_q_dot = np.array([-last.q1_dot, -last.q2_dot])
        M11 = I1zz + I2zz + m1*l1**2 + m2*(L1**2 + l2**2 + 2*L1*l2*np.cos(last.q2))
        M12 = I2zz + m2*(l2**2 + L1*l2*np.cos(last.q2))
        M22 = I2zz + m2*l2**2
        M = np.array([[M11, M12], [M12, M22]])
        impulse = M @ delta_q_dot
        tau_imp = impulse / dt
        last.tau1 = tau_imp[0]
        last.tau2 = tau_imp[1]

        # Дополнительная точка с нулевыми моментами
        zero_point = TrajectoryPoint(
            t=last.t + dt,
            x=last.x,
            y=last.y,
            q1=last.q1,
            q2=last.q2,
            q1_dot=0.0,
            q2_dot=0.0,
            q1_ddot=0.0,
            q2_ddot=0.0,
            tau1=0.0,
            tau2=0.0
        )
        points.append(zero_point)

    return points

# ------------------------- Основная программа -------------------------
def main():
    # Задание траектории: ломаная из 3 сегментов (БЕЗ возврата в начало!)
    segments = [
        Segment((0.0, -0.5),  (-0.25, -0.25), 10),
        Segment((-0.25, -0.25),  (-0.25, 0.25), 10),
        Segment((-0.25, 0.25), (0.25, 0.25),  10),
        Segment((0.25, 0.25),  (0.25, -0.25), 10)
        Segment((0.25, -0.25),  (0.0 -0.5), 10),
    ]

    dt = 0.001
    points = generate_trajectory(segments, dt)

    # Извлекаем данные для графиков
    time = [p.t for p in points]
    q1 = [p.q1 for p in points]
    q2 = [p.q2 for p in points]
    tau1 = [p.tau1 for p in points]
    tau2 = [p.tau2 for p in points]
    
    print(f'{q1[0]=}, {q2[0]=} ')
    
    # ═══════════════════════════════════════════════════════════════
    # СОХРАНЕНИЕ ВСЕХ ДАННЫХ В CSV (обновлённый формат)
    # ═══════════════════════════════════════════════════════════════
    pkg_scara_sim = get_package_share_directory('scara_sim')
    data_dir = os.path.join(pkg_scara_sim, 'data')
    os.makedirs(data_dir, exist_ok=True)
    file_path = os.path.join(data_dir, 'moments.csv')

    with open(file_path, 'w', newline='') as f:
        writer = csv.writer(f)
        # Заголовок: время, x, y, q1, q2, q1_dot, q2_dot, q1_ddot, q2_ddot, tau1, tau2
        writer.writerow(['time', 'x', 'y', 'q1', 'q2', 'q1_dot', 'q2_dot', 'q1_ddot', 'q2_ddot', 'tau1', 'tau2'])
        for p in points:
            writer.writerow([
                p.t, 
                p.x, p.y, 
                p.q1, p.q2, 
                p.q1_dot, p.q2_dot, 
                p.q1_ddot, p.q2_ddot, 
                p.tau1, p.tau2
            ])
    print(f"CSV saved to {file_path}")
    print(f"Total points: {len(points)}")
    print(f"Duration: {points[-1].t:.2f}s")
    print(f"First point: t={points[0].t:.3f}, x={points[0].x:.3f}, y={points[0].y:.3f}")
    print(f"Last point:  t={points[-1].t:.3f}, x={points[-1].x:.3f}, y={points[-1].y:.3f}")

    # Построение графиков с логарифмической шкалой для моментов
    plt.figure(figsize=(12, 10))

    plt.subplot(2, 2, 1)
    plt.plot(time, q1)
    plt.xlabel('Время, с')
    plt.ylabel('q1, рад')
    plt.title('Обобщённая координата q1')
    plt.grid(True)

    plt.subplot(2, 2, 2)
    plt.plot(time, q2)
    plt.xlabel('Время, с')
    plt.ylabel('q2, рад')
    plt.title('Обобщённая координата q2')
    plt.grid(True)

    plt.subplot(2, 2, 3)
    plt.plot(time, tau1)
    plt.yscale('symlog', linthresh=1e-3)
    plt.xlabel('Время, с')
    plt.ylabel('τ1, Н·м (симлог)')
    plt.title('Момент в сочленении 1')
    plt.grid(True, which='both')

    plt.subplot(2, 2, 4)
    plt.plot(time, tau2)
    plt.yscale('symlog', linthresh=1e-3)
    plt.xlabel('Время, с')
    plt.ylabel('τ2, Н·м (симлог)')
    plt.title('Момент в сочленении 2')
    plt.grid(True, which='both')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()