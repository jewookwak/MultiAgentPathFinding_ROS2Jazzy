# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseStamped
# import math
# import threading
# import sys
# import numpy as np

# from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
# from PyQt5.QtCore import QTimer
# from matplotlib.figure import Figure
# from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

# NUM_ROBOTS = 3  # 필요에 따라 로봇 수 조정

# # ✅ 직접 정의한 맵 (1 = 벽, 0 = 자유공간)
# custom_map = [
#     [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
#     [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
#     [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
#     [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
#     [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
#     [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
#     [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
#     [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
#     [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
#     [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
#     [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
#     [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
# ]

# class RobotMonitor(Node):
#     def __init__(self):
#         super().__init__('multi_robot_monitor')
#         self.robot_poses = {}  # {id: (x, y, theta)}
#         self.robot_goals = {}  # {id: (x, y)}

#         for i in range(1, NUM_ROBOTS + 1):
#             self.create_subscription(Odometry, f'/odom_{i}', self.make_odom_cb(i), 10)
#             self.create_subscription(PoseStamped, f'/goalpose{i}', self.make_goal_cb(i), 10)

#     def make_odom_cb(self, robot_id):
#         def callback(msg):
#             pose = msg.pose.pose
#             self.robot_poses[robot_id] = (
#                 pose.position.x, pose.position.y, self.yaw_from_quat(pose.orientation)
#             )
#         return callback

#     def make_goal_cb(self, robot_id):
#         def callback(msg):
#             pose = msg.pose
#             self.robot_goals[robot_id] = (pose.position.x, pose.position.y)
#         return callback

#     def yaw_from_quat(self, q):
#         siny_cosp = 2 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
#         return math.atan2(siny_cosp, cosy_cosp)

# def ros_spin(node):
#     rclpy.spin(node)

# class MainWindow(QMainWindow):
#     def __init__(self, node):
#         super().__init__()
#         self.setWindowTitle("Multi-Robot Monitor with Custom Map")
#         self.node = node

#         # ✅ 창 크기 1.5배 확대
#         self.resize(1500, 750)

#         # ✅ 맵을 numpy로 변환
#         self.map_array = np.array(custom_map, dtype=np.uint8)
#         self.map_array = (1 - self.map_array) * 255  # 0 → 255 (white), 1 → 0 (black)
#         self.map_array = np.flipud(self.map_array)   # y축 상하반전

#         self.map_resolution = 2.0 / 22  # 2.0m / 22 cells → 0.0909m/cell (임의로 지정)

#         # UI
#         main_widget = QWidget()
#         layout = QVBoxLayout()
#         self.figure = Figure()
#         self.canvas = FigureCanvas(self.figure)
#         layout.addWidget(self.canvas)
#         main_widget.setLayout(layout)
#         self.setCentralWidget(main_widget)

#         self.ax = self.figure.add_subplot(111)
#         self.timer = QTimer()
#         self.timer.timeout.connect(self.update_plot)
#         self.timer.start(100)

#     def update_plot(self):
#         self.ax.clear()

#         # ✅ 맵 출력: 범위 (-0.1, -0.1) ~ (2.1, 1.1)
#         extent = [-0.1, 2.1, -0.1, 1.1]
#         self.ax.imshow(self.map_array, cmap='gray', origin='lower', extent=extent)

#         # ✅ 로봇 위치와 목표 위치 표시
#         for rid in range(1, NUM_ROBOTS + 1):
#             if rid in self.node.robot_poses:
#                 x, y, theta = self.node.robot_poses[rid]
#                 dx = 0.05 * math.cos(theta)
#                 dy = 0.05 * math.sin(theta)
#                 self.ax.arrow(x, y, dx, dy, head_width=0.02, color=f'C{rid}')
#                 self.ax.plot(x, y, 'o', color=f'C{rid}')

#             if rid in self.node.robot_goals:
#                 gx, gy = self.node.robot_goals[rid]
#                 self.ax.plot(gx, gy, 'X', color=f'C{rid}', markersize=6)

#         # ✅ 눈금, 범위 설정
#         self.ax.set_xlim(-0.1, 2.1)
#         self.ax.set_ylim(-0.1, 1.1)
#         self.ax.set_xticks(np.arange(-0.1, 2.11, 0.1))
#         self.ax.set_yticks(np.arange(-0.1, 1.11, 0.1))

#         self.ax.set_title("Robot Positions on Custom Map")
#         self.ax.set_xlabel("X (m)")
#         self.ax.set_ylabel("Y (m)")
#         self.ax.grid(True)
#         self.canvas.draw()

# def main(args=None):
#     rclpy.init(args=args)
#     node = RobotMonitor()

#     spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
#     spin_thread.start()

#     app = QApplication(sys.argv)
#     window = MainWindow(node)
#     window.show()
#     ret = app.exec_()

#     node.destroy_node()
#     rclpy.shutdown()
#     sys.exit(ret)

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math
import threading
import sys
import numpy as np

from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

NUM_ROBOTS = 3  # 필요에 따라 로봇 수 조정

# ✅ 직접 정의한 맵 (1 = 벽, 0 = 자유공간)
custom_map = [
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
]

class RobotMonitor(Node):
    def __init__(self):
        super().__init__('multi_robot_monitor')
        self.robot_poses = {}  # {id: (x, y, theta)}
        self.robot_goals = {}  # {id: (x, y)}
        self.robot_paths = {}  # {id: [(x1, y1), (x2, y2), ...]}

        for i in range(1, NUM_ROBOTS + 1):
            # 기존 구독자들
            self.create_subscription(Odometry, f'/odom_{i}', self.make_odom_cb(i), 10)
            self.create_subscription(PoseStamped, f'/goalpose{i}', self.make_goal_cb(i), 10)
            
            # ✅ 경로 구독자 추가
            self.create_subscription(Path, f'/path{i}', self.make_path_cb(i), 10)

    def make_odom_cb(self, robot_id):
        def callback(msg):
            pose = msg.pose.pose
            self.robot_poses[robot_id] = (
                pose.position.x, pose.position.y, self.yaw_from_quat(pose.orientation)
            )
        return callback

    def make_goal_cb(self, robot_id):
        def callback(msg):
            pose = msg.pose
            self.robot_goals[robot_id] = (pose.position.x, pose.position.y)
        return callback

    def make_path_cb(self, robot_id):
        """✅ 경로 콜백 함수"""
        def callback(msg):
            path_points = []
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                path_points.append((x, y))
            self.robot_paths[robot_id] = path_points
            self.get_logger().info(f'🛤️  로봇 {robot_id} 경로 수신: {len(path_points)} 점')
        return callback

    def yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def ros_spin(node):
    rclpy.spin(node)

class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.setWindowTitle("Multi-Robot Monitor with Custom Map & Paths")
        self.node = node

        # ✅ 창 크기 1.5배 확대
        self.resize(1500, 750)

        # ✅ 맵을 numpy로 변환
        self.map_array = np.array(custom_map, dtype=np.uint8)
        self.map_array = (1 - self.map_array) * 255  # 0 → 255 (white), 1 → 0 (black)
        self.map_array = np.flipud(self.map_array)   # y축 상하반전

        self.map_resolution = 2.0 / 22  # 2.0m / 22 cells → 0.0909m/cell (임의로 지정)

        # UI
        main_widget = QWidget()
        layout = QVBoxLayout()
        self.figure = Figure()
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)
        main_widget.setLayout(layout)
        self.setCentralWidget(main_widget)

        self.ax = self.figure.add_subplot(111)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

    def update_plot(self):
        self.ax.clear()

        # ✅ 맵 출력: 범위 (-0.1, -0.1) ~ (2.1, 1.1)
        extent = [-0.1, 2.1, -0.1, 1.1]
        self.ax.imshow(self.map_array, cmap='gray', origin='lower', extent=extent)

        # ✅ 각 로봇의 경로를 30% 투명도로 표시
        for rid in range(1, NUM_ROBOTS + 1):
            if rid in self.node.robot_paths and len(self.node.robot_paths[rid]) > 1:
                path_points = self.node.robot_paths[rid]
                x_coords = [point[0] for point in path_points]
                y_coords = [point[1] for point in path_points]
                
                # 경로를 선으로 연결하여 표시 (30% 투명도)
                self.ax.plot(x_coords, y_coords, '-', 
                           color=f'C{rid}', linewidth=2, alpha=0.3, 
                           label=f'Robot {rid} Path')
                
                # 경로 시작점과 끝점 표시
                if len(path_points) > 0:
                    # 시작점 (원)
                    self.ax.plot(x_coords[0], y_coords[0], 'o', 
                               color=f'C{rid}', markersize=8, alpha=0.7)
                    # 끝점 (사각형)
                    self.ax.plot(x_coords[-1], y_coords[-1], 's', 
                               color=f'C{rid}', markersize=8, alpha=0.7)

        # ✅ 로봇 현재 위치와 목표 위치 표시
        for rid in range(1, NUM_ROBOTS + 1):
            if rid in self.node.robot_poses:
                x, y, theta = self.node.robot_poses[rid]
                dx = 0.05 * math.cos(theta)
                dy = 0.05 * math.sin(theta)
                # 로봇 현재 위치 (화살표)
                self.ax.arrow(x, y, dx, dy, head_width=0.02, 
                            color=f'C{rid}', alpha=1.0, linewidth=2)
                # 로봇 현재 위치 (점)
                self.ax.plot(x, y, 'o', color=f'C{rid}', markersize=10, 
                           markeredgecolor='black', markeredgewidth=1)

            if rid in self.node.robot_goals:
                gx, gy = self.node.robot_goals[rid]
                # 현재 목표점 (X 표시)
                self.ax.plot(gx, gy, 'X', color=f'C{rid}', markersize=8, 
                           markeredgecolor='black', markeredgewidth=1)

        # ✅ 눈금, 범위 설정
        self.ax.set_xlim(-0.1, 2.1)
        self.ax.set_ylim(-0.1, 1.1)
        self.ax.set_xticks(np.arange(-0.1, 2.11, 0.1))
        self.ax.set_yticks(np.arange(-0.1, 1.11, 0.1))

        self.ax.set_title("Multi-Robot Monitor with Paths")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True, alpha=0.3)
        
        # 범례 추가 (경로가 있는 로봇만)
        handles, labels = self.ax.get_legend_handles_labels()
        if handles:
            self.ax.legend(loc='upper right', fontsize=8)
        
        self.canvas.draw()

def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()

    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()
    ret = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()