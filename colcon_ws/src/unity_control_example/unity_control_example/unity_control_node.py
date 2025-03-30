#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
import time
import threading

class TerminalInput:
    """
    使用 termios, tty, select 检测终端按键输入。
    按键检测采用后台线程不断读取字符，
    当检测到 'w', 's', 'a' 或 'd' 时，将对应状态设置为 True，
    否则状态为 False（每次轮询时重置）。
    """
    def __init__(self):
        self.key_state = {'w': False, 's': False, 'a': False, 'd': False, 'q': False, 'e': False}
        self.settings = termios.tcgetattr(sys.stdin)
        # 设置终端为非阻塞原始模式
        tty.setcbreak(sys.stdin.fileno())
        self.running = True
        self.thread = threading.Thread(target=self._key_loop)
        self.thread.daemon = True
        self.thread.start()

    def _key_loop(self):
        while self.running:
            # 超时设置为 0.01 秒，非阻塞检测
            dr, _, _ = select.select([sys.stdin], [], [], 0.01)
            # 重置按键状态
            self.key_state = {'w': False, 's': False, 'a': False, 'd': False, 'q': False, 'e': False}
            if dr:
                ch = sys.stdin.read(1)
                if ch in self.key_state:
                    self.key_state[ch] = True

    def get_key_state(self):
        return self.key_state.copy()

    def stop(self):
        self.running = False
        self.thread.join()
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

class ROS2TerminalControlPublisher(Node):
    """
    ROS2 节点，使用终端检测的按键：
      - 'w': 前进（正向线速度）
      - 's': 后退（负向线速度）
      - 'a': 左转（正向角速度）
      - 'd': 右转（负向角速度）
    按下时逐渐由 0 加速到 1（或 -1），未按时立即归零。
    """
    def __init__(self):
        super().__init__('terminal_control_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.05  # 50Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.last_time = time.time()

        self.terminal_input = TerminalInput()

    def timer_callback(self):
        """"""
        # current_time = time.time()
        # dt = current_time - self.last_time
        # self.last_time = current_time

        # key_state = self.terminal_input.get_key_state()

        # # 线速度：'w' 加速到 1.0，'s' 加速到 -1.0，其他情况归零
        # if key_state.get('w'):
        #     self.linear_speed = min(self.linear_speed + 0.05, 1.0)
        # elif key_state.get('s'):
        #     self.linear_speed = max(self.linear_speed - 0.05, -1.0)
        # elif not key_state.get('w') or not key_state.get('s'):
        #     self.linear_speed = 0.0
        # # else:
        # #     self.linear_speed = 0.0

        # # 角速度：'a' 加速到 1.0，'d' 加速到 -1.0，其他情况归零
        # if key_state.get('a'):
        #     self.angular_speed = max(self.angular_speed - 0.05, -1.0)
        # elif key_state.get('d'):
        #     self.angular_speed =  min(self.angular_speed + 0.05, 1.0)
        # elif not key_state.get('a') or not key_state.get('d'):
        #     self.angular_speed = 0.0
        # # else:
        # #     self.angular_speed = 0.0

        # twist = Twist()
        # twist.linear.x = self.linear_speed
        # twist.angular.z = self.angular_speed
        # self.publisher_.publish(twist)

    def destroy_node(self):
        self.terminal_input.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ROS2TerminalControlPublisher()
    try:
        while rclpy.ok():
            key_state = node.terminal_input.get_key_state()

            # 线速度：'w' 加速到 1.0，'s' 加速到 -1.0，其他情况归零
            if key_state.get('w'):
                node.linear_speed = min(node.linear_speed + 0.05, 1.0)
            elif key_state.get('s'):
                node.linear_speed = max(node.linear_speed - 0.05, -1.0)
            # elif not key_state.get('w') or not key_state.get('s'):
            #     node.linear_speed = 0.0
            else:
                node.linear_speed = 0.0

            # 角速度：'a' 加速到 1.0，'d' 加速到 -1.0，其他情况归零
            if key_state.get('a'):
                node.angular_speed = max(node.angular_speed - 0.01, -1.0)
            elif key_state.get('d'):
                node.angular_speed =  min(node.angular_speed + 0.01, 1.0)
            else:
                if node.angular_speed > 0:
                    node.angular_speed = max(node.angular_speed - 0.01, 0.0)
                else:
                    node.angular_speed =  min(node.angular_speed + 0.01, 0.0)

            twist = Twist()
            twist.linear.x = node.linear_speed
            twist.angular.z = node.angular_speed
            node.publisher_.publish(twist)

            print(f"twist.linear.x: {twist.linear.x}, twist.angular.z: {twist.angular.z}")

    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
