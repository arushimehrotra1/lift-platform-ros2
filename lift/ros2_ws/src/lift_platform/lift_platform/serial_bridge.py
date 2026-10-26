#!/usr/bin/env python3
"""ROS 2 <-> ESP32 serial bridge.

Subscribes : /lift/cmd         std_msgs/String   (STOP, UP, DOWN, GOTO 50, RESET, ...)
             /lift/set_height  std_msgs/Float32  (mm, same as 'GOTO <mm>')
Publishes  : /lift/joint_states sensor_msgs/JointState  (positions in metres, effort = drive)
             /lift/height       std_msgs/Float32        (mm, mean extension)
             /lift/status       std_msgs/String         (human readable state)
             /lift/device_log   std_msgs/String         (ack/error/info lines from the ESP32)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import JointState
import serial

from lift_platform.protocol import clean_command, parse_telemetry, status_text


class SerialBridge(Node):
    def __init__(self):
        super().__init__('lift_serial_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.ser = serial.Serial(port, baud, timeout=0)   # non-blocking
        self.buf = b''
        self.get_logger().info(f'Opened {port} @ {baud}')

        self.pub_js = self.create_publisher(JointState, '/lift/joint_states', 10)
        self.pub_h = self.create_publisher(Float32, '/lift/height', 10)
        self.pub_status = self.create_publisher(String, '/lift/status', 10)
        self.pub_log = self.create_publisher(String, '/lift/device_log', 10)
        self.create_subscription(String, '/lift/cmd', self.on_cmd, 10)
        self.create_subscription(Float32, '/lift/set_height', self.on_height, 10)
        self.create_timer(0.005, self.poll)               # 200 Hz serial poll

    # ---- ROS -> ESP32 ---------------------------------------------------------
    def send(self, text):
        c = clean_command(text)
        if c is None:
            self.get_logger().warn(f'Rejected command: {text!r}')
            return
        try:
            self.ser.write((c + '\n').encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def on_cmd(self, msg):
        self.send(msg.data)

    def on_height(self, msg):
        self.send(f'GOTO {msg.data:.1f}')

    # ---- ESP32 -> ROS ---------------------------------------------------------
    def poll(self):
        try:
            self.buf += self.ser.read(512)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read failed: {e}', throttle_duration_sec=2.0)
            return
        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            self.handle_line(line.decode(errors='replace').strip())

    def handle_line(self, line):
        if not line:
            return
        t = parse_telemetry(line)
        if t is not None:
            self.publish_state(t)
            return
        if line[:2] in ('A,', 'N,', 'E,', 'I,', 'S,'):
            self.pub_log.publish(String(data=line))
            if line[0] == 'E':
                self.get_logger().error(f'FAULT: {line[2:]}')
            elif line[0] == 'N':
                self.get_logger().warn(f'Refused: {line[2:]}')
            else:
                self.get_logger().info(line)

    def publish_state(self, t):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['left_actuator', 'right_actuator']
        js.position = [t['x_left'] / 1000.0, t['x_right'] / 1000.0]
        js.effort = [t['u_left'], t['u_right']]
        self.pub_js.publish(js)
        self.pub_h.publish(Float32(data=t['height']))
        self.pub_status.publish(String(data=status_text(t)))


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.write(b'STOP\n')
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
