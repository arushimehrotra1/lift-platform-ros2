#!/usr/bin/env python3
"""Keyboard control for the lifting platform. Run in its own terminal.

Raise/lower are LATCHED (press once, press SPACE to stop) and re-sent at 10 Hz,
so if this program or the bridge dies the firmware stops the motors by itself
after 0.4 s. Closed-loop height moves use GOTO.
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

HELP = """
 Lift platform keyboard control
 ------------------------------------------------------------
  w / UP      raise (latched)          space / x   STOP
  s / DOWN    lower (latched)          r           RESET fault
  g           type a height in mm, Enter  -> closed-loop GOTO
  1..5        GOTO 0 / 25 / 50 / 75 / 100 % of stroke
  [  /  ]     calibrate: [ = fully RETRACTED, ] = fully EXTENDED
  j / k       recovery jog LEFT up/down      n / m   jog RIGHT up/down
  ?           device settings                q       quit
 ------------------------------------------------------------
"""
PRESETS = {'1': 0, '2': 25, '3': 50, '4': 75, '5': 100}


class Teleop(Node):
    def __init__(self):
        super().__init__('lift_keyboard_teleop')
        self.declare_parameter('stroke_mm', 100.0)
        self.stroke = self.get_parameter('stroke_mm').value
        self.pub = self.create_publisher(String, '/lift/cmd', 10)
        self.create_subscription(String, '/lift/status', self.on_status, 10)
        self.create_subscription(String, '/lift/device_log', self.on_log, 10)
        self.status = '(no telemetry yet)'
        self.log = ''
        self.hold = None            # 'UP' / 'DOWN' / ('JOG', 'L', '1') ...
        self.typing = None          # digits being typed for 'g'
        self.create_timer(0.1, self.tick)

    def send(self, s):
        self.pub.publish(String(data=s))

    def on_status(self, m):
        self.status = m.data

    def on_log(self, m):
        self.log = m.data

    def tick(self):                 # 10 Hz hold-to-move refresh
        if self.hold:
            self.send(self.hold)
        line = f'\r\x1b[K{self.status}'
        if self.typing is not None:
            line += f'   height> {self.typing}'
        elif self.log:
            line += f'   [{self.log}]'
        sys.stdout.write(line)
        sys.stdout.flush()

    def key(self, k):
        if self.typing is not None:                       # entering a number
            if k in '0123456789.':
                self.typing += k
            elif k in ('\r', '\n'):
                if self.typing:
                    self.hold = None
                    self.send(f'GOTO {self.typing}')
                self.typing = None
            elif k in ('\x7f', '\b'):
                self.typing = self.typing[:-1]
            elif k == '\x1b':
                self.typing = None
            return True
        if k in ('w', 'UP'):
            self.hold = 'UP'
        elif k in ('s', 'DOWN'):
            self.hold = 'DOWN'
        elif k in (' ', 'x'):
            self.hold = None
            self.send('STOP')
        elif k == 'r':
            self.hold = None
            self.send('RESET')
        elif k == 'g':
            self.hold = None
            self.typing = ''
        elif k in PRESETS:
            self.hold = None
            self.send(f'GOTO {self.stroke * PRESETS[k] / 100.0:.1f}')
        elif k == '[':
            self.send('CALMIN')
        elif k == ']':
            self.send('CALMAX')
        elif k in ('j', 'k'):
            self.hold = f"JOG L {'1' if k == 'j' else '-1'}"
        elif k in ('n', 'm'):
            self.hold = f"JOG R {'1' if k == 'n' else '-1'}"
        elif k == '?':
            self.send('GET')
        elif k in ('q', '\x03'):
            self.hold = None
            self.send('STOP')
            return False
        return True


def read_key():
    """Non-blocking single key; decodes arrow keys."""
    if not select.select([sys.stdin], [], [], 0)[0]:
        return None
    ch = sys.stdin.read(1)
    if ch == '\x1b' and select.select([sys.stdin], [], [], 0.01)[0]:
        seq = sys.stdin.read(2)
        return {'[A': 'UP', '[B': 'DOWN'}.get(seq, '')
    return ch


def main():
    rclpy.init()
    node = Teleop()
    if not sys.stdin.isatty():
        print('keyboard_teleop needs an interactive terminal')
        return
    old = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        running = True
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            k = read_key()
            if k:
                running = node.key(k)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.send('STOP')
        print('\nstopped.')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
