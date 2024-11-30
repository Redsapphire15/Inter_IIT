#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
        i
   j    k    l

i: move forward
k: move backward
j: turn left
l: turn right
s: stop

CTRL-C to quit
"""

moveBindings = {
    'i': (0.8, 0.0),
    ',': (-0.8, 0.0),
    'j': (0.0, 0.4),
    'l': (0.0, -0.4),
}

stopBindings = {'k': (0.0, 0.0)}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, 'robot1_1/cmd_vel', 10)
        self.lin_vel = 0.0
        self.ang_vel = 0.0

    def send_velocity(self):
        twist = Twist()
        twist.linear.x = self.lin_vel
        twist.angular.z = self.ang_vel
        self.publisher.publish(twist)

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = Teleop()

    try:
        print(msg)
        while True:
            key = getKey()
            if key in moveBindings.keys():
                node.lin_vel = moveBindings[key][0]
                node.ang_vel = moveBindings[key][1]
            elif key in stopBindings.keys():
                node.lin_vel = stopBindings[key][0]
                node.ang_vel = stopBindings[key][1]
            else:
                if (key == '\x03'):
                    break

            node.send_velocity()

    except Exception as e:
        print(e)

    finally:
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
