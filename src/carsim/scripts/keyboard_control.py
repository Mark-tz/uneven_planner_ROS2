#!/usr/bin/env python3

import atexit
import os
import signal
from threading import Lock
from tkinter import Frame, Label, Tk
import time
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

UP = "w"
LEFT = "a"
DOWN = "s"
RIGHT = "d"
QUIT = "q"

state = [False, False, False, False]
state_lock = Lock()
node = None
root = None
control = False


def keyeq(e, c):
    return e.char == c or e.keysym == c


def keyup(e):
    global state
    global control

    with state_lock:
        if keyeq(e, UP):
            state[0] = False
        elif keyeq(e, LEFT):
            state[1] = False
        elif keyeq(e, DOWN):
            state[2] = False
        elif keyeq(e, RIGHT):
            state[3] = False
        control = sum(state) > 0


def keydown(e):
    global state
    global control

    with state_lock:
        if keyeq(e, QUIT):
            shutdown()
        elif keyeq(e, UP):
            state[0] = True
            state[2] = False
        elif keyeq(e, LEFT):
            state[1] = True
            state[3] = False
        elif keyeq(e, DOWN):
            state[2] = True
            state[0] = False
        elif keyeq(e, RIGHT):
            state[3] = True
            state[1] = False
        control = sum(state) > 0

def exit_func():
    os.system("xset r on")


def shutdown():
    root.destroy()
    node.destroy_node()
    rclpy.shutdown()


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.state_pub = self.create_publisher(Twist, "/racebot/cmd_vel", 1)
        self.command = Twist()
        self.max_velocity = 1
        self.max_steering_angle = 0.5
        self.timer = self.create_timer(0.05, self.publish_cb)

    def publish_cb(self):
        global state
        global state_lock
        
        with state_lock:
            if state[0]:  # UP
                self.command.linear.x = self.max_velocity
            elif state[2]:  # DOWN
                self.command.linear.x = -self.max_velocity
            else:
                self.command.linear.x = 0.0

            if state[1]:  # LEFT
                self.command.angular.z = self.max_steering_angle
            elif state[3]:  # RIGHT
                self.command.angular.z = -self.max_steering_angle
            else:
                self.command.angular.z = 0.0

        self.state_pub.publish(self.command)


def main():
    global root
    global node
    
    rclpy.init()
    node = KeyboardControlNode()
    
    atexit.register(exit_func)
    os.system("xset r off")

    root = Tk()
    frame = Frame(root, width=100, height=100)
    frame.bind("<KeyPress>", keydown)
    frame.bind("<KeyRelease>", keyup)
    frame.pack()
    frame.focus_set()
    lab = Label(
        frame,
        height=10,
        width=30,
        text="Focus on this window\nand use the WASD keys\nto drive the car.\n\nPress Q to quit",
    )
    lab.pack()
    print("Press %c to quit" % QUIT)
    root.mainloop()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, lambda s, f: shutdown())
    time.sleep(5) #sleep for 5 seconds to wait for spawning model
    main()
