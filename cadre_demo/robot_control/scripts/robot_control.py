#!/usr/bin/env python3
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import tkinter as tk  # Use Tkinter for GUI

class Commander(Node):

    def __init__(self, robot_name):
        super().__init__('commander_' + robot_name)
        timer_period = 0.002

        self.robot_name = robot_name
        self.wheel_seperation = 0.41605
        self.wheel_base = 0.425
        self.wheel_radius = 0.099

        # Initialize wheel velocities
        self.vel = np.array([0, 0, 0, 0], float)  # left_front, right_front, left_rear, right_rear

        # Initialize velocity message and mode selection
        self.vel_msg = Twist()
        self.mode_selection = 0

        # Publishers
        self.pub_vel = self.create_publisher(Float64MultiArray, f'{self.robot_name}/forward_velocity_controller/commands', 100)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Default to zero velocities
        self.vel[:] = 0

        if self.mode_selection == 1:  # Forward/Reverse
            self.vel[0] = -self.vel_msg.linear.x
            self.vel[1] = -self.vel_msg.linear.x
            self.vel[2] = self.vel_msg.linear.x
            self.vel[3] = self.vel_msg.linear.x
        elif self.mode_selection == 2:  # Rotate Left
            self.vel[0] = -self.vel_msg.angular.z*0
            self.vel[1] = -self.vel_msg.angular.z*0
            self.vel[2] = -self.vel_msg.angular.z*1
            self.vel[3] = -self.vel_msg.angular.z*1
        elif self.mode_selection == 3:  # Rotate Right
            self.vel[0] = self.vel_msg.angular.z*0
            self.vel[1] = self.vel_msg.angular.z*0
            self.vel[2] = self.vel_msg.angular.z*1
            self.vel[3] = self.vel_msg.angular.z*1
        elif self.mode_selection == 4:  # Stop
            self.vel[:] = 0

        # Publish the velocity command
        vel_array = Float64MultiArray(data=self.vel)
        self.pub_vel.publish(vel_array)

def send_command(robot_commander, command, speed):
    if command == "Forward":
        robot_commander.vel_msg.linear.x = float(speed)
        robot_commander.mode_selection = 1
    elif command == "Reverse":
        robot_commander.vel_msg.linear.x = float(-speed)
        robot_commander.mode_selection = 1
    elif command == "Left":
        robot_commander.vel_msg.angular.z = float(speed)
        robot_commander.mode_selection = 2
    elif command == "Right":
        robot_commander.vel_msg.angular.z = float(speed)
        robot_commander.mode_selection = 3
    elif command == "Stop":
        robot_commander.vel_msg.linear.x = float(0)
        robot_commander.vel_msg.angular.z = float(0)
        robot_commander.mode_selection = 4

    # Trigger the timer callback to publish the updated velocities
    robot_commander.timer_callback()

def create_gui():
    root = tk.Tk()
    root.title("Robot Control GUI")

    # Speed Control
    speed_scale = tk.Scale(root, from_=0, to=10, orient=tk.HORIZONTAL, label="Speed")
    speed_scale.pack()

    # Robot 1 Control
    tk.Label(root, text="Robot 1 Control").pack()
    tk.Button(root, text="Forward", command=lambda: send_command(commander1, "Forward", speed_scale.get())).pack()
    tk.Button(root, text="Reverse", command=lambda: send_command(commander1, "Reverse", speed_scale.get())).pack()
    tk.Button(root, text="Left", command=lambda: send_command(commander1, "Left", speed_scale.get())).pack()
    tk.Button(root, text="Right", command=lambda: send_command(commander1, "Right", speed_scale.get())).pack()
    tk.Button(root, text="Stop", command=lambda: send_command(commander1, "Stop", speed_scale.get())).pack()

    # Robot 2 Control
    tk.Label(root, text="Robot 2 Control").pack()
    tk.Button(root, text="Forward", command=lambda: send_command(commander2, "Forward", speed_scale.get())).pack()
    tk.Button(root, text="Reverse", command=lambda: send_command(commander2, "Reverse", speed_scale.get())).pack()
    tk.Button(root, text="Left", command=lambda: send_command(commander2, "Left", speed_scale.get())).pack()
    tk.Button(root, text="Right", command=lambda: send_command(commander2, "Right", speed_scale.get())).pack()
    tk.Button(root, text="Stop", command=lambda: send_command(commander2, "Stop", speed_scale.get())).pack()

    # Robot 3 Control
    tk.Label(root, text="Robot 3 Control").pack()
    tk.Button(root, text="Forward", command=lambda: send_command(commander3, "Forward", speed_scale.get())).pack()
    tk.Button(root, text="Reverse", command=lambda: send_command(commander3, "Reverse", speed_scale.get())).pack()
    tk.Button(root, text="Left", command=lambda: send_command(commander3, "Left", speed_scale.get())).pack()
    tk.Button(root, text="Right", command=lambda: send_command(commander3, "Right", speed_scale.get())).pack()
    tk.Button(root, text="Stop", command=lambda: send_command(commander3, "Stop", speed_scale.get())).pack()

    root.mainloop()

def main():
    rclpy.init(args=None)

    global commander1, commander2, commander3
    commander1 = Commander("robot_1")
    commander2 = Commander("robot_2")
    commander3 = Commander("robot_3")

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander1)
    executor.add_node(commander2)
    executor.add_node(commander3)

    # Start the executor in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        create_gui()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main()
