#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as SensorImage
from rclpy.node import Node
from threading import Thread, Event
import cv2
from cv_bridge import CvBridge

class RobotControlNode(Node):
    def __init__(self, robot_name, image_callback):
        super().__init__('robot_control_node')
        self.publisher = self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        self.twist = Twist()
        self.image_callback = image_callback
        self.create_subscription(SensorImage, f'/{robot_name}/{robot_name}_camera/image_raw', self.image_callback, 10)

    def update_velocity(self, linear_x, angular_z):
        self.twist.linear.x = linear_x
        self.twist.angular.z = angular_z
        self.publisher.publish(self.twist)

class RobotControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("CADRE Control GUI")
        self.root.geometry("900x900")
        rclpy.init(args=None)
        self.robot_name = 'robot_1'
        self.node = RobotControlNode(self.robot_name, self.image_callback)
        self.bridge = CvBridge()
        self.shutdown_event = Event()
        
       
        self.robot_var = tk.StringVar(value=self.robot_name)
        # ttk.Label(root, text="Select Robot:").grid(row=0, column=0, padx=10, pady=10)
        ttk.Radiobutton(root, text="Robot 1", variable=self.robot_var, value='robot_1', command=self.select_robot).grid(row=0, column=0, padx=5, pady=5)
        ttk.Radiobutton(root, text="Robot 2", variable=self.robot_var, value='robot_2', command=self.select_robot).grid(row=0, column=1, padx=5, pady=5)
        ttk.Radiobutton(root, text="Robot 3", variable=self.robot_var, value='robot_3', command=self.select_robot).grid(row=0, column=2, padx=5, pady=5)
        
    
        self.canvas = tk.Canvas(root, width=300, height=300, bg='lightgrey')
        self.canvas.grid(row=1, column=0, columnspan=3, padx=10, pady=10)
        
        self.joystick_radius = 200
        self.joystick_center = (150, 150)
        
        self.canvas.create_oval(self.joystick_center[0]-self.joystick_radius, 
                                self.joystick_center[1]-self.joystick_radius, 
                                self.joystick_center[0]+self.joystick_radius, 
                                self.joystick_center[1]+self.joystick_radius, 
                                fill='grey', outline='black')

        self.joystick_marker = self.canvas.create_oval(self.joystick_center[0]-10, 
                                                        self.joystick_center[1]-10, 
                                                        self.joystick_center[0]+10, 
                                                        self.joystick_center[1]+10, 
                                                        fill='blue')

        self.stop_button = tk.Button(root, text="Stop", command=self.stop_robot, bg='red', fg='white')
        self.stop_button.grid(row=2, column=1, padx=10, pady=10)
        
        self.canvas.bind("<B1-Motion>", self.move_joystick)
        self.current_linear = 0
        self.current_angular = 0

    
        self.camera_label = tk.Label(root)
        self.camera_label.grid(row=1, column=3, padx=10, pady=10)
        self.image = None
        self.image_tk = None

        self.thread = Thread(target=self.update_ros, daemon=True)
        self.thread.start()

    def select_robot(self):
        new_robot_name = self.robot_var.get()
        if new_robot_name != self.robot_name:
            self.robot_name = new_robot_name
            if self.node:
                self.node.destroy_node()
            self.node = RobotControlNode(self.robot_name, self.image_callback)

    def stop_robot(self):
        
        self.node.update_velocity(0.0, 0.0)
      
        self.canvas.coords(self.joystick_marker, 
                           self.joystick_center[0]-10, 
                           self.joystick_center[1]-10, 
                           self.joystick_center[0]+10, 
                           self.joystick_center[1]+10)

    def move_joystick(self, event):
        x = event.x
        y = event.y
        dx = x - self.joystick_center[0]
        dy = y - self.joystick_center[1]
        distance = (dx**2 + dy**2)**0.5
        
        if distance > self.joystick_radius:
            dx = dx * self.joystick_radius / distance
            dy = dy * self.joystick_radius / distance
        
        new_x = self.joystick_center[0] + dx
        new_y = self.joystick_center[1] + dy
        
        self.canvas.coords(self.joystick_marker, 
                           new_x-10, new_y-10, 
                           new_x+10, new_y+10)
        
        linear_x = dy / self.joystick_radius
        angular_z = dx / self.joystick_radius
        
        self.current_linear = -angular_z*0.9
        self.current_angular = -linear_x*4.0
        
        if self.node:
            self.node.update_velocity(self.current_linear, self.current_angular)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        except Exception as e:
            print(f"Error converting image: {e}")

    def update_ros(self):
        while not self.shutdown_event.is_set():
            if self.node:
                try:
                    rclpy.spin_once(self.node, timeout_sec=0.01)
                except Exception as e:
                    print(f"Error spinning node: {e}")
            if self.image is not None:
                self.image_tk = ImageTk.PhotoImage(image=Image.fromarray(self.image))
                self.camera_label.config(image=self.image_tk)

    def run(self):
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def on_closing(self):
        self.shutdown_event.set()
        if self.node:
            self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControlApp(root)
    app.run()
