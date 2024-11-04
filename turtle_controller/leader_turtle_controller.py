#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist
import math
from turtlesim.msg import Pose

class LeaderController(Node):
    def __init__(self):
        super().__init__("Leader_Controller")
        self.declare_parameter("port", value="/dev/ttyACM0")
        self.declare_parameter("baudrate", value=115200)
        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.arduino = serial.Serial(port=self.port, baudrate=self.baudrate)
        self.pub_turtle_2 = self.create_publisher(Twist, "/turtle2/cmd_vel", 20)
        self.timer = self.create_timer(0.0001, self.read_arduino)
        self.leader_pose_update = self.create_subscription(Pose, "/turtle2/pose", self.cb_update_pose, 20)
        self.leader_pose = Pose()
        self.tolerance = 0.3
        self.p_angle_proportional = 6.5

    def cb_update_pose(self, msg:Pose):
        self.leader_pose = msg

    def read_arduino(self):
        if rclpy.ok and self.arduino.is_open:
            self.rec_str = self.arduino.readline()
            x = ""
            y = ""
            # try catch block is very important as the value is not ready at first
            try:
                # extract the x, coordinates from the sent string
                i = 1
                while chr(self.rec_str[i]) != ',':
                    x += chr(self.rec_str[i])
                    i+=1
                i+=1
                while chr(self.rec_str[i]) != ')':
                    y += chr(self.rec_str[i])
                    i+=1

                # convert the sent velocity commands into optimal velocities
                x = float(x)/100.0
                y = float(y)/100.0

                message = Twist()
                magnitude = math.sqrt(x**2 + y**2)
                # angle w.r.to the reference frame
                angle = math.atan2(y, x)

                if magnitude > self.tolerance:
                    # calculate the error
                    angle_error = angle - self.leader_pose.theta
                    # Normalize the angle difference to the range [-pi, pi]
                    angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
                else:
                    # if the joystick is released send stop command
                    magnitude = 0.0
                    angle_error = 0.0

                message.linear.x = magnitude
                message.angular.z = angle_error * self.p_angle_proportional

                self.pub_turtle_2.publish(msg=message)

            except: 
                pass


def main(args=None):
    rclpy.init(args=args) # to start ros2 communication

    node = LeaderController()
    rclpy.spin(node)

    rclpy.shutdown() # to end ros2 communication

