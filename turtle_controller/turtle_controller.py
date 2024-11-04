#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class ControllerNode(Node):

    

    def __init__(self):
        super().__init__("Controller_node")
        self.get_logger().info("Turtle Controller node has started")
        self.publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 20)
        self.subscriber = self.create_subscription(Pose, "/turtle1/pose", self.__cb_sub_turtle_pose, 20)
        self.rec_turtle_pose = self.create_subscription(Pose, "/turtle2/pose", self.__cb_controller_turtle_pose, 20)
        self.create_timer(0.1, self.move_turtle_to_2)
        self.tolerance = 0.1
        self.p_angle = 3.2
        self.p_linear = 1.5
        self.max_linear_speed = 3.0
        self.target = Pose()
        self.__curr_turtle_pose = Pose()

    def __cb_controller_turtle_pose(self, msg:Pose):
        self.target.x = msg.x
        self.target.y = msg.y

    def __cb_sub_turtle_pose(self, pose: Pose):
        self.__curr_turtle_pose = pose

    def move_turtle_to_2(self):
        # Create a Twist message to control the turtle
        msg = Twist()

        target_x = self.target.x
        target_y = self.target.y

        # Calculate the distance to the target position
        distance = math.sqrt(
            (target_x - self.__curr_turtle_pose.x) ** 2 +
            (target_y - self.__curr_turtle_pose.y) ** 2
        )

        target_theta = math.atan2((target_y - self.__curr_turtle_pose.y),
                                  (target_x - self.__curr_turtle_pose.x))

        angle_error = target_theta - self.__curr_turtle_pose.theta
        
        # Normalize the angle difference to the range [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if math.fabs(angle_error) > self.tolerance or distance > self.tolerance:
            msg.angular.z = angle_error * self.p_angle
            self.get_logger().info(f"the angle error = {angle_error}")

            linear_speed = distance * self.p_linear

            if linear_speed > self.max_linear_speed:
                linear_speed = self.max_linear_speed  # Cap the linear speed

            msg.linear.x = linear_speed

        else:
            self.get_logger().info(f"the turtle has arrived = {angle_error}, {distance}")            

        # Publish the velocity command
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)  # To start ROS 2 communication

    node = ControllerNode()
    rclpy.spin(node)

    rclpy.shutdown()  # To end ROS 2 communication

if __name__ == '__main__':
    main()

