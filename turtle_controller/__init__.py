#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_custom_interfaces.srv import MoveTurtle
from my_custom_interfaces.msg import Coordinates2D
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from functools import partial

class ControllerNode(Node):

    inProgress = False

    __curr_turtle_pose = Pose()

    def __init__(self):
        super().__init__("Controller_node")
        self.get_logger().info("Turtle Controller node has started")
        self.service = self.create_service(MoveTurtle, "Move", self.__cb_move_turtle)
        self.publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 20)
        self.subscriber = self.create_subscription(Pose, "/turtle1/pose", self.__cb_sub_turtle_pose, 20)
        self.tolerance = 0.1
        self.__prev_pose = Pose()
    
    def __cb_sub_turtle_pose(self, pose: Pose):
        self.__curr_turtle_pose = pose

    def __cb_move_turtle(self, request, response):
        x_target = request.coordinates.x        
        y_target = request.coordinates.y

        self.get_logger().info(f"Request to move to x: {x_target}, y: {y_target}")

        # in case of redundant requests of the same position
        if self.__prev_pose.x == x_target and self.__prev_pose.y == y_target:
            response.success = False

            # this is the confirmatory message after this is the new message
            # that is why I reseted the self.__prev_pose variable
            print("redundant target !!")

            self.__prev_pose.x = 0.0
            self.__prev_pose.y = 0.0
            return response

        if not self.inProgress:
            self.__service_call_timer = self.create_timer(0.1, partial(self.move_turtle_to, x_target, y_target))
            self.move_turtle_to(x_target, y_target) # just to keep self.inProgress updated

        response.success = not self.inProgress

        return response

    def move_turtle_to(self, target_x, target_y):
        # Create a Twist message to control the turtle
        msg = Twist()

        # Calculate the distance to the target position
        distance = math.sqrt(
            (target_x - self.__curr_turtle_pose.x) ** 2 +
            (target_y - self.__curr_turtle_pose.y) ** 2
        )

        if distance > self.tolerance:
            self.inProgress = True

            # Calculate the angle to the target position
            angle_to_target = math.atan2(
                target_y - self.__curr_turtle_pose.y,
                target_x - self.__curr_turtle_pose.x
            )

            # Calculate the difference between the turtle's current heading and the target angle
            angle_diff = angle_to_target - self.__curr_turtle_pose.theta

            # Normalize the angle difference to the range [-pi, pi]
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            # Proportional controller for linear velocity
            linear_speed = 4.8 * distance
            if linear_speed > 7.5:
                linear_speed = 7.5  # Cap the linear speed

            # Proportional controller for angular velocity
            angular_speed = 4.0 * angle_diff
            #if angular_speed > 2.0:
            #    angular_speed = 2.0  # Cap the angular speed

            # Set the velocities
            msg.linear.x = linear_speed
            msg.angular.z = angular_speed

            # Log the distance and angle difference
            self.get_logger().info(f'Distance: {distance:.2f}, Angle difference: {angle_diff:.2f}')
        else:
            self.__prev_pose.x = target_x
            self.__prev_pose.y = target_y
            
            self.inProgress = False
            self.__service_call_timer.destroy()
            # Stop the turtle if within the target tolerance
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            # Log that the turtle has reached the target
            self.get_logger().info('Reached the target position!')

        # Publish the velocity command
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)  # To start ROS 2 communication

    node = ControllerNode()
    rclpy.spin(node)

    rclpy.shutdown()  # To end ROS 2 communication

if __name__ == '__main__':
    main()

