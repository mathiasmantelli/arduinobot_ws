#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import EulerToQuaternion, QuaternionToEuler
from tf_transformations import quaternion_from_euler, euler_from_quaternion


class AnglesConverter(Node):
    def __init__(self):
        super().__init__("angle_conversion_service_server")
        self.euler_to_quaternion_ = self.create_service(EulerToQuaternion, "euler_to_quaternion", self.eulerToQuaternionCallback)
        self.quaternion_to_euler_ = self.create_service(QuaternionToEuler, "quaternion_to_euler", self.quaternionToEulerCallback)
        self.get_logger().info("Angle conversion services are ready")

    def eulerToQuaternionCallback(self, request, response):
        self.get_logger().info("Requested to convert euler angles: Roll: %f | Pitch: %f | Yaw: %f" %(request.roll, request.pitch, request.yaw))
        (response.x, response.y, response.z, response.w) = quaternion_from_euler(request.roll, request.pitch, request.yaw)
        self.get_logger().info("Corresponding quaternions: X: %f | Y: %f | Z: %f | W: %f" %(response.x, response.y, response.z, response.w))
        return response
        
    def quaternionToEulerCallback(self, request, response):
        self.get_logger().info("Requested to convert Quaterion. X: %f | Y: %f | Z: %f | W: %f" %(request.x, request.y, request.z, request.w))
        (response.roll, response.pitch, response.yaw) = euler_from_quaternion([request.x, request.y, request.z, request.w])
        self.get_logger().info("Corresponding euler angles: Roll: %f | Pitch: %f | Yaw: %f" %(response.roll, response.pitch, response.yaw))
        return response


def main():
    rclpy.init()
    angles_converter = AnglesConverter()
    rclpy.spin(angles_converter)
    angles_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()