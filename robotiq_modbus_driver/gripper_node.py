import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from .robotiq_modbus_driver import RobotiqGripperModbusDriver


class RobotiqModbus(Node):
    def __init__(self):
        super().__init__("robotiq_modbus")
        # TODO device_id should be a parameter
        self.gripper = RobotiqGripperModbusDriver(device_id='/dev/ttyUSB0')
        self.gripper.connect()
        self.gripper.reset()
        self.gripper.activate()

        self.open_srv = self.create_service(Trigger, '/open_gripper', self.open_callback)
        self.close_srv = self.create_service(Trigger, '/close_gripper', self.close_callback)

    def open_callback(self, request, response):
        self.gripper.open()
        response.success = True
        return response

    def close_callback(self, request, response):
        self.gripper.close()
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotiqModbus()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
