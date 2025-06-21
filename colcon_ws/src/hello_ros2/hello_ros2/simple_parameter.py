# ros2 run hello_ros2 simple_parameter --ros-args -p use_sim_time:=True

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter


class Simple_parameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")
        self.create_timer(1, self.update)
        self.declare_parameter("para1", 0)
        self.para1 = self.get_parameter("para1").get_parameter_value().integer_value

        self.add_on_set_parameters_callback(self.parameter_callback)

    def update(self):
        self.get_logger().info(f"parameter : {self.para1}")
        self.para1 += 1
        self.set_parameters([Parameter("para1", Parameter.Type.INTEGER, self.para1)])

    def parameter_callback(self, parameters: list[Parameter]):
        for parameter in parameters:
            if parameter.name == "para1":
                self.para1 = int(parameter.value)  # type: ignore
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = Simple_parameter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()