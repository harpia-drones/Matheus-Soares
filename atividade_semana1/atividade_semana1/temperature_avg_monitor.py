#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float64 #importando msg do tipo Float64

class TemperatureAvgMonitorNode(Node):
    def __init__(self):
        super().__init__("temperature_avg_monitor")
        self.subscriber = self.create_subscription(Float64, "average_temperature", self.callback_temperature_average_monitor, 10) #criando um subscriber do topico "average_temperature" do tipo Float64

    def callback_temperature_average_monitor(self, msg): #metodo para receber a publicacao recebida do topico "average_temperature"
        formatted_avg_temperature = f"{msg.data:.3f}"
        self.get_logger().info("Average temperature : " + str(formatted_avg_temperature) + " degrees Celsius")
    
def main(args=None):
    rclpy.init(args=args)
    node = TemperatureAvgMonitorNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()