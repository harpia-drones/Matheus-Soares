#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from temperature_interfaces.msg import Temperature #importando uma custom msg do tipo Temperature
import math
import random #importanto bibliotecas para o calculo da temperatura

class TemperaturePublisherNode(Node):
    def __init__(self):
        super().__init__("temperature_publisher")
        self.publisher = self.create_publisher(Temperature, "temperature", 10) #criando um publisher para o topico "temperature" do tipo Float64
        self.timer_ = self.create_timer(0.5, self.publish_temperature) #criando um timer de 2Hz para o metodo publish_temperature
        self.get_logger().info("Temperature Publisher has been started!")

    def publish_temperature(self): #funcao para editar publicar a temperatura
        random_number = random.randint(80, 100)
        temperature = 25.0 + 5.0 * math.sin(random_number / 10.0) #calculo da temperatura
        msg = Temperature() #variavel msg do tipo Temperature
        msg.temperature = temperature #o dado dessa mensagem eh a temperatura
        self.publisher.publish(msg) #publicando a mensagem (temperatura)
 
def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()