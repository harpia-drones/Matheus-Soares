#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float64 #importando msg do tipo Float64
from example_interfaces.srv import SetBool #importanto um servico do tipo SetBool

class TemperatureMonitorNode(Node):
    def __init__(self):
        super().__init__("temperature_monitor")
        self.temperature_readings = [] #lista para armazenar as leituras das temperaturas
        self.subscriber = self.create_subscription(Float64, "temperature", self.callback_temperature, 10) #criando um subscriber do topico "temperature" do tipo Float64
        self.average_publisher = self.create_publisher(Float64, "average_temperature", 10) #criando um publicador para o tpoico "average_temperature" do tipo Float64
        self.reset_average_service = self.create_service(SetBool, "reset_average", self.callback_reset_average) #criando um servico "reset_average" do tipo SetBool que utiliza o metodo callback_reset_average

    def callback_temperature(self, msg): #metodo para receber a publicacao recebida do topico "temperature"
                                         #e publicar a media no topico average_temperature
        self.temperature_readings.append(msg.data) #insere a leitura da temperatura na lista
        if len(self.temperature_readings) >= 5: #media comeca quando a lista ter 5 termos
            temperature_average = sum(self.temperature_readings)/5 #calculo da media
            average_msg = Float64() #mensagem do tipo Float64
            average_msg.data = temperature_average #o conteudo da msg sera a media da temperatura
            self.average_publisher.publish(average_msg) #publica a mensagem (media da temperatura)
            self.temperature_readings.pop(0) #elimina o primeiro elemento da lista para calculo de uma nova media
    
    def callback_reset_average(self, request, response): #metodo para o servico reset_average

        if request.data == True: #se o no average_reset_client for chamado
            self.temperature_readings = [] #limpa a lista para resetar as informacoes e fazer um novo calculo da media
            response.success = True
            response.message = "The average has been reset"
            self.get_logger().info("The average has been reset")
        else:
            response.success = False
            response.message = "The average has not been reset"
            self.get_logger().info("The average has not been reset")

        return response

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureMonitorNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()