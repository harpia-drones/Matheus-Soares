#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from temperature_interfaces.msg import Temperature #importando uma custom msg do tipo Temperature
from temperature_interfaces.srv import ResetAvg #importanto um custom service do tipo ResetAvg

class TemperatureMonitorNode(Node):
    def __init__(self):
        super().__init__("temperature_monitor")
        self.temperature_readings = [] #lista para armazenar as leituras das temperaturas
        self.subscriber = self.create_subscription(Temperature, "temperature", self.callback_temperature, 10) #criando um subscriber do topico "temperature" do tipo Temperature
        self.average_publisher = self.create_publisher(Temperature, "average_temperature", 10) #criando um publicador para o tpoico "average_temperature" do tipo Temperature
        self.reset_average_service = self.create_service(ResetAvg, "reset_average", self.callback_reset_average) #criando um servico "reset_average" do tipo custom ResetAvg que utiliza o metodo callback_reset_average

    def callback_temperature(self, msg): #metodo para receber a publicacao recebida do topico "temperature"
                                                     #e publicar a media no topico average_temperature
        self.temperature_readings.append(msg.temperature) #insere a leitura da temperatura na lista
        average_msg = Temperature()
        if len(self.temperature_readings) >= 5: #media comeca quando a lista ter 5 termos
            temperature_average = sum(self.temperature_readings)/5 #calculo da media
            average_msg.avg = temperature_average #o conteudo da msg sera a media da temperatura
            self.temperature_readings.pop(0) #elimina o primeiro elemento da lista para calculo de uma nova media
        
        average_msg.temperature = msg.temperature
        self.average_publisher.publish(average_msg) #publica a mensagem
    def callback_reset_average(self, request, response): #metodo para o servico reset_average

        if request.reset_avg == True: #se o no average_reset_client for chamado
            self.temperature_readings = [] #limpa a lista para resetar as informacoes e fazer um novo calculo da media
            response.success = True
            self.get_logger().info("The average has been reset")
        else:
            response.success = False
            self.get_logger().info("The average has not been reset")

        return response

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureMonitorNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()