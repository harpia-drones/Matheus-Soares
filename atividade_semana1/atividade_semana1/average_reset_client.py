#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from temperature_interfaces.srv import ResetAvg #importanto um custom service do tipo ResetAvg

class AverageResetNode(Node):
    def __init__(self):
        super().__init__("average_reset_client")
        self.call_reset_average(True) #chamada da funcao call_reset_average com argumento True para que
                                      #quando o no for chamado, o calculo da media seja resetada
    def call_reset_average(self, reset):
        client = self.create_client(ResetAvg, "reset_average") #criando um cliente para o servico reset_average
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Reset Average...")

        request = ResetAvg.Request()
        request.reset_avg = reset

        future = client.call_async(request)
        future.add_done_callback(self.callback_call_reset_average)
    
    def callback_call_reset_average(self, future): #pega a resposta futura e a analisa
        try:
            response = future.result()
            if response.success == True: #servico conseguiu se comunicar com o cliente e resetou
                self.get_logger().info("Average reset successfully")
            else: #servico conseguiu se comunicar com o cliente, mas por algum motivo nao resetou
                self.get_logger().info("Failed to reset average")

        except Exception as e: #caso ocorra algum erro
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AverageResetNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()