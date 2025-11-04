import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String

class JetbotService(Node):
    def __init__(self):
        super().__init__('jetbot_service')
        self.publisher_ = self.create_publisher(String, 'turtle_command', 10)
        self.srv = self.create_service(Trigger, 'control_jetbot', self.control_callback)
        self.get_logger().info('Servicio listo: /control_jetbot')
        self.commands = ["pause", "resume", "reset"]

    def control_callback(self, request, response):
        self.get_logger().info("Llamada recibida al servicio /control_jetbot")
        self.get_logger().info("Comandos disponibles: pause, resume, reset")
        self.get_logger().info("Introduce el comando manualmente con 'ros2 topic pub'.")
        response.success = True
        response.message = "Servicio JetBot activo. Usa 'ros2 topic pub /turtle_command std_msgs/String ...' para enviar comandos."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = JetbotService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()