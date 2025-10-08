import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String

class TurtleService(Node):
    def __init__(self):
        super().__init__('turtle_service')
        self.publisher_ = self.create_publisher(String, 'turtle_command', 10)
        self.srv = self.create_service(Trigger, 'control_drawing', self.control_callback)
        self.get_logger().info('Servicio listo: /control_drawing')
        self.commands = ["pause", "resume", "reset"]

    def control_callback(self, request, response):
        self.get_logger().info("Llamada recibida al servicio /control_drawing")
        self.get_logger().info("Comandos disponibles: pause, resume, reset")
        self.get_logger().info("Introduce el comando en la terminal (no automático).")
        response.success = True
        response.message = "Servicio activo. Usa 'ros2 topic pub' para enviar comandos."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TurtleService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
