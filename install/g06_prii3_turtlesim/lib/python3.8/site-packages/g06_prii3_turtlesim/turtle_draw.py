import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

class TurtleDraw(Node):
    def __init__(self):
        super().__init__('turtle_draw')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.command_sub_ = self.create_subscription(String, 'turtle_command', self.command_callback, 10)
        self.timer = self.create_timer(0.1, self.move)
        self.reset()

    def reset(self):
        self.get_logger().info("Reiniciando dibujo del número 6...")
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.phase = 0
        self.active = True
        self.paused_time = 0.0
        self.last_pause = None

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "pause":
            if self.active:
                self.active = False
                self.last_pause = self.get_clock().now().seconds_nanoseconds()[0]
                self.get_logger().info("Dibujo pausado.")
        elif cmd == "resume":
            if not self.active:
                now = self.get_clock().now().seconds_nanoseconds()[0]
                self.paused_time += now - self.last_pause
                self.active = True
                self.get_logger().info("Dibujo reanudado.")
        elif cmd == "reset":
            self.reset()

    def move(self):
        if not self.active:
            return

        msg = Twist()
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = current_time - self.start_time - self.paused_time

        
        if self.phase == 0:
            
            msg.linear.x = 0.0
            msg.angular.z = 2.0  
            if elapsed > 1: 
                self.phase = 1
                self.start_time = current_time
                self.get_logger().info("Fase 1: giro")
                
        elif self.phase == 1:
           
            msg.linear.x = 1.0
            msg.angular.z = 1.0  
            if elapsed > 1.28:  
                self.phase = 2
                self.start_time = current_time
                
                self.get_logger().info("gancho.")
        elif self.phase == 2:
            
            msg.linear.x = 2.0
            msg.angular.z = 4.0  
            if elapsed > 1.28:  
                self.phase = 3
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
                self.get_logger().info("Dibujo del número 6 completado correctamente.")


        if self.phase < 3:
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleDraw()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()