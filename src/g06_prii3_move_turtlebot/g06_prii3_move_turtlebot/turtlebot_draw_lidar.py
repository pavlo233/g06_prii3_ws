
#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class TurtlebotDrawLidar(Node):
    def __init__(self):
        super().__init__('turtlebot_draw_lidar')

        # Publicadores y suscriptores
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_sub_ = self.create_subscription(String, 'turtle_command', self.command_callback, 10)
        self.lidar_sub_ = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Temporizadores
        self.timer_move = self.create_timer(0.1, self.move)   # movimiento
        self.timer_check = self.create_timer(0.2, self.check_status)  # chequeo del entorno

        # Estado
        self.reset()
        self.obstacle_detected = False
        self.last_min_distance = float('inf')
        self.last_lidar_time = 0.0

        # Parámetros
        self.declare_parameter('obstacle_distance', 0.35)
        self.declare_parameter('front_angle', 35.0)

    def get_time(self):
        sec, nsec = self.get_clock().now().seconds_nanoseconds()
        return sec + nsec / 1e9

    def reset(self):
        self.get_logger().info("Reiniciando dibujo del número 6 para TurtleBot3...")
        self.start_time = self.get_time()
        self.phase = 0
        self.active = True
        self.paused_time = 0.0
        self.last_pause = None

   
    def pause(self):
        if self.active:
            self.active = False
            self.last_pause = self.get_time()
            self.publisher_.publish(Twist())
            self.get_logger().warn("Robot pausado.")

    def resume(self):
        if not self.active:
            now = self.get_time()
            if self.last_pause:
                self.paused_time += now - self.last_pause
            self.active = True
            self.get_logger().info("Robot reanudado.")

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'pause':
            self.pause()
        elif cmd == 'resume':
            self.resume()
        elif cmd == 'reset':
            self.reset()

   
    def lidar_callback(self, msg: LaserScan):
        limit = float(self.get_parameter('obstacle_distance').value)
        front_angle = float(self.get_parameter('front_angle').value)
        total_points = len(msg.ranges)
        if total_points == 0:
            return

        angles = [msg.angle_min + i * msg.angle_increment for i in range(total_points)]
        front_rad = math.radians(front_angle)
        front_indices = [i for i, a in enumerate(angles) if -front_rad <= a <= front_rad]

        valid_ranges = [msg.ranges[i] for i in front_indices if 0.05 < msg.ranges[i] < 10.0]
        if not valid_ranges:
            return

        self.last_min_distance = min(valid_ranges)
        self.last_lidar_time = self.get_time()

   
    def check_status(self):
        limit = float(self.get_parameter('obstacle_distance').value)
        margin = 0.1

        # Si hace mucho que no llega LIDAR, no hacer nada
        if self.get_time() - self.last_lidar_time > 2.0:
            return

        # Estado de detección
        if self.last_min_distance < limit:
            #if not self.obstacle_detected:
                self.obstacle_detected = True
                self.pause()
                self.get_logger().warn(f"Obstáculo detectado a {self.last_min_distance:.2f} m.")
        else:
            if self.obstacle_detected:
                self.obstacle_detected = False
                self.resume()
                self.get_logger().info(f"Camino despejado ({self.last_min_distance:.2f} m).")

  
    def move(self):
        if self.obstacle_detected or not self.active:
            stop = Twist()
            self.publisher_.publish(stop)
            return

        msg = Twist()
        current_time = self.get_time()
        elapsed = current_time - self.start_time - self.paused_time

        # Fase 0: giro inicial
        if self.phase == 0:
            msg.linear.x = 0.0
            msg.angular.z = 0.6
            if elapsed > 0.1:
                self.phase = 1
                self.start_time = current_time
                self.get_logger().info("Fase 1: trazo curvo")

        # Fase 1: avance curvo
        elif self.phase == 1:
            msg.linear.x = 0.2
            msg.angular.z = 0.5
            if elapsed > 5.0:
                self.phase = 2
                self.start_time = current_time
                self.get_logger().info("Fase 2: giro más cerrado")

        # Fase 2: giro fuerte
        elif self.phase == 2:
            msg.linear.x = 0.1
            msg.angular.z = 0.75
            if elapsed > 9.0:
                self.phase = 3
                self.publisher_.publish(Twist())
                self.get_logger().info("Dibujo del número 6 completado.")
                return

        if self.phase < 3:
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotDrawLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
