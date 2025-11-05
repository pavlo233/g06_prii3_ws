#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JetBot que dibuja un 6 y rodea obstáculos detectados por LIDAR apuntando hacia atrás.
Durante la evasión, no se cuenta el tiempo de dibujo.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


class JetbotDrawLidar(Node):
    def __init__(self):
        super().__init__('jetbot_draw_lidar')

        # Publishers & Subscribers
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_sub_ = self.create_subscription(String, 'turtle_command', self.command_callback, 10)
        self.lidar_sub_ = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Timer principal
        self.timer = self.create_timer(0.1, self.move)

        # Estado
        self.reset()
        self.obstacle_detected = False
        self.evasion_state = None  # None o "active"
        self.evasion_start_time = 0.0
        self.avoid_direction = 1  # 1 = derecha, -1 = izquierda

        # Parámetros
        self.declare_parameter('obstacle_distance', 0.3)
        self.declare_parameter('rear_angle', 30.0)
        self.declare_parameter('back_speed', 0.15)
        self.declare_parameter('turn_speed', 0.4)

    # -----------------------------
    # CONTROL MANUAL
    # -----------------------------
    def reset(self):
        self.get_logger().info("Reiniciando dibujo del número 6...")
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.phase = 0
        self.active = True
        self.paused_time = 0.0
        self.last_pause = None
        self.evasion_state = None
        self.obstacle_detected = False

    def pause(self):
        if self.active:
            self.active = False
            self.last_pause = self.get_clock().now().seconds_nanoseconds()[0]
            self.publisher_.publish(Twist())
            self.get_logger().warn("Robot pausado.")

    def resume(self):
        if not self.active:
            now = self.get_clock().now().seconds_nanoseconds()[0]
            if self.last_pause:
                self.paused_time += now - self.last_pause
            self.active = True
            self.get_logger().info("Robot reanudado.")

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "pause":
            self.pause()
        elif cmd == "resume":
            self.resume()
        elif cmd == "reset":
            self.reset()

    # -----------------------------
    # LIDAR CALLBACK
    # -----------------------------
    def lidar_callback(self, msg: LaserScan):
        limit = float(self.get_parameter('obstacle_distance').value)
        rear_angle = float(self.get_parameter('rear_angle').value)

        total_points = len(msg.ranges)
        if total_points == 0:
            return

        rear_rad = math.radians(rear_angle)
        angle_behind = msg.angle_min 
        rear_index = int((angle_behind - msg.angle_min) / msg.angle_increment)

        
        window_size = int(rear_rad / msg.angle_increment)

        start_index = max(rear_index - window_size, 0)
        end_index = min(rear_index + window_size, total_points - 1)

        valid_ranges = [r for r in msg.ranges[start_index:end_index] if 0.01 < r < float('inf')]
        if not valid_ranges:
            return

        min_distance = min(valid_ranges)

        if min_distance < limit and self.evasion_state is None:
            self.obstacle_detected = True
            self.evasion_state = "active"
            self.evasion_start_time = self.get_clock().now().seconds_nanoseconds()[0]
            # Decidir dirección con más espacio
            left_space = sum([r for r in msg.ranges[start_index:rear_index] if r < float('inf')])
            right_space = sum([r for r in msg.ranges[rear_index:end_index] if r < float('inf')])
            self.avoid_direction = 1 if right_space > left_space else -1
            self.get_logger().warn(f"Obstáculo detectado a {min_distance:.2f} m. Iniciando evasión...")

        elif min_distance >= limit and self.evasion_state == "active":
            # Camino despejado
            self.obstacle_detected = False
            self.evasion_state = None
            now = self.get_clock().now().seconds_nanoseconds()[0]
            # No contar tiempo de evasión
            self.paused_time += now - self.evasion_start_time
            self.get_logger().info("Evasión completada. Retomando dibujo.")

    # -----------------------------
    # MOVIMIENTO DEL ROBOT
    # -----------------------------
    def move(self):
        if not self.active:
            return

        msg = Twist()
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = current_time - self.start_time - self.paused_time

        # Modo evasión
        if self.evasion_state == "active":
            # Avanza y gira suavemente para rodear el obstáculo
            msg.linear.x = float(self.get_parameter('back_speed').value)
            msg.angular.z = float(self.get_parameter('turn_speed').value) * self.avoid_direction
            self.publisher_.publish(msg)
            return

        # Movimiento normal para dibujar el 6
        if self.phase == 0:
            msg.linear.x = 0.0
            msg.angular.z = 0.6
            if elapsed > 1.0:
                self.phase = 1
                self.start_time = current_time
                self.get_logger().info("Fase 1: Iniciando trazo curvo")

        elif self.phase == 1:
            msg.linear.x = 0.2
            msg.angular.z = 0.5
            if elapsed > 3.0:
                self.phase = 2
                self.start_time = current_time
                self.get_logger().info("Fase 2: Giro más cerrado")

        elif self.phase == 2:
            msg.linear.x = 0.1
            msg.angular.z = 0.75
            if elapsed > 7.0:
                self.phase = 3
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_.publish(msg)
                self.get_logger().info("Dibujo del número 6 completado.")
                return

        if self.phase < 3:
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JetbotDrawLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
