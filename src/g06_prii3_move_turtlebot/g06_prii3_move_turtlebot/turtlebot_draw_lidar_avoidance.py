#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class TurtlebotDrawLidarAvoidanceFullFinal(Node):
    def __init__(self):
        super().__init__('turtlebot_draw_lidar_avoidance_fullfinal')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_sub = self.create_subscription(String, 'turtle_command', self.cmd_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.1, self.loop)

        self.reset()
        self.mode = "normal"
        self.avoid_start = 0.0
        self.initial_turn_done = False

        # Variables odometría
        self.yaw_start = None
        self.current_yaw = 0.0
        self.ninety_turn_detected = False

        # Variables realineación
        self.realign_phase = 0
        self.realign_phase_start = 0.0

        # Distancias lidar
        self.front_dist = 1.0
        self.right_dist = 1.0
        self.left_dist = 1.0
        self.front_right_dist = 1.0
        self.front_left_dist = 1.0
        self.min_dist = 1.0

        # Parámetros
        self.declare_parameter('front_threshold', 0.60)
        self.declare_parameter('side_target', 0.45)
        self.declare_parameter('kp_side', 1.2)
        self.declare_parameter('min_avoid_time', 8.0)
        self.declare_parameter('max_angular', 0.8)

    # ---------------- UTILIDADES ----------------
    def get_time(self):
        s, ns = self.get_clock().now().seconds_nanoseconds()
        return s + ns / 1e9

    def clean(self, val):
        return val if (not math.isinf(val) and not math.isnan(val)) else 5.0

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ---------------- CALLBACKS ----------------
    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q)

    def cmd_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == "pause":
            self.active = False
            self.last_pause = self.get_time()
            self.pub.publish(Twist())
            self.get_logger().info("Pausado manualmente.")
        elif cmd == "resume":
            if not self.active:
                now = self.get_time()
                if self.last_pause:
                    self.paused_time += now - self.last_pause
                self.active = True
                self.get_logger().info("Reanudado manualmente.")
        elif cmd == "reset":
            self.reset()

    def scan_cb(self, msg: LaserScan):
        total = len(msg.ranges)
        if total == 0:
            return
        angles = [msg.angle_min + i * msg.angle_increment for i in range(total)]

        def avg(a0, a1):
            vals = [self.clean(msg.ranges[i]) for i, a in enumerate(angles)
                    if a0 <= a <= a1 and 0.05 < msg.ranges[i] < 5.0]
            return sum(vals)/len(vals) if vals else 5.0

        self.front_dist = avg(-0.20, 0.20)
        self.right_dist = avg(-1.57, -1.3)
        self.left_dist = avg(1.3, 1.57)
        self.front_right_dist = avg(-0.8, -0.4)
        self.front_left_dist = avg(0.4, 0.8)
        vals = [self.clean(r) for r in msg.ranges if 0.05 < r < 5.0]
        self.min_dist = min(vals) if vals else 5.0

    # ---------------- MODOS ----------------
    def reset(self):
        self.get_logger().info("Reiniciando dibujo del número 6...")
        self.start_time = self.get_time()
        self.phase = 0
        self.active = True
        self.paused_time = 0.0
        self.last_pause = None

    def start_avoid(self):
        self.mode = "avoid"
        self.avoid_start = self.get_time()
        self.yaw_start = self.current_yaw  # Guardamos orientación inicial
        self.ninety_turn_detected = False
        self.initial_turn_done = False
        self.pause_time = self.get_time()
        self.get_logger().warn("Obstáculo detectado → rodeando por la derecha.")

    def start_realign(self):
        self.mode = "realign"
        self.realign_phase = 0
        self.realign_phase_start = self.get_time()
        self.get_logger().info("Giro 90° detectado → realineando hacia la derecha.")

    def stop_avoid(self):
     if self.mode in ["avoid", "realign"]:
        self.mode = "normal"
        self.initial_turn_done = False
        self.pub.publish(Twist())
        self.get_logger().info("Retomando dibujo del 6.")
        # Reiniciar tiempo para el 6
        self.start_time = self.get_time()
        self.paused_time = 0.0


    # ---------------- LOOP PRINCIPAL ----------------
    def loop(self):
        msg = Twist()
        now = self.get_time()

        if not self.active:
            self.pub.publish(Twist())
            return

        front_thr = float(self.get_parameter('front_threshold').value)
        side_target = float(self.get_parameter('side_target').value)
        kp_side = float(self.get_parameter('kp_side').value)
        max_angular = float(self.get_parameter('max_angular').value)

        corner_detected = (self.front_dist < front_thr or 
                           self.front_right_dist < (front_thr + 0.1) or
                           self.front_left_dist < (front_thr + 0.1))

        # ---------------- DETECCIÓN DE OBSTÁCULO ----------------
        if self.mode == "normal" and corner_detected:
            self.start_avoid()
            return

        # ---------------- REALINEACIÓN CONTROLADA ----------------
        if self.mode == "realign":
            elapsed = now - self.realign_phase_start
            if self.realign_phase == 0:  # Giro 1/4 vuelta derecha
                msg.linear.x = 0.0
                msg.angular.z = -0.8
                if elapsed > 1.5:
                    self.realign_phase = 1
                    self.realign_phase_start = now
            elif self.realign_phase == 1:  # Avanzar recto 2s
                msg.linear.x = 0.25
                msg.angular.z = 0.0
                if elapsed > 2.0:
                    self.stop_avoid()
            self.pub.publish(msg)
            return

        # ---------------- MODO AVOID ----------------
        if self.mode == "avoid":
            elapsed_avoid = now - self.avoid_start

            # Giro inicial
            if not self.initial_turn_done:
                msg.linear.x = 0.08
                msg.angular.z = -0.9
                self.pub.publish(msg)
                if self.front_dist > (front_thr + 0.4) and self.front_right_dist > 0.7:
                    self.initial_turn_done = True
                    self.get_logger().info("Siguiendo pared izquierda del obstáculo.")
                return

            # Seguimiento de pared
            if self.left_dist > 2.0:
                msg.linear.x = 0.22
                msg.angular.z = 0.0
            else:
                error = side_target - self.left_dist
                error = self.clamp(error, -0.5, 0.5)
                msg.angular.z = kp_side * error

            if self.front_dist < 0.6 or self.front_left_dist < 0.5:
                msg.angular.z -= 0.6
                msg.linear.x = 0.12

            msg.angular.z = self.clamp(msg.angular.z, -max_angular, max_angular)
            self.pub.publish(msg)

            # ---------------- DETECCIÓN DE GIRO 90° IZQUIERDA ----------------
            if self.yaw_start is not None:
                yaw_diff = self.current_yaw - self.yaw_start
                yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi  # Normaliza [-pi, pi]
                if yaw_diff > math.pi * 0.45 and not self.ninety_turn_detected:  # ≈90°
                    self.ninety_turn_detected = True
                    self.start_realign()
                    return
            return

        # ---------------- DIBUJO NORMAL DEL 6 ----------------
        elapsed = now - self.start_time - self.paused_time
        if self.phase == 0:
            msg.linear.x = 0.0
            msg.angular.z = 0.6
            if elapsed > 0.1:
                self.phase = 1
                self.start_time = now
                self.get_logger().info("Fase 1: trazo curvo del 6.")
        elif self.phase == 1:
            msg.linear.x = 0.2
            msg.angular.z = 0.5
            if elapsed > 5.0:
                self.phase = 2
                self.start_time = now
                self.get_logger().info("Fase 2: giro cerrado del 6.")
        elif self.phase == 2:
            msg.linear.x = 0.1
            msg.angular.z = 0.75
            if elapsed > 9.0:
                self.phase = 3
                self.pub.publish(Twist())
                self.get_logger().info("Dibujo del 6 completado.")
                return

        if self.phase < 3:
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotDrawLidarAvoidanceFullFinal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
