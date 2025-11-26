#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import numpy as np
import time
from threading import Thread

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # ============================================================
        # CONFIGURACIÓN: Ajusta estos valores según tu mapa
        # ============================================================
        self.INITIAL_X = -2.08543         # Posición X inicial en el mapa
        self.INITIAL_Y = 4.9386           # Posición Y inicial en el mapa
        self.INITIAL_YAW = -0.0349756     # Orientación inicial (radianes)
        
        # Tabla de waypoints (x, y, orientación_yaw)
        # IMPORTANTE: Usa coordenadas en zonas ABIERTAS y ACCESIBLES del mapa
        # Evita puntos cerca de paredes o en espacios estrechos
        self.waypoints = [
            (0.03, 0.13, 0.0),      # Waypoint 1
            (4.96, 0.39, 0.0),      # Waypoint 2
            (5.36, 5.53, 1.57),     # Waypoint 3
            (12.49, 0.00, -1.57),   # Waypoint 4
            (12.40, 5.07, 3.14)     # Waypoint 5
        ]
        
        # Tolerancias para considerar que llegó al waypoint
        self.goal_tolerance = 0.5  # metros - más tolerante
        self.max_retries = 2       # Reintentos por waypoint
        # ============================================================
        
        # Publisher para la pose inicial en RViz/AMCL
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Subscriber para obtener la posición actual del robot
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Action client para navegación
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Posición actual del robot
        self.current_pose = None
        self.pose_published = False
        self.current_waypoint_index = 0
        self.navigation_active = False
        self.retry_count = 0
        
        self.get_logger().info('Waypoint Navigator iniciado')
        self.get_logger().info('Esperando que arranquen los servicios...')
        
        # Timer para publicar la pose inicial automáticamente
        self.timer = self.create_timer(0.5, self.check_and_publish_initial_pose)
    
    def odom_callback(self, msg):
        """Callback para actualizar la posición actual del robot"""
        self.current_pose = msg.pose.pose
    
    def check_and_publish_initial_pose(self):
        """Publica la pose inicial automáticamente"""
        if not self.pose_published:
            self.publish_initial_pose()
            self.pose_published = True
            self.timer.cancel()
            
            # Esperar antes de empezar a navegar
            self.get_logger().info('Esperando 5 segundos antes de iniciar navegación...')
            self.nav_timer = self.create_timer(5.0, self.start_navigation)
    
    def publish_initial_pose(self):
        """Publica la pose inicial del robot en RViz/AMCL automáticamente"""
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Usar las coordenadas configuradas
        initial_pose.pose.pose.position.x = self.INITIAL_X
        initial_pose.pose.pose.position.y = self.INITIAL_Y
        initial_pose.pose.pose.position.z = 0.0
        
        # Convertir yaw a quaternion
        initial_pose.pose.pose.orientation.z = np.sin(self.INITIAL_YAW / 2.0)
        initial_pose.pose.pose.orientation.w = np.cos(self.INITIAL_YAW / 2.0)
        
        # Covarianza (incertidumbre inicial)
        initial_pose.pose.covariance[0] = 0.25    # varianza en x
        initial_pose.pose.covariance[7] = 0.25    # varianza en y
        initial_pose.pose.covariance[35] = 0.06853  # varianza en yaw
        
        # Publicar solo 2 veces con pausa entre ellas (evita crash de AMCL)
        self.initial_pose_pub.publish(initial_pose)
        time.sleep(0.5)
        self.initial_pose_pub.publish(initial_pose)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'✓ Pose inicial publicada automáticamente:')
        self.get_logger().info(f'  X: {self.INITIAL_X:.3f}')
        self.get_logger().info(f'  Y: {self.INITIAL_Y:.3f}')
        self.get_logger().info(f'  Yaw: {self.INITIAL_YAW:.3f} rad ({np.degrees(self.INITIAL_YAW):.1f}°)')
        self.get_logger().info('=' * 60)
    
    def start_navigation(self):
        """Inicia la navegación por waypoints"""
        # Cancelar el timer para que no se vuelva a llamar
        if hasattr(self, 'nav_timer'):
            self.nav_timer.cancel()
        
        self.get_logger().info(' Iniciando navegación por waypoints...')
        self.navigation_active = True
        self.current_waypoint_index = 0
        self.send_next_waypoint()
    
    def create_goal_pose(self, x, y, yaw):
        """Crea un mensaje PoseStamped para el objetivo"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # Convertir yaw a quaternion
        goal_pose.pose.orientation.z = np.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = np.cos(yaw / 2.0)
        
        return goal_pose
    
    def send_next_waypoint(self):
        """Envía el siguiente waypoint"""
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('=' * 60)
            self.get_logger().info('CINE ¡Todos los waypoints completados!')
            self.get_logger().info('El robot ha terminado su recorrido.')
            self.get_logger().info('=' * 60)
            self.navigation_active = False
            return
        
        x, y, yaw = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f' Waypoint {self.current_waypoint_index+1}/{len(self.waypoints)}: ({x:.2f}, {y:.2f}, {np.degrees(yaw):.0f}°)')
        
        # Esperar a que el action server esté disponible
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('No se puede conectar al servidor de navegación')
            return
        
        # Crear y enviar el objetivo
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_goal_pose(x, y, yaw)
        
        self.get_logger().info(f' Enviando waypoint {self.current_waypoint_index+1}...')
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        """Callback para recibir feedback durante la navegación"""
        # Opcional: puedes loggear el progreso aquí si quieres
        pass
    
    def goal_response_callback(self, future):
        """Callback cuando el servidor acepta o rechaza el objetivo"""
        goal_handle = future.result()
        
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'TROLEADA Waypoint {self.current_waypoint_index+1} rechazado')
            
            # Reintentar si no se alcanzó el máximo de reintentos
            if self.retry_count < self.max_retries:
                self.retry_count += 1
                self.get_logger().info(f'Reintentando waypoint {self.current_waypoint_index+1} (intento {self.retry_count}/{self.max_retries})...')
                time.sleep(3)
                self.send_next_waypoint()
            else:
                # Pasar al siguiente waypoint
                self.get_logger().warn(f'  Saltando waypoint {self.current_waypoint_index+1} después de {self.max_retries} intentos')
                self.retry_count = 0
                self.current_waypoint_index += 1
                time.sleep(2)
                self.send_next_waypoint()
            return
        
        self.get_logger().info(f' Navegando hacia waypoint {self.current_waypoint_index+1}...')
        self.retry_count = 0  # Reset reintentos al aceptarse
        
        # Obtener el resultado
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Callback cuando la navegación termina"""
        result = future.result()
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f'GOOOOOOD Waypoint {self.current_waypoint_index+1} completado exitosamente')
            self.retry_count = 0
        elif result.status == 6:  # ABORTED
            self.get_logger().warn(f'TROLEADA Waypoint {self.current_waypoint_index+1} abortado (estado: {result.status})')
            
            # Reintentar si no se alcanzó el máximo
            if self.retry_count < self.max_retries:
                self.retry_count += 1
                self.get_logger().info(f'Reintentando waypoint {self.current_waypoint_index+1} (intento {self.retry_count}/{self.max_retries})...')
                time.sleep(3)
                self.send_next_waypoint()
                return
            else:
                self.get_logger().warn(f'  Saltando waypoint {self.current_waypoint_index+1} después de {self.max_retries} intentos')
                self.retry_count = 0
        else:
            self.get_logger().warn(f'  Waypoint {self.current_waypoint_index+1} terminó con estado: {result.status}')
            self.retry_count = 0
        
        # Pausa antes del siguiente waypoint
        self.get_logger().info('Esperando 3 segundos antes del siguiente waypoint...')
        time.sleep(3)
        
        # Ir al siguiente waypoint
        self.current_waypoint_index += 1
        self.send_next_waypoint()


def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()