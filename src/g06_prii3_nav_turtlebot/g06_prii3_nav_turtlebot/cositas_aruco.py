#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

class ArucoWaypointNavigator(Node):
    def __init__(self):
        super().__init__('aruco_waypoint_navigator')
        
        # ============================================================
        # CONFIGURACIÓN: Ajusta estos valores según tu mapa
        # ============================================================
        self.INITIAL_X = -2.08543
        self.INITIAL_Y = 4.9386
        self.INITIAL_YAW = -0.0349756
        
        # Waypoints definidos
        self.WAYPOINT_1 = (0.03, 0.13, 0.0)       # Punto de detección
        self.WAYPOINT_2 = (4.96, 0.39, 0.0)       # Punto intermedio (no se usa)
        self.WAYPOINT_3 = (5.36, 5.53, 1.57)      # Para ArUco ID 0
        self.WAYPOINT_4 = (12.49, 0.00, -1.57)    # Para ArUco ID 1
        self.WAYPOINT_5 = (12.40, 5.07, 3.14)     # Para ArUco ID 10
        
        self.max_retries = 2
        # ============================================================
        
        # Publishers y Subscribers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Subscriber para la cámara
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Ajusta el topic según tu robot
            self.image_callback,
            10
        )
        
        # Action client para navegación
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Variables de estado
        self.current_pose = None
        self.pose_published = False
        self.retry_count = 0
        self.navigation_state = 'INIT'  # INIT -> GOING_TO_POINT1 -> DETECTING -> GOING_TO_FINAL -> DONE
        self.detected_aruco_id = None
        self.detection_attempts = 0
        self.max_detection_attempts = 30  # 30 frames para detectar (unos 10 segundos)
        
        # OpenCV
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        self.get_logger().info('ArUco Waypoint Navigator iniciado')
        self.get_logger().info('Esperando que arranquen los servicios...')
        
        # Timer para publicar la pose inicial
        self.timer = self.create_timer(0.5, self.check_and_publish_initial_pose)
    
    def odom_callback(self, msg):
        """Callback para actualizar la posición actual del robot"""
        self.current_pose = msg.pose.pose
    
    def image_callback(self, msg):
        """Callback para procesar imágenes de la cámara"""
        # Solo detectar ArUco cuando estamos en estado DETECTING
        if self.navigation_state != 'DETECTING':
            return
        
        try:
            # Convertir imagen ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detectar marcadores ArUco
            corners, ids, rejected = cv2.aruco.detectMarkers(
                cv_image, 
                self.aruco_dict, 
                parameters=self.aruco_params
            )
            
            if ids is not None and len(ids) > 0:
                for marker_id in ids.flatten():
                    self.get_logger().info(f' ArUco detectado: ID {marker_id}')
                    
                    # Verificar si es uno de los IDs que nos interesan
                    if marker_id in [0, 1, 10]:
                        self.detected_aruco_id = marker_id
                        self.get_logger().info(f'cineeeeee ArUco ID {marker_id} confirmado!')
                        self.navigation_state = 'ARUCO_DETECTED'
                        self.go_to_final_waypoint()
                        return
            
            # Incrementar contador de intentos
            self.detection_attempts += 1
            
            if self.detection_attempts >= self.max_detection_attempts:
                self.get_logger().warn('sad No se detectó ningún ArUco válido después de múltiples intentos')
                self.get_logger().warn('Yendo a waypoint por defecto (Waypoint 3)')
                self.detected_aruco_id = 0  # Por defecto
                self.navigation_state = 'ARUCO_DETECTED'
                self.go_to_final_waypoint()
                
        except Exception as e:
            self.get_logger().error(f'Error procesando imagen: {e}')
    
    def check_and_publish_initial_pose(self):
        """Publica la pose inicial automáticamente"""
        if not self.pose_published:
            self.publish_initial_pose()
            self.pose_published = True
            self.timer.cancel()
            
            self.get_logger().info('Esperando 5 segundos antes de iniciar navegación...')
            self.nav_timer = self.create_timer(5.0, self.start_navigation)
    
    def publish_initial_pose(self):
        """Publica la pose inicial del robot en RViz/AMCL automáticamente"""
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        
        initial_pose.pose.pose.position.x = self.INITIAL_X
        initial_pose.pose.pose.position.y = self.INITIAL_Y
        initial_pose.pose.pose.position.z = 0.0
        
        initial_pose.pose.pose.orientation.z = np.sin(self.INITIAL_YAW / 2.0)
        initial_pose.pose.pose.orientation.w = np.cos(self.INITIAL_YAW / 2.0)
        
        initial_pose.pose.covariance[0] = 0.25
        initial_pose.pose.covariance[7] = 0.25
        initial_pose.pose.covariance[35] = 0.06853
        
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
        """Inicia la navegación hacia el punto 1"""
        if hasattr(self, 'nav_timer'):
            self.nav_timer.cancel()
        
        self.get_logger().info(' Iniciando navegación hacia Waypoint 1 (punto de detección)...')
        self.navigation_state = 'GOING_TO_POINT1'
        self.go_to_waypoint(self.WAYPOINT_1, 'Waypoint 1')
    
    def create_goal_pose(self, x, y, yaw):
        """Crea un mensaje PoseStamped para el objetivo"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        goal_pose.pose.orientation.z = np.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = np.cos(yaw / 2.0)
        
        return goal_pose
    
    def go_to_waypoint(self, waypoint, waypoint_name):
        """Envía un waypoint al robot"""
        x, y, yaw = waypoint
        self.get_logger().info(f' Navegando hacia {waypoint_name}: ({x:.2f}, {y:.2f}, {np.degrees(yaw):.0f}°)')
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('No se puede conectar al servidor de navegación')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_goal_pose(x, y, yaw)
        
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        """Callback para feedback durante la navegación"""
        pass
    
    def goal_response_callback(self, future):
        """Callback cuando el servidor acepta o rechaza el objetivo"""
        goal_handle = future.result()
        
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'❌ Objetivo rechazado')
            
            if self.retry_count < self.max_retries:
                self.retry_count += 1
                self.get_logger().info(f'Reintentando (intento {self.retry_count}/{self.max_retries})...')
                time.sleep(3)
                
                # Reintentar según el estado
                if self.navigation_state == 'GOING_TO_POINT1':
                    self.go_to_waypoint(self.WAYPOINT_1, 'Waypoint 1')
                elif self.navigation_state == 'GOING_TO_FINAL':
                    final_waypoint = self.get_final_waypoint()
                    self.go_to_waypoint(final_waypoint[0], final_waypoint[1])
            else:
                self.get_logger().error(' No se pudo navegar después de múltiples intentos')
                self.navigation_state = 'DONE'
            return
        
        self.get_logger().info(f'✓ Objetivo aceptado, navegando...')
        self.retry_count = 0
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Callback cuando la navegación termina"""
        result = future.result()
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f'good Navegación completada exitosamente')
            
            if self.navigation_state == 'GOING_TO_POINT1':
                # Llegamos al punto 1, empezar detección
                self.get_logger().info('scaning Iniciando detección de ArUco...')
                self.get_logger().info('Buscando marcadores ArUco (IDs: 0, 1, 10)...')
                self.navigation_state = 'DETECTING'
                self.detection_attempts = 0
                
            elif self.navigation_state == 'GOING_TO_FINAL':
                # Llegamos al destino final
                self.get_logger().info('=' * 60)
                self.get_logger().info('milagro ¡Misión completada!')
                self.get_logger().info(f'ArUco detectado: ID {self.detected_aruco_id}')
                self.get_logger().info('=' * 60)
                self.navigation_state = 'DONE'
                
        elif result.status == 6:  # ABORTED
            self.get_logger().warn(f'fooook Navegación abortada')
            
            if self.retry_count < self.max_retries:
                self.retry_count += 1
                time.sleep(3)
                
                if self.navigation_state == 'GOING_TO_POINT1':
                    self.go_to_waypoint(self.WAYPOINT_1, 'Waypoint 1')
                elif self.navigation_state == 'GOING_TO_FINAL':
                    final_waypoint = self.get_final_waypoint()
                    self.go_to_waypoint(final_waypoint[0], final_waypoint[1])
            else:
                self.navigation_state = 'DONE'
        else:
            self.get_logger().warn(f' Navegación terminó con estado: {result.status}')
            self.navigation_state = 'DONE'
    
    def go_to_final_waypoint(self):
        """Navega al waypoint final según el ArUco detectado"""
        waypoint_map = {
            0: (self.WAYPOINT_3, 'Waypoint 3 (ArUco ID 0)'),
            1: (self.WAYPOINT_4, 'Waypoint 4 (ArUco ID 1)'),
            10: (self.WAYPOINT_5, 'Waypoint 5 (ArUco ID 10)')
        }
        
        if self.detected_aruco_id in waypoint_map:
            waypoint, name = waypoint_map[self.detected_aruco_id]
            self.get_logger().info(f' Navegando hacia {name}')
            self.navigation_state = 'GOING_TO_FINAL'
            time.sleep(2)  # Pausa antes de iniciar
            self.go_to_waypoint(waypoint, name)
        else:
            self.get_logger().error(f'❌ ID de ArUco no reconocido: {self.detected_aruco_id}')
            self.navigation_state = 'DONE'
    
    def get_final_waypoint(self):
        """Retorna el waypoint final según el ArUco detectado"""
        waypoint_map = {
            0: (self.WAYPOINT_3, 'Waypoint 3'),
            1: (self.WAYPOINT_4, 'Waypoint 4'),
            10: (self.WAYPOINT_5, 'Waypoint 5')
        }
        return waypoint_map.get(self.detected_aruco_id, (self.WAYPOINT_3, 'Waypoint 3'))


def main(args=None):
    rclpy.init(args=args)
    navigator = ArucoWaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()