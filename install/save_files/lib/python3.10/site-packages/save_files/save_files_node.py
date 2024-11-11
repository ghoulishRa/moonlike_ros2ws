import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
from datetime import datetime
import os
from sensor_msgs_py import point_cloud2 as pc2

class Save_Files(Node):

    def __init__(self):
        super().__init__('image_saver_node')

        # Suscripciones al tópico de la cámara
        self.image_subscription = self.create_subscription(
            Image,
            '/thermal_camera/image_raw',  
            self.image_listener_callback,
            10) 
        
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            '/velodyne_points',  
            self.pointcloud_listener_callback,
            10)

        # Publishers para la imagen y la nube de puntos
        self.image_publisher = self.create_publisher(Image, '/time_stamp/thermal_camera/raw_image', 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, '/time_stamp/lidar/pointcloud', 10)

        self.bridge = CvBridge()

        # Variables para almacenar la última imagen y nube de puntos recibidas
        self.current_image = None
        self.current_pointcloud = None

        # Crea un temporizador que llamará a save_files_callback cada 8 segundos
        self.save_timer = self.create_timer(5.0, self.save_files_callback)

    def image_listener_callback(self, msg):
        try:
            # Convierte el mensaje de imagen de ROS a OpenCV
            self.current_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        except Exception as e:
            self.get_logger().error(f"Error al recibir la imagen térmica: {e}")

    def pointcloud_listener_callback(self, msg):
        # Almacena la última nube de puntos recibida
        self.current_pointcloud = msg

    def save_files_callback(self):
        # Guardar la imagen térmica si está disponible
        if self.current_image is not None:
            try:
                # Crear nombre de archivo con timestamp
                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S%f")
                filename = f"/root/rosbag/timestamp/thermal/thermal_image_{timestamp}.jpg"

                # Guarda la imagen en disco
                cv2.imwrite(filename, self.current_image)
                self.get_logger().info(f"Imagen guardada: {filename}")

                # Publicar la imagen
                image_msg = self.bridge.cv2_to_imgmsg(self.current_image, encoding='mono8')
                self.image_publisher.publish(image_msg)
                self.get_logger().info("Imagen publicada en el tópico /thermal_camera/saved_image")

            except Exception as e:
                self.get_logger().error(f"Error al guardar la imagen térmica: {e}")
        else:
            self.get_logger().warn("No hay imagen disponible para guardar.")

        # Guardar la nube de puntos si está disponible
        if self.current_pointcloud is not None:
            try:
                # Crear un nombre de archivo único usando el timestamp
                timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S%f")
                pcl_filename = f"/root/rosbag/timestamp/lidar/lidar_{timestamp}.pcd"
                os.makedirs(os.path.dirname(pcl_filename), exist_ok=True)

                # Leer los puntos del mensaje PointCloud2 y escribir en .pcd
                with open(pcl_filename, 'w') as f:
                    f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                    f.write("VERSION 0.7\n")
                    f.write("FIELDS x y z\n")
                    f.write("SIZE 4 4 4\n")
                    f.write("TYPE F F F\n")
                    f.write("COUNT 1 1 1\n")
                    f.write(f"WIDTH {self.current_pointcloud.width * self.current_pointcloud.height}\n")
                    f.write("HEIGHT 1\n")
                    f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                    f.write(f"POINTS {self.current_pointcloud.width * self.current_pointcloud.height}\n")
                    f.write("DATA ascii\n")
                    for point in pc2.read_points(self.current_pointcloud, field_names=("x", "y", "z"), skip_nans=True):
                        f.write(f"{point[0]} {point[1]} {point[2]}\n")

                self.get_logger().info(f"Saved PCD file to {pcl_filename}")

                # Publicar la nube de puntos
                self.pointcloud_publisher.publish(self.current_pointcloud)
                self.get_logger().info("Nube de puntos publicada en el tópico /velodyne/saved_points")

            except Exception as e:
                self.get_logger().error(f"Failed to save camera_02 PCD file: {e}")
        else:
            self.get_logger().warn("No hay nube de puntos disponible para guardar.")

def main(args=None):
    rclpy.init(args=args)
    image_saver_node = Save_Files()
    rclpy.spin(image_saver_node)
    image_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
