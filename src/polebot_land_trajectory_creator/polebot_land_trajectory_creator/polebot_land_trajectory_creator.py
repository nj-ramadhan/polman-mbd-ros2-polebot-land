#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

class TrajectoryCreator(Node):
    def __init__(self):
        super().__init__('line_creator')
        self.subscription = self.create_subscription(
            Point,
            '/coordinates',  # Topik dari coordinate_publisher
            self.listener_callback,
            10
        )

        # 1. Buat sebuah profil QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1, # Simpan 1 pesan terakhir
            durability=DurabilityPolicy.TRANSIENT_LOCAL # Ini kuncinya!
        )

        self.publisher_ = self.create_publisher(Path, '/line_trajectory',  qos_profile)  # Topik trajektori
        self.coordinates = []
        self.get_logger().info("LineCreator node started and subscribed to coordinates.")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received coordinate: x={msg.x}, y={msg.y}, z={msg.z}")
        # Menyimpan titik yang diterima dari coordinate_publisher
        self.coordinates.append([msg.x, msg.y, msg.z])

        if len(self.coordinates) == 31:  # Jika sudah ada 31 titik
            self.publish_trajectory()
            self.get_logger().info(f"All coordinates received: {self.coordinates}")

    def publish_trajectory(self):
        """Publish the trajectory as a Path message."""
        path_msg = Path()
        now = self.get_clock().now().to_msg()
        path_msg.header.stamp = now
        path_msg.header.frame_id = "odom"
        
        # Add poses with proper orientation
        for i, coord in enumerate(self.coordinates):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = now
            pose_stamped.header.frame_id = "odom"
            
            # Position
            pose_stamped.pose.position.x = float(coord[0])
            pose_stamped.pose.position.y = float(coord[1])
            pose_stamped.pose.position.z = float(coord[2])
            
            # Calculate orientation to point to next waypoint
            if i < len(self.coordinates) - 1:
                next_coord = self.coordinates[i + 1]
                dx = next_coord[0] - coord[0]
                dy = next_coord[1] - coord[1]
                theta = math.atan2(dy, dx)
                # Convert theta to quaternion
                pose_stamped.pose.orientation.w = math.cos(theta/2)
                pose_stamped.pose.orientation.z = math.sin(theta/2)
            else:
                # Last point keeps the same orientation as the previous one
                pose_stamped.pose.orientation.w = 1.0
                pose_stamped.pose.orientation.z = 0.0
            
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            path_msg.poses.append(pose_stamped)

        # Publish repeatedly to ensure delivery
        for _ in range(3):  # Publish 3 times to ensure reception
            self.publisher_.publish(path_msg)
            self.get_logger().info(
                f'Published path with {len(path_msg.poses)} poses. '
                f'Start: ({self.coordinates[0][0]:.2f}, {self.coordinates[0][1]:.2f}), '
                f'End: ({self.coordinates[-1][0]:.2f}, {self.coordinates[-1][1]:.2f})'
            )

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryCreator()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point
# from std_msgs.msg import Float32MultiArray

# class LineCreator(Node):
#     def __init__(self):
#         super().__init__('line_creator')
#         self.subscription = self.create_subscription(
#             Point,
#             '/coordinates',  # Topik dari coordinate_publisher
#             self.listener_callback,
#             10
#         )
#         self.publisher_ = self.create_publisher(Float32MultiArray, '/line_trajectory', 10)  # Topik garis
#         self.coordinates = []
#         self.get_logger().info("LineCreator node started and subscribed to coordinates.")


#     def listener_callback(self, msg):
#         self.get_logger().info(f"Received coordinate: x={msg.x}, y={msg.y}, z={msg.z}")
#         # Menyimpan titik yang diterima dari coordinate_publisher
#         self.coordinates.append([msg.x, msg.y, msg.z])
        
#         if len(self.coordinates) == 21:  # Jika sudah ada 21 titik
#             self.publish_trajectory()
#             self.get_logger().info(f"All coordinates received: {self.coordinates}")

#     def publish_trajectory(self):
#         # Mengubah koordinat menjadi array float dan mempublikasikan
#         trajectory_msg = Float32MultiArray()
#         trajectory_msg.data = [coord for point in self.coordinates for coord in point]
        
#         # Publikasikan garis (trajectory)
#         self.publisher_.publish(trajectory_msg)
#         self.get_logger().info(f'Publishing trajectory: {trajectory_msg.data}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = LineCreator()
#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Point, PoseStamped
# from nav_msgs.msg import Path
# from builtin_interfaces.msg import Time
# from std_msgs.msg import Int32

# class LineCreator(Node):
#     def __init__(self):
#         super().__init__('line_creator')
#         self.subscription = self.create_subscription(
#             Point,
#             '/coordinates',  # Topik dari coordinate_publisher
#             self.listener_callback,
#             10
#         )
#         self.publisher_ = self.create_publisher(Path, '/line_trajectory', 100)  # Topik trajektori
#         self.publisher_RStart = self.create_publisher(Int32, '/start_cmd', 100)  # Topik start robot
#         self.coordinates = []
#         self.get_logger().info("LineCreator node started and subscribed to coordinates.")

#     def listener_callback(self, msg):
#         self.get_logger().info(f"Received coordinate: x={msg.x}, y={msg.y}, z={msg.z}")
#         # Menyimpan titik yang diterima dari coordinate_publisher
#         self.coordinates.append([msg.x, msg.y, msg.z])

#         if len(self.coordinates) == 21:  # Jika sudah ada 31 titik
#             self.publish_trajectory()
#             self.publish_cmd()
#             self.get_logger().info(f"All coordinates received: {self.coordinates}")

#     def publish_trajectory(self):
#         # Membuat objek Path
#         path_msg = Path()
#         path_msg.header.stamp = self.get_clock().now().to_msg()  # Timestamp saat ini
#         path_msg.header.frame_id = "odom"  # Frame referensi, bisa disesuaikan
        
#         # Menambahkan PoseStamped untuk setiap titik dalam koordinat
#         for coord in self.coordinates:
#             pose_stamped = PoseStamped()
#             pose_stamped.header.stamp = self.get_clock().now().to_msg()  # Timestamp untuk setiap pose
#             pose_stamped.header.frame_id = "odom"  # Frame referensi untuk pose
#             pose_stamped.pose.position.x = coord[0]
#             pose_stamped.pose.position.y = coord[1]
#             pose_stamped.pose.position.z = coord[2]
#             pose_stamped.pose.orientation.x = 0.0  # Orientasi x
#             pose_stamped.pose.orientation.y = 0.0  # Orientasi y
#             pose_stamped.pose.orientation.z = 0.0  # Orientasi z
#             pose_stamped.pose.orientation.w = 1.0  # Orientasi w (mengarah ke arah yang benar)
#             path_msg.poses.append(pose_stamped)

#         # Publikasikan trajektori (path)
#         self.publisher_.publish(path_msg)   # ini buat ngirim data "trajektori beres" ke smc?
#         self.get_logger().info(f'Publishing path: {len(path_msg.poses)} poses')

#     def publish_cmd(self):
#         # Membuat objek Bool
#         path_msga = Int32()
#         path_msga.data = 1
#         self.publisher_RStart.publish(path_msga)

# def main(args=None):
#     rclpy.init(args=args)
#     node = LineCreator()
#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
