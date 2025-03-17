#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
import tf_transformations as tft
from geometry_msgs.msg import TransformStamped
import tf2_ros

class ArucoMarkerTFPublisher(Node):
    def __init__(self):
        super().__init__('aruco_marker_tf_publisher')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Параметры камеры (RealSense) и ArUco
        self.camera_matrix = np.array([[382.8452, 0.0, 318.5836],
                                       [0.0, 382.8452, 232.4519],
                                       [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        self.marker_length = 0.053  # длина стороны маркера в метрах
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Инициализация RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.get_logger().info("Запущен ArucoMarkerTFPublisher")

        # Таймер для периодической обработки кадров
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length,
                                                                  self.camera_matrix, self.dist_coeffs)
            for i, marker_id in enumerate(ids.flatten()):
                # Извлекаем векторы поворота и трансляции
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                # Преобразуем rvec в матрицу поворота и затем в кватернион
                R, _ = cv2.Rodrigues(rvec)
                T = np.eye(4)
                T[:3, :3] = R
                quat = tft.quaternion_from_matrix(T)
                
                # Формируем сообщение TF
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "cam_frame"       # родительский фрейм
                t.child_frame_id = f"marker_{marker_id}"    # фрейм маркера
                t.transform.translation.x = float(tvec[0])
                t.transform.translation.y = float(tvec[1])
                t.transform.translation.z = float(tvec[2])
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]
                
                self.tf_broadcaster.sendTransform(t)
                self.get_logger().info(f"Опубликован TF для маркера {marker_id}")
                
                # Рисуем оси и контур для визуализации
                cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)
            cv2.aruco.drawDetectedMarkers(frame, corners)
        
        cv2.imshow('RealSense Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.shutdown()

    def shutdown(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()

if __name__ == "__main__":
    main()
