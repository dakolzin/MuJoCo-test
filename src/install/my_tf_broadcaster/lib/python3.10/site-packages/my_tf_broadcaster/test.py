#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import math
import time
import numpy as np
import threading
import socket
import argparse

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import tf2_ros
from geometry_msgs.msg import TransformStamped

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return [w, x, y, z]

def rotation_matrix_to_quaternion(R):
    R = np.array(R).reshape(3, 3)
    trace = np.trace(R)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (R[2,1] - R[1,2]) / s
        y = (R[0,2] - R[2,0]) / s
        z = (R[1,0] - R[0,1]) / s
    elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
        s = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        w = (R[2,1] - R[1,2]) / s
        x = 0.25 * s
        y = (R[0,1] + R[1,0]) / s
        z = (R[0,2] + R[2,0]) / s
    elif R[1,1] > R[2,2]:
        s = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        w = (R[0,2] - R[2,0]) / s
        x = (R[0,1] + R[1,0]) / s
        y = 0.25 * s
        z = (R[1,2] + R[2,1]) / s
    else:
        s = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        w = (R[1,0] - R[0,1]) / s
        x = (R[0,2] + R[2,0]) / s
        y = (R[1,2] + R[2,1]) / s
        z = 0.25 * s
    return [w, x, y, z]

class TfBroadcasterOffline(Node):
    def __init__(self, json_dir="saved_data"):
        super().__init__('tf_broadcaster_offline')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        half_angle = -math.pi / 4
        self.q_minus_90y = [math.cos(half_angle), 0.0, math.sin(half_angle), 0.0]
        self.json_dir = json_dir
        self.json_files = self._find_json_files(json_dir)
        self.file_index = 0
        self.grasp_index = 0
        self.grasps_for_file = []
        self.current_grasp_17 = None
        self.lookup_sent_once = False

        self.timer = self.create_timer(0.02, self.process_timer_callback)

        self.tcp_host = '127.0.0.1'
        self.tcp_port = 54321
        self.sock = None
        self._connect_persistent_socket()

        self.srv = self.create_service(Empty, 'next_grasp', self.next_grasp_callback)
        self.get_logger().info("Сервис 'next_grasp' запущен. Вызовите его для переключения захвата.")

    def _connect_persistent_socket(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((self.tcp_host, self.tcp_port))
            self.sock = s
            self.get_logger().info(f"[Socket] Подключены к {self.tcp_host}:{self.tcp_port}")
        except Exception as e:
            self.sock = None
            self.get_logger().error(f"[Socket] Не удалось подключиться: {e}")

    def _find_json_files(self, folder):
        all_files = []
        if not os.path.isdir(folder):
            self.get_logger().error(f"Папка '{folder}' не существует!")
            return all_files
        for fname in os.listdir(folder):
            if fname.startswith("grasp_data_000001") and fname.endswith(".json"):
                all_files.append(os.path.join(folder, fname))
        all_files.sort()
        return all_files

    def load_current_file(self):
        if self.file_index >= len(self.json_files):
            return
        path = self.json_files[self.file_index]
        self.get_logger().info(f"Читаем файл: {path}")
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        arr = data.get("grasps_17", [])
        if not isinstance(arr, list) or len(arr) == 0:
            self.get_logger().warn(f"Файл {path} не содержит grasps_17")
            self.file_index += 1
            return
        self.grasps_for_file = arr
        self.grasp_index = 0
        self.get_logger().info(f"Найдено {len(self.grasps_for_file)} захватов.")

    def process_timer_callback(self):
        if self.file_index >= len(self.json_files):
            self.get_logger().info("Все JSON-файлы обработаны. Останавливаем ноду.")
            self.timer.cancel()
            return

        if not self.grasps_for_file:
            self.load_current_file()
            if not self.grasps_for_file:
                return

        if self.current_grasp_17 is None:
            if self.grasp_index < len(self.grasps_for_file):
                self.current_grasp_17 = self.grasps_for_file[self.grasp_index]
                self.get_logger().info(f"Захват #{self.grasp_index}, (файл {self.file_index}).")
                self.lookup_sent_once = False
            else:
                self.file_index += 1
                self.grasps_for_file = []
                self.current_grasp_17 = None
                return

        if self.current_grasp_17 is not None:
            self.broadcast_tf(self.current_grasp_17)
            if not self.lookup_sent_once:
                try:
                    # Пытаемся получить трансформацию от базового фрейма до ptg,
                    # где базовый фрейм задаётся в URDF
                    stamped = self.buffer.lookup_transform("base", "ptg", rclpy.time.Time())
                    tr = stamped.transform.translation
                    rot = stamped.transform.rotation
                    trans_bptg = [tr.x, tr.y, tr.z]
                    q_bptg = [rot.x, rot.y, rot.z, rot.w]
                    self.get_logger().info(f"[TCP] base->ptg trans = {trans_bptg}")
                    self.get_logger().info(f"[TCP] base->ptg rot   = {q_bptg}")
                    self.send_transform_via_tcp(trans_bptg, q_bptg)
                    self.lookup_sent_once = True
                except Exception:
                    pass

    def next_grasp_callback(self, request, response):
        self.grasp_index += 1
        if self.grasp_index < len(self.grasps_for_file):
            self.current_grasp_17 = self.grasps_for_file[self.grasp_index]
            self.lookup_sent_once = False
            self.get_logger().info(f"Переключение на захват #{self.grasp_index}. Публикую TF немедленно.")
            def burst_publish():
                for _ in range(5):
                    self.broadcast_tf(self.current_grasp_17)
                    time.sleep(0.01)
            threading.Thread(target=burst_publish, daemon=True).start()
        else:
            self.get_logger().warn("Достигнут конец списка захватов.")
        return response

    def broadcast_tf(self, grasp_17):
        now = self.get_clock().now().to_msg()

        # Публикуем только преобразования относительно фрейма camera,
        # так как фреймы base и camera задаются через URDF
        # camera->grasp
        R_vals = grasp_17[4:13]
        T_vals = grasp_17[13:16]
        R_g = np.array(R_vals).reshape(3, 3)
        q_g = rotation_matrix_to_quaternion(R_g)
        t_cg = TransformStamped()
        t_cg.header.stamp = now
        t_cg.header.frame_id = "camera"
        t_cg.child_frame_id = "grasp"
        t_cg.transform.translation.x = float(T_vals[0])
        t_cg.transform.translation.y = float(T_vals[1])
        t_cg.transform.translation.z = float(T_vals[2])
        t_cg.transform.rotation.x = q_g[1]
        t_cg.transform.rotation.y = q_g[2]
        t_cg.transform.rotation.z = q_g[3]
        t_cg.transform.rotation.w = q_g[0]
        self.broadcaster.sendTransform(t_cg)

        # camera->ptg (дополнительный поворот -90° вокруг Y)
        q_ptg = quaternion_multiply(q_g, self.q_minus_90y)
        t_cptg = TransformStamped()
        t_cptg.header.stamp = now
        t_cptg.header.frame_id = "camera"
        t_cptg.child_frame_id = "ptg"
        t_cptg.transform.translation.x = float(T_vals[0])
        t_cptg.transform.translation.y = float(T_vals[1])
        t_cptg.transform.translation.z = float(T_vals[2])
        t_cptg.transform.rotation.x = q_ptg[1]
        t_cptg.transform.rotation.y = q_ptg[2]
        t_cptg.transform.rotation.z = q_ptg[3]
        t_cptg.transform.rotation.w = q_ptg[0]
        self.broadcaster.sendTransform(t_cptg)

    def send_transform_via_tcp(self, translation, rotation):
        if self.sock is None:
            self._connect_persistent_socket()
            if self.sock is None:
                self.get_logger().error("[Socket] Нет подключения.")
                return
        transform_dict = {"translation": translation, "rotation": rotation}
        data = json.dumps(transform_dict) + "\n"
        try:
            self.sock.sendall(data.encode('utf-8'))
            self.get_logger().info(f"[TCP] base->ptg => {self.tcp_host}:{self.tcp_port}")
        except Exception as e:
            self.get_logger().error(f"[TCP] Ошибка: {e}")
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

def main(args=None):
    parser = argparse.ArgumentParser(
        description="TF Broadcaster Offline с выбором директории JSON файла")
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--diff', action='store_true',
                       help="Использовать директорию saved_data/graspnet/diff")
    group.add_argument('--vase', action='store_true',
                       help="Использовать директорию saved_data/graspnet/vase")
    parsed_args, unknown = parser.parse_known_args()

    if parsed_args.diff:
        json_dir = "my_tf_broadcaster/saved_data/graspnet/diff"
    elif parsed_args.vase:
        json_dir = "my_tf_broadcaster/saved_data/graspnet/vase"
    else:
        json_dir = "my_tf_broadcaster/my_tf_broadcaster"

    print(f"[INFO] Используем путь к JSON-файлам: {json_dir}")

    rclpy.init(args=args)
    node = TfBroadcasterOffline(json_dir=json_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
