#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import math
import numpy as np

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

import socket
import threading

"""
Основные изменения:
- Вместо +90° вокруг Y сделан поворот на -90° (q_minus_90y).
- При отправке в TCP теперь рассчитываем и отправляем base->ptg, а не base->grasp.
"""

def quaternion_multiply(q1, q2):
    """
    Умножение кватернионов (w, x, y, z).
    Возвращает q1 * q2.
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return [w, x, y, z]

def rotation_matrix_to_quaternion(R):
    """
    Преобразует матрицу 3x3 в кватернион [w, x, y, z].
    """
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

        # Позиция/ориентация камеры (base->camera)
        self.camera_pos = np.array([0.0, 0.3, 0.26])
        self.camera_quat = [0.44499672, 0.54952518, 0.54952518, 0.44499672]

        # Кватернион для -90° вокруг оси Y:
        # Угол = -90° => -π/2, half_angle = -π/4
        half_angle = -math.pi / 4
        self.q_minus_90y = [
            math.cos(half_angle),  # w
            0.0,                   # x
            math.sin(half_angle),  # y
            0.0                    # z
        ]
        # (Будет применен к q_grasp: q_ptg = q_grasp * q_minus_90y)

        self.json_dir = json_dir
        self.json_files = self._find_json_files(json_dir)
        self.file_index = 0
        self.grasp_index = 0
        self.grasps_for_file = []

        self.current_grasp_17 = None
        self.waiting_input = False

        # Таймер на 10 Гц
        self.timer = self.create_timer(0.1, self.process_timer_callback)

        # Параметры TCP
        self.tcp_host = '127.0.0.1'
        self.tcp_port = 54321
        self.sock = None
        self._connect_persistent_socket()

    def _connect_persistent_socket(self):
        """
        Разово пытаемся подключиться к tcp_host:tcp_port,
        храним сокет в self.sock без закрытия после sendall.
        """
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((self.tcp_host, self.tcp_port))
            self.sock = s
            self.get_logger().info(f"[Socket] Успешно подключены к {self.tcp_host}:{self.tcp_port}")
        except Exception as e:
            self.sock = None
            self.get_logger().error(f"[Socket] Не удалось подключиться: {e}")

    def _find_json_files(self, folder):
        all_files = []
        if not os.path.isdir(folder):
            self.get_logger().error(f"Папка '{folder}' не существует!")
            return all_files

        for fname in os.listdir(folder):
            if fname.startswith("grasp_data_") and fname.endswith(".json"):
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
            self.get_logger().warn(f"Файл {path} не содержит grasps_17 или оно пустое.")
            self.file_index += 1
            return

        self.grasps_for_file = arr
        self.grasp_index = 0
        self.get_logger().info(f"Найдено {len(self.grasps_for_file)} захватов в {path}.")

    def process_timer_callback(self):
        if self.file_index >= len(self.json_files):
            self.get_logger().info("Все JSON-файлы обработаны. Останавливаем ноду.")
            self.timer.cancel()
            return

        if not self.grasps_for_file:
            self.load_current_file()
            if not self.grasps_for_file:
                return

        if self.current_grasp_17 is None and not self.waiting_input:
            if self.grasp_index < len(self.grasps_for_file):
                self.current_grasp_17 = self.grasps_for_file[self.grasp_index]
                self.get_logger().info(f"Переходим к захвату #{self.grasp_index} (файл {self.file_index})")
                self.get_logger().info("Нажмите Enter, чтобы перейти к следующему захвату.")
                self.send_base_ptg_transform(self.current_grasp_17)  # Вместо base->grasp отправляем base->ptg
                self.waiting_input = True
                threading.Thread(target=self.wait_for_enter, daemon=True).start()
            else:
                self.file_index += 1
                self.grasps_for_file = []
                self.current_grasp_17 = None
                return

        if self.current_grasp_17 is not None:
            self.broadcast_tf(self.current_grasp_17)

    def wait_for_enter(self):
        input()
        self.grasp_index += 1
        self.current_grasp_17 = None
        self.waiting_input = False

    def broadcast_tf(self, grasp_17):
        """
        Публикуем в TF:
         - base->camera
         - camera->grasp
         - camera->ptg (захват, повернутый на -90° вокруг Y).
        """
        now = self.get_clock().now().to_msg()

        # 1) base->camera
        t_bc = TransformStamped()
        t_bc.header.stamp = now
        t_bc.header.frame_id = "base"
        t_bc.child_frame_id = "camera"

        pos = self.camera_pos
        quat = self.camera_quat
        # Множим на [0,0,0,1], если хотим
        new_q = quaternion_multiply(quat, [0,0,0,1])

        t_bc.transform.translation.x = float(pos[0])
        t_bc.transform.translation.y = float(pos[1])
        t_bc.transform.translation.z = float(pos[2])
        t_bc.transform.rotation.x = new_q[1]
        t_bc.transform.rotation.y = new_q[2]
        t_bc.transform.rotation.z = new_q[3]
        t_bc.transform.rotation.w = new_q[0]
        self.broadcaster.sendTransform(t_bc)

        # 2) camera->grasp
        R_vals = grasp_17[4:13]
        T_vals = grasp_17[13:16]
        R_g = np.array(R_vals).reshape(3,3)
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

        # 3) camera->ptg = camera->grasp * (-90° вокруг лок. Y)
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

    def send_base_ptg_transform(self, grasp_17):
        """
        Вместо base->grasp, теперь вычисляем base->ptg.
        = (base->camera) * (camera->ptg).
        Позиция остается та же, что camera->grasp (т.к. ptg не меняет transl),
        но ориентация q_ptg = q_grasp * q_minus_90y.
        Отправляем JSON.
        """
        if self.sock is None:
            self._connect_persistent_socket()
            if self.sock is None:
                self.get_logger().error("[Socket] Нет активного соединения.")
                return

        # base->camera
        q_bc = self.camera_quat
        pos_bc = self.camera_pos
        # camera->grasp
        trans_cg = grasp_17[13:16]
        R_cg = np.array(grasp_17[4:13]).reshape(3,3)
        q_cg = rotation_matrix_to_quaternion(R_cg)

        # Считаем q_ptg
        q_ptg = quaternion_multiply(q_cg, self.q_minus_90y)

        # base->ptg:
        # Позиция (та же, что trans_cg) + R_bc.dot(trans_cg).
        # (ptg не меняет transl)
        R_bc = self.quaternion_to_rotation_matrix(q_bc)
        trans_bptg = pos_bc + R_bc.dot(trans_cg)

        # Ориентация:
        q_bptg = quaternion_multiply(q_bc, q_ptg)

        self.get_logger().info(f"[TCP] base->ptg translation = {trans_bptg}")
        self.get_logger().info(f"[TCP] base->ptg rotation = {q_bptg}")

        transform_dict = {
            "translation": trans_bptg.tolist(),
            "rotation": q_bptg
        }
        data = json.dumps(transform_dict) + "\n"
        try:
            self.sock.sendall(data.encode('utf-8'))
            self.get_logger().info(
                f"[TCP] Отправлен base->ptg ({len(data.encode('utf-8'))} байт) на {self.tcp_host}:{self.tcp_port}"
            )
        except Exception as e:
            self.get_logger().error(f"[TCP] Ошибка при отправке: {e}")
            try:
                self.sock.close()
            except:
                pass
            self.sock = None

    def quaternion_to_rotation_matrix(self, q):
        w, x, y, z = q
        return np.array([
            [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
            [2*(x*y + z*w),   1-2*(x*x+z*z),   2*(y*z - x*w)],
            [2*(x*z - y*w),   2*(y*z + x*w),   1-2*(x*x+y*y)]
        ])

def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcasterOffline(json_dir="my_tf_broadcaster/my_tf_broadcaster")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
