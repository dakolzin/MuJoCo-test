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
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import tf2_ros
from geometry_msgs.msg import TransformStamped


def quaternion_multiply(q1, q2):
    """
    Умножение кватернионов q1 * q2.
    Возвращает кватернион в виде списка [w, x, y, z].
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
    Преобразование матрицы поворота 3x3 (R) в кватернион [w, x, y, z].
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
    """
    Нода, которая:
      1. Ищет в папке json_dir JSON-файлы (формата grasp_data_000003...),
      2. Последовательно читает захваты (grasps_17),
      3. Публикует TF camera->grasp и camera->ptg (PTG - grasp с поворотом -90° вокруг Y),
      4. В отдельном потоке может отправлять через TCP трансформацию base->ptg,
         если lookup_transform("base", "ptg") успешен.
      5. Реализует сервис next_grasp (Empty) для переключения к следующему захвату.
    """

    def __init__(self, json_dir="saved_data"):
        super().__init__('tf_broadcaster_offline')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

        # Поворот -90° вокруг оси Y (кватернион)
        half_angle = -math.pi / 4
        self.q_minus_90y = [math.cos(half_angle), 0.0, math.sin(half_angle), 0.0]

        # Папка, в которой ищем JSON-файлы
        self.json_dir = json_dir

        # Составляем список подходящих файлов
        self.json_files = self._find_json_files(json_dir)
        self.file_index = 0
        self.grasp_index = 0
        self.grasps_for_file = []
        self.current_grasp_17 = None
        self.lookup_sent_once = False

        # Таймер на 20 мс
        self.timer = self.create_timer(0.02, self.process_timer_callback)

        # Подключаемся по TCP
        self.tcp_host = '127.0.0.1'
        self.tcp_port = 54321
        self.sock = None
        self._connect_persistent_socket()

        # Сервис, чтобы переключать захват
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
        """
        Ищем в папке все файлы, начинающиеся на grasp_data_000003 и заканчивающиеся .json,
        сортируем по имени.
        """
        all_files = []
        if not os.path.isdir(folder):
            self.get_logger().error(f"Папка '{folder}' не существует!")
            return all_files
        for fname in os.listdir(folder):
            if fname.startswith("grasp_data_000003") and fname.endswith(".json"):
                all_files.append(os.path.join(folder, fname))
        all_files.sort()
        return all_files

    def load_current_file(self):
        """
        Загружаем текущий файл json_files[file_index], читаем массив grasps_17.
        """
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
        """
        Главный цикл, вызывается каждые 0.02с.
        1) Если файлы закончились – останавливаемся.
        2) Если не загружены захваты – пробуем загрузить из файла.
        3) Если текущий захват не выбран – берём первый.
        4) Публикуем TF, при первом вызове пытаемся lookup_transform(base->ptg).
        """
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
                self.get_logger().info(f"Захват #{self.grasp_index} (файл {self.file_index}).")
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
                    # Пытаемся получить трансформ base->ptg
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
        """
        Сервис: переключаемся на следующий захват, если есть.
        """
        self.grasp_index += 1
        if self.grasp_index < len(self.grasps_for_file):
            self.current_grasp_17 = self.grasps_for_file[self.grasp_index]
            self.lookup_sent_once = False
            self.get_logger().info(f"Переключение на захват #{self.grasp_index}. Публикую TF немедленно.")

            # Несколько раз подряд опубликуем тот же TF, чтобы RViz успел подтянуть
            def burst_publish():
                for _ in range(5):
                    self.broadcast_tf(self.current_grasp_17)
                    time.sleep(0.01)
            threading.Thread(target=burst_publish, daemon=True).start()
        else:
            self.get_logger().warn("Достигнут конец списка захватов.")
        return response

    def broadcast_tf(self, grasp_17):
        """
        Публикуем два TF:
          camera->grasp (ориентация из grasp_17)
          camera->ptg   (то же, но дополнительно -90° вокруг Y)
        """
        now = self.get_clock().now().to_msg()

        # camera->grasp
        R_vals = grasp_17[4:13]   # rotation matrix (3x3 = 9 элементов)
        T_vals = grasp_17[13:16]  # translation (x,y,z)
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

        # camera->ptg (дополнительный поворот -90° вокруг оси Y)
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
        """
        Отправляем трансформацию (base->ptg) по TCP-сокету (JSON-строка).
        """
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
    """
    Точка входа в скрипт. Парсим известные аргументы (--sbg, --graspnet, --diff, --vase, ...)
    и игнорируем/пропускаем прочие (--ros-args и т.п.), чтобы не было ошибки.
    """
    if args is None:
        # Если вызывается без args, подставим sys.argv[1:]
        args = sys.argv[1:]

    parser = argparse.ArgumentParser(
        description="TF Broadcaster Offline для JSON-файлов grasp_data.",
        # Отключим auto_help, чтобы не конфликтовать с --ros-args (по желанию)
        add_help=True
    )

    # Флаги основных папок
    parser.add_argument('--sbg', action='store_true', help="Основная папка: sbg")
    parser.add_argument('--graspnet', action='store_true', help="Основная папка: graspnet")
    parser.add_argument('--hggd', action='store_true', help="Основная папка: hggd")

    # Флаги подпапок
    parser.add_argument('--diff', action='store_true', help="Подпапка: diff")
    parser.add_argument('--vase', action='store_true', help="Подпапка: vase")

    # parse_known_args вернёт tuple: (Namespace c известными аргументами, список неизвестных)
    parsed_args, unknown = parser.parse_known_args(args)

    # Логика определения директории:
    json_dir = "my_tf_broadcaster/saved_data/sbg"  # По умолчанию sbg
    if parsed_args.graspnet:
        json_dir = "my_tf_broadcaster/saved_data/graspnet"
    elif parsed_args.hggd:
        json_dir = "my_tf_broadcaster/saved_data/hggd"

    if parsed_args.diff:
        json_dir = os.path.join(json_dir, "diff")
    elif parsed_args.vase:
        json_dir = os.path.join(json_dir, "vase")

    print(f"[INFO] Используем путь к JSON-файлам: {json_dir}")

    # Инициируем rclpy, передавая *все* аргументы (и известные, и неизвестные),
    # чтобы ROS смог обработать свои служебные --ros-args
    rclpy.init(args=sys.argv)

    # Создаём ноду
    node = TfBroadcasterOffline(json_dir=json_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


if __name__ == "__main__":
    main()
