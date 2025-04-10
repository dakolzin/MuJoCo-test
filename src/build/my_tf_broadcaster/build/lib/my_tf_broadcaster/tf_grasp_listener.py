#tf_grasp_listener.py

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import TransformStamped

import numpy as np
import math
import socket
import threading
import json

"""
Суть работы скрипта:
- Реализует ROS2-ноду, которая поднимает TCP-сервер на порту 12345 и ожидает получения JSON-массива из 17 чисел.
- Из полученного массива извлекает данные для формирования матрицы вращения (9 чисел) и вектора переноса (3 числа) захвата (grasp).
- Преобразует матрицу вращения в кватернион с помощью функции rotation_matrix_to_quaternion.
- Публикует TF-преобразование, связывающее координатную систему камеры (cam_frame) с системой grasp, позволяя другим узлам ROS использовать эту информацию.
"""

def rotation_matrix_to_quaternion(R):
    """
    Преобразует 3x3-матрицу вращения (numpy) в кватернион [w, x, y, z].
    """
    trace = np.trace(R)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    return [w, x, y, z]


class GraspTfBroadcaster(Node):
    """
    Нода, которая:
    1) Поднимает TCP-сервер (порт 12345).
    2) Ждёт JSON-массив из 17 float-чисел.
    3) Формирует TF: cam_frame -> grasp.
    """
    def __init__(self):
        super().__init__('grasp_tf_broadcaster')

        # Параметр, чтобы при желании переопределять имя кадра камеры:
        self.declare_parameter('camera_frame', 'cam_frame')
        self.camera_frame = self.get_parameter('camera_frame').value

        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Здесь храним последние полученные данные (array of float)
        self.grasp_data = None
        self.lock = threading.Lock()

        # Запускаем сервер в отдельном потоке
        self.server_thread = threading.Thread(target=self.tcp_server, daemon=True)
        self.server_thread.start()

        # Таймер для периодической публикации TF
        self.timer = self.create_timer(0.1, self.broadcast_grasp_tf)  # 10 Гц

    def tcp_server(self):
        """
        TCP-сервер на порту 12345. Принимает JSON (list из 17 float),
        сохраняет в self.grasp_data.
        """
        host = ''
        port = 12345  # меняйте, если нужно
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.listen(1)
        self.get_logger().info(f"TCP-сервер ждёт подключений на :{port}...")

        while rclpy.ok():
            # Ожидаем клиента
            conn, addr = s.accept()
            self.get_logger().info(f"Подключился клиент: {addr}")
            data_buffer = b''

            with conn:
                while True:
                    packet = conn.recv(4096)
                    if not packet:
                        break  # клиент закрыл соединение
                    data_buffer += packet

                    # Пытаемся распарсить как JSON (возможно, придёт несколько частей)
                    try:
                        decoded_str = data_buffer.decode('utf-8')
                        parsed = json.loads(decoded_str)

                        if isinstance(parsed, list):
                            # Проверяем, не 17 ли там чисел
                            if len(parsed) == 17 and all(isinstance(x, (int, float)) for x in parsed):
                                with self.lock:
                                    self.grasp_data = parsed
                                self.get_logger().info(f"Получены данные: {parsed}")
                                # Можно очистить буфер после корректного приёма
                                data_buffer = b''
                            else:
                                self.get_logger().warn(
                                    f"Формат JSON: ожидался список из 17 float, а пришло: {len(parsed)}"
                                )
                        else:
                            self.get_logger().warn("Ожидался список (list) для grasp_data")

                    except json.JSONDecodeError:
                        # Значит, JSON пока неполный (читаем дальше)
                        pass

                self.get_logger().info("Клиент отключился.")

    def broadcast_grasp_tf(self):
        """
        Раз в 0.1 с формируем Transform cam_frame->grasp, если есть данные.
        """
        if self.grasp_data is None:
            return

        with self.lock:
            arr = self.grasp_data[:]  # копия, чтоб не мешать другому потоку

        # Предположим, что:
        # arr[4:13] -> 9 чисел для матрицы вращения R (3x3)
        # arr[13:16] -> 3 числа для вектора переноса XYZ

        R_vals = arr[4:13]
        T_vals = arr[13:16]
        R = np.array(R_vals).reshape((3, 3))
        t = np.array(T_vals)

        # Преобразуем R -> кватернион [w, x, y, z]
        q = rotation_matrix_to_quaternion(R)

        # Формируем TF
        now = self.get_clock().now().to_msg()
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.camera_frame
        tf_msg.child_frame_id = 'grasp'

        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])

        tf_msg.transform.rotation.w = q[0]
        tf_msg.transform.rotation.x = q[1]
        tf_msg.transform.rotation.y = q[2]
        tf_msg.transform.rotation.z = q[3]

        # Публикуем
        self.broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GraspTfBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
