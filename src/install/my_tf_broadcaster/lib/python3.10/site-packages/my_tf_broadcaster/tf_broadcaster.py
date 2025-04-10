#tf_broadcaster.py

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
import socket
import json
import threading
import numpy as np

"""
Суть работы скрипта:
- Реализует узел ROS2, который транслирует (broadcast) TF-преобразования между различными координатными системами.
- Получает данные по TCP (ожидается список из 17 чисел, содержащих матрицу вращения и позицию объекта "grasp").
- Вычисляет преобразования:
    • base -> camera: фиксированное преобразование, задающее положение и ориентацию камеры.
    • camera -> grasp: преобразование, полученное из входящих данных, описывающее положение и ориентацию объекта относительно камеры.
    • camera -> ptg: аналогично grasp, но с дополнительным поворотом на 180° вокруг локальной оси Y (flip).
- Публикует эти TF-преобразования в ROS для использования другими узлами.
- Отправляет составное преобразование (base -> ptg) по TCP на клиентский порт.
- Использует функции для работы с кватернионами и матрицами вращения для корректного вычисления преобразований.
"""

def quaternion_multiply(q1, q2):
    """Умножение кватернионов (w, x, y, z). Возвращает q1 * q2."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return [w, x, y, z]

def rotation_matrix_to_quaternion(R):
    """Преобразует 3x3-матрицу вращения (row-major) в кватернион [w, x, y, z]."""
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

def quaternion_to_rotation_matrix(q):
    """Преобразует кватернион [w, x, y, z] в матрицу 3x3 (row-major)."""
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [  2*(x*y + z*w), 1-2*(x*x+z*z),   2*(y*z - x*w)],
        [  2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ])

class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transforms)  # 10 Гц
        self.grasp_data = None
        self.composite_sent = False  # Флаг, что мы отправили трансформацию в сокет

        # Поток для приёма 17 чисел
        self.socket_thread = threading.Thread(target=self.receive_socket_data, daemon=True)
        self.socket_thread.start()

        # Кватернион вращения вокруг Y на 180° (локально)
        # Угол pi, ось Y => q = [cos(pi/2), 0, sin(pi/2), 0] = [0, 0, 1, 0].
        self.q_flip_z_around_y = [0.0, 0.0, 1.0, 0.0]

    def receive_socket_data(self):
        HOST = ''
        PORT = 12345
        self.get_logger().info(f"Ожидание подключения на {HOST}:{PORT}...")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT))
            s.listen(1)
            conn, addr = s.accept()
            self.get_logger().info(f"Подключено: {addr}")
            data = b""
            with conn:
                while True:
                    packet = conn.recv(4096)
                    if not packet:
                        break
                    data += packet
                    try:
                        decoded = data.decode('utf-8')
                        received_array = json.loads(decoded)
                        if isinstance(received_array, list):
                            # Ждём список из 17 чисел
                            if len(received_array) == 17 and all(isinstance(x, (int, float)) for x in received_array):
                                self.grasp_data = received_array
                                self.get_logger().info(f"Получены 17 чисел: {received_array}")
                                break
                            elif (
                                len(received_array) > 0 and
                                isinstance(received_array[0], list) and
                                len(received_array[0]) == 17
                            ):
                                self.grasp_data = received_array[0]
                                self.get_logger().info(f"Получены данные (первый из списка): {self.grasp_data}")
                                break
                            else:
                                self.get_logger().error("Неподдерживаемый формат JSON (ожидается 17 чисел)")
                                break
                        else:
                            self.get_logger().error("Неподдерживаемый формат JSON (не список)")
                            break
                    except json.JSONDecodeError:
                        continue
                self.get_logger().info("Отключаем клиента (закрываем сокет).")
                try:
                    conn.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                conn.close()
        self.get_logger().info("Серверный сокет закрыт. receive_socket_data завершена.")

    def compute_base_to_grasp_transform(self):
        """
        base->camera умножить на camera->grasp.
        Возвращаем словарь {translation, rotation}.
        """
        camera_pos = np.array([0.0, 0.3, 0.26])
        camera_quat = [0.44499672, 0.54952518, 0.54952518, 0.44499672]

        # camera->grasp берём из grasp_data
        if self.grasp_data is not None and len(self.grasp_data) == 17:
            trans_cg = np.array(self.grasp_data[13:16])
            R_cg = np.array(self.grasp_data[4:13]).reshape(3,3)
            q_cg = rotation_matrix_to_quaternion(R_cg)
        else:
            trans_cg = np.array([0.0, 0.0, 0.0])
            q_cg = [1.0, 0.0, 0.0, 0.0]

        # base->camera
        # (для примера домножим camera_quat на [0,0,0,1], можно и сразу брать camera_quat)
        q_fix = [0,0,0,1]
        q_bc = quaternion_multiply(camera_quat, q_fix)

        # Составим base->grasp
        R_bc = quaternion_to_rotation_matrix(q_bc)
        trans_bg = camera_pos + R_bc.dot(trans_cg)
        q_bg = quaternion_multiply(q_bc, q_cg)

        return {
            "translation": trans_bg.tolist(),
            "rotation": q_bg
        }

    def compute_base_to_ptg_transform(self):
        """
        base->camera умножить на camera->ptg,
        где camera->ptg имеет ту же позицию, что camera->grasp,
        но повёрнут на 180° вокруг локальной оси Y (флип Z).
        """
        # Сначала получаем camera->grasp
        if self.grasp_data is not None and len(self.grasp_data) == 17:
            R_cg = np.array(self.grasp_data[4:13]).reshape(3,3)
            q_cg = rotation_matrix_to_quaternion(R_cg)
            trans_cg = np.array(self.grasp_data[13:16])
        else:
            q_cg = [1.0, 0.0, 0.0, 0.0]
            trans_cg = np.array([0.0, 0.0, 0.0])

        # Поворачиваем вокруг ЛОКАЛЬНОЙ Y схвата на 180°, умножая справа
        # ptg = grasp * (поворот_на_180_вокруг_Y).
        q_ptg = quaternion_multiply(q_cg, self.q_flip_z_around_y)

        # Теперь base->camera тот же, что и раньше
        camera_pos = np.array([0.0, 0.3, 0.26])
        camera_quat = [0.44499672, 0.54952518, 0.54952518, 0.44499672]
        q_bc = quaternion_multiply(camera_quat, [0,0,0,1])

        R_bc = quaternion_to_rotation_matrix(q_bc)
        trans_bptg = camera_pos + R_bc.dot(trans_cg)
        q_bptg = quaternion_multiply(q_bc, q_ptg)

        return {
            "translation": trans_bptg.tolist(),
            "rotation": q_bptg
        }

    def send_composite_transform(self, transform, frame_name="PTG"):
        """Отправить base->PTG (или base->grasp) по TCP на порт 54321."""
        HOST = '127.0.0.1'
        PORT = 54321
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.connect((HOST, PORT))
                data = json.dumps(transform).encode('utf-8')
                sock.sendall(data)
                self.get_logger().info(f"Отправлена трансформация base->{frame_name} на клиент")
        except Exception as e:
            self.get_logger().error(f"Ошибка при отправке трансформации: {e}")

    def broadcast_transforms(self):
        now = self.get_clock().now().to_msg()

        # -------- base->camera --------
        t_base_to_camera = TransformStamped()
        t_base_to_camera.header.stamp = now
        t_base_to_camera.header.frame_id = "base"
        t_base_to_camera.child_frame_id = "camera"
        camera_pos = [0.0, 0.3, 0.26]
        camera_quat = [0.44499672, 0.54952518, 0.54952518, 0.44499672]
        new_camera_quat = quaternion_multiply(camera_quat, [0, 0, 0, 1])
        t_base_to_camera.transform.translation.x = camera_pos[0]
        t_base_to_camera.transform.translation.y = camera_pos[1]
        t_base_to_camera.transform.translation.z = camera_pos[2]
        t_base_to_camera.transform.rotation.x = new_camera_quat[1]
        t_base_to_camera.transform.rotation.y = new_camera_quat[2]
        t_base_to_camera.transform.rotation.z = new_camera_quat[3]
        t_base_to_camera.transform.rotation.w = new_camera_quat[0]
        self.broadcaster.sendTransform(t_base_to_camera)

        # -------- camera->grasp (как было) --------
        t_camera_to_grasp = TransformStamped()
        t_camera_to_grasp.header.stamp = now
        t_camera_to_grasp.header.frame_id = "camera"
        t_camera_to_grasp.child_frame_id = "grasp"

        if self.grasp_data is not None and len(self.grasp_data) == 17:
            R_grasp = np.array(self.grasp_data[4:13]).reshape(3,3)
            grasp_quat = rotation_matrix_to_quaternion(R_grasp)
            translation = self.grasp_data[13:16]
            t_camera_to_grasp.transform.translation.x = translation[0]
            t_camera_to_grasp.transform.translation.y = translation[1]
            t_camera_to_grasp.transform.translation.z = translation[2]
            t_camera_to_grasp.transform.rotation.x = grasp_quat[1]
            t_camera_to_grasp.transform.rotation.y = grasp_quat[2]
            t_camera_to_grasp.transform.rotation.z = grasp_quat[3]
            t_camera_to_grasp.transform.rotation.w = grasp_quat[0]
        else:
            t_camera_to_grasp.transform.translation.x = 0.0
            t_camera_to_grasp.transform.translation.y = 0.0
            t_camera_to_grasp.transform.translation.z = 0.0
            t_camera_to_grasp.transform.rotation.x = 0.0
            t_camera_to_grasp.transform.rotation.y = 0.0
            t_camera_to_grasp.transform.rotation.z = 0.0
            t_camera_to_grasp.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t_camera_to_grasp)

        # -------- camera->ptg (добавляем новый) --------
        t_camera_to_ptg = TransformStamped()
        t_camera_to_ptg.header.stamp = now
        t_camera_to_ptg.header.frame_id = "camera"
        t_camera_to_ptg.child_frame_id = "ptg"

        if self.grasp_data is not None and len(self.grasp_data) == 17:
            R_grasp = np.array(self.grasp_data[4:13]).reshape(3,3)
            q_grasp = rotation_matrix_to_quaternion(R_grasp)
            # Поворачиваем вокруг локальной Y на 180°
            q_ptg = quaternion_multiply(q_grasp, self.q_flip_z_around_y)
            translation = self.grasp_data[13:16]

            t_camera_to_ptg.transform.translation.x = translation[0]
            t_camera_to_ptg.transform.translation.y = translation[1]
            t_camera_to_ptg.transform.translation.z = translation[2]
            t_camera_to_ptg.transform.rotation.x = q_ptg[1]
            t_camera_to_ptg.transform.rotation.y = q_ptg[2]
            t_camera_to_ptg.transform.rotation.z = q_ptg[3]
            t_camera_to_ptg.transform.rotation.w = q_ptg[0]
        else:
            t_camera_to_ptg.transform.translation.x = 0.0
            t_camera_to_ptg.transform.translation.y = 0.0
            t_camera_to_ptg.transform.translation.z = 0.0
            t_camera_to_ptg.transform.rotation.x = 0.0
            t_camera_to_ptg.transform.rotation.y = 0.0
            t_camera_to_ptg.transform.rotation.z = 0.0
            t_camera_to_ptg.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t_camera_to_ptg)

        # -------- Отправка base->ptg вместо base->grasp в сокет --------
        if not self.composite_sent and self.grasp_data is not None:
            ptg_transform = self.compute_base_to_ptg_transform()
            threading.Thread(
                target=self.send_composite_transform, 
                args=(ptg_transform,"PTG"), 
                daemon=True
            ).start()
            self.composite_sent = True

def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
