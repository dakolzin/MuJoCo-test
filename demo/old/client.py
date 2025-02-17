#!/usr/bin/env python3
import socket
import argparse
import json
import time
import numpy as np

def parse_args():
    parser = argparse.ArgumentParser(description="TCP Receiver для захватов GraspNet (выбор лучшего захвата за 5 секунд)")
    parser.add_argument('--ip', type=str, default='0.0.0.0',
                        help='IP адрес для прослушивания (по умолчанию 0.0.0.0)')
    parser.add_argument('--port', type=int, default=12345,
                        help='TCP порт для прослушивания (по умолчанию 12345)')
    args = parser.parse_args()
    return args

def rotation_matrix_to_quaternion(m):
    """
    Преобразует 3x3 матрицу поворота (numpy.ndarray) в кватернион в формате [w, x, y, z].
    """
    tr = m[0,0] + m[1,1] + m[2,2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2  # S=4*qw
        qw = 0.25 * S
        qx = (m[2,1] - m[1,2]) / S
        qy = (m[0,2] - m[2,0]) / S
        qz = (m[1,0] - m[0,1]) / S
    elif (m[0,0] > m[1,1]) and (m[0,0] > m[2,2]):
        S = np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2  # S=4*qx
        qw = (m[2,1] - m[1,2]) / S
        qx = 0.25 * S
        qy = (m[0,1] + m[1,0]) / S
        qz = (m[0,2] + m[2,0]) / S
    elif m[1,1] > m[2,2]:
        S = np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2  # S=4*qy
        qw = (m[0,2] - m[2,0]) / S
        qx = (m[0,1] + m[1,0]) / S
        qy = 0.25 * S
        qz = (m[1,2] + m[2,1]) / S
    else:
        S = np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2  # S=4*qz
        qw = (m[1,0] - m[0,1]) / S
        qx = (m[0,2] + m[2,0]) / S
        qy = (m[1,2] + m[2,1]) / S
        qz = 0.25 * S
    return [qw, qx, qy, qz]

def main():
    args = parse_args()
    # Создаем TCP сервер
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((args.ip, args.port))
    server_socket.listen(1)
    print(f"Сервер запущен на {args.ip}:{args.port}. Ожидание подключения...")

    conn, addr = server_socket.accept()
    print(f"Подключен клиент: {addr}")

    all_grasps = []  # сюда будут добавляться все полученные захваты
    buffer = ""
    start_time = time.time()
    collection_duration = 5.0  # собираем данные 5 секунд

    with conn:
        while True:
            # Если прошло 5 секунд, прекращаем сбор
            if time.time() - start_time > collection_duration:
                break

            data = conn.recv(4096)
            if not data:
                break
            buffer += data.decode('utf-8')
            # Разбиваем по разделителю (новая строка)
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                try:
                    # Предполагается, что каждое сообщение - список захватов
                    grasps = json.loads(line)
                    if isinstance(grasps, list):
                        all_grasps.extend(grasps)
                    else:
                        print("Ожидался список захватов, получено:", grasps)
                except json.JSONDecodeError as e:
                    print("Ошибка декодирования JSON:", e)
                    print("Строка:", line)

    if not all_grasps:
        print("Не получено ни одного захвата за 5 секунд.")
        return

    # Выбираем захват с максимальным score (первый элемент массива)
    best_grasp = max(all_grasps, key=lambda g: g[0])
    score = best_grasp[0]
    # Из структуры: 
    # best_grasp = [score, width, height, depth, rotation_matrix(9), translation(3), object_id]
    # rotation_matrix занимает индексы 4:13, translation – индексы 13:16.
    rot_flat = best_grasp[4:13]
    translation = best_grasp[13:16]
    # Преобразуем rotation_matrix в 3x3 numpy-массив
    rot_matrix = np.array(rot_flat).reshape(3,3)
    quaternion = rotation_matrix_to_quaternion(rot_matrix)

    print("\nЛучший захват за 5 секунд:")
    print(f"Score: {score}")
    print("Положение (translation):")
    print(f"  x: {translation[0]:.6f}, y: {translation[1]:.6f}, z: {translation[2]:.6f}")
    print("Ориентация (кватернион [w, x, y, z]):")
    print(f"  {quaternion[0]:.6f}, {quaternion[1]:.6f}, {quaternion[2]:.6f}, {quaternion[3]:.6f}")

if __name__ == "__main__":
    main()
