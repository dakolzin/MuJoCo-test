import socket
import json
import time
import numpy as np
from utils import rotation_matrix_to_quaternion

def run_socket_server(ip='0.0.0.0', port=12345, collection_duration=5.0):
    """
    Запускает TCP-сервер, собирает данные захватов за collection_duration секунд
    и возвращает лучший захват в виде кортежа (translation, quaternion).
    Если данных не получено, возвращается None.
    """
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((ip, port))
    server_socket.listen(1)
    print(f"Сервер запущен на {ip}:{port}. Ожидание подключения...")
    conn, addr = server_socket.accept()
    print(f"Подключен клиент: {addr}")

    all_grasps = []
    buffer = ""
    start_time = time.time()

    with conn:
        while True:
            if time.time() - start_time > collection_duration:
                break
            data = conn.recv(4096)
            if not data:
                break
            buffer += data.decode('utf-8')
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                try:
                    grasps = json.loads(line)
                    if isinstance(grasps, list):
                        all_grasps.extend(grasps)
                    else:
                        print("Ожидался список захватов, получено:", grasps)
                except json.JSONDecodeError as e:
                    print("Ошибка декодирования JSON:", e)
                    print("Строка:", line)

    server_socket.close()

    if not all_grasps:
        print("Не получено ни одного захвата за отведённое время.")
        return None

    best_grasp = max(all_grasps, key=lambda g: g[0])
    score = best_grasp[0]
    # Извлекаем rotation_matrix (индексы 4:13) и translation (индексы 13:16)
    rot_flat = best_grasp[4:13]
    translation = best_grasp[13:16]
    rot_matrix = np.array(rot_flat).reshape(3, 3)
    quaternion = rotation_matrix_to_quaternion(rot_matrix)

    print("\nЛучший захват за 5 секунд:")
    print(f"Score: {score}")
    print("Положение (translation):")
    print(f"  x: {translation[0]:.6f}, y: {translation[1]:.6f}, z: {translation[2]:.6f}")
    print("Ориентация (кватернион [w, x, y, z]):")
    print(f"  {quaternion[0]:.6f}, {quaternion[1]:.6f}, {quaternion[2]:.6f}, {quaternion[3]:.6f}")
    
    return translation, quaternion
