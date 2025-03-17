#!/usr/bin/env python3
import socket
import json
import time
import numpy as np

def main():
    """
    Этот скрипт-клиент подключается к серверу socket_server.py (по умолчанию на 127.0.0.1:12345)
    и отправляет один "захват" (grasp) в формате [score, width, height, depth, rotation(9), translation(3), object_id].
    Таким образом вы можете протестировать, как ваша симуляция интерпретирует полученные координаты.
    """

    # 1) Определяем нужное положение и ориентацию (кватернион),
    #    а также строим из него 3x3 матрицу поворота.
    #    Ниже просто пример с уже известными значениями.

    # Допустим, у нас есть:
    #   Положение (translation): x=0.437337, y=0.282593, z=0.056936
    #   Кватернион: w=-0.006719, x=0.014292, y=-0.08784, z=0.996009
    #   (Формат [w, x, y, z])
    w  = -0.006719
    qx =  0.014292
    qy =  -0.08784
    qz =  0.996009

    # Из кватерниона получаем матрицу поворота (row-major)
    R00 = 1 - 2*(qy**2 + qz**2)
    R01 = 2*(qx*qy - qz*w)
    R02 = 2*(qx*qz + qy*w)

    R10 = 2*(qx*qy + qz*w)
    R11 = 1 - 2*(qx**2 + qz**2)
    R12 = 2*(qy*qz - qx*w)

    R20 = 2*(qx*qz - qy*w)
    R21 = 2*(qy*qz + qx*w)
    R22 = 1 - 2*(qx**2 + qy**2)

    # Положение
    tx = 0.437337
    ty = 0.282593
    tz = 0.056936

    # 2) Формируем один "grasp" в формате 17 чисел:
    #    [score, width, height, depth, 9-элементная матрица, 3 координаты, object_id]
    #    Здесь score=0.9, width=0.02, height=0.03, depth=0.04, object_id=999 (для примера).
    grasp = [
        0.5,      # score
        0.02,     # width
        0.03,     # height
        0.04,     # depth
        R00, R01, R02,
        R10, R11, R12,
        R20, R21, R22,
        tx, ty, tz,
        999       # object_id (можно любое)
    ]

    # 3) Формируем список, чтобы отправить именно "список захватов" (в данном случае один).
    #    socket_server.py ждет JSON-строку вида: [[...17 чисел...]]
    data_to_send = [grasp]

    # 4) Подключаемся к серверу (по умолчанию: 127.0.0.1:12345) и отправляем одну строку JSON + \n
    host = '127.0.0.1'
    port = 12345
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        print(f"Подключаемся к {host}:{port}...")
        s.connect((host, port))
        line = json.dumps(data_to_send) + "\n"
        s.sendall(line.encode('utf-8'))
        print("Данные отправлены. Ожидаем 1 сек...")
        time.sleep(1)
    print("Соединение закрыто.")

if __name__ == '__main__':
    main()
