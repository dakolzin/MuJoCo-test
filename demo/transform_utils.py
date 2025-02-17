import numpy as np
from utils import rotation_matrix_to_quaternion

def quaternion_to_rotation_matrix(q):
    """
    Преобразует кватернион q = [w, x, y, z] в 3x3 матрицу поворота.
    """
    w, x, y, z = q
    return np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x**2 + z**2),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
    ])

def quaternion_multiply(q1, q2):
    """
    Умножает два кватерниона q1 и q2 (формат [w, x, y, z]).
    Возвращает произведение в формате [w, x, y, z].
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return [w, x, y, z]

def transform_camera_to_base(camera_pos, camera_quat, target_pos_camera, target_quat_camera):
    # Вычисляем матрицу поворота камеры
    R_cam = quaternion_to_rotation_matrix(camera_quat)
    print("Вычисленная матрица поворота камеры (R_cam):\n", R_cam)
    
    # Корректируем координаты цели: инвертируем оси X и Y
    target_pos_camera_corr = np.array([
        -target_pos_camera[0],
        -target_pos_camera[1],
         target_pos_camera[2]
    ])
    print("Скорректированные координаты цели в системе камеры:", target_pos_camera_corr)
    
    intermediate = R_cam.dot(target_pos_camera_corr)
    print("Промежуточный результат R_cam * target_pos_camera:", intermediate)
    
    target_pos_base = np.array(camera_pos) + intermediate
    print("Положение в базовой системе:", target_pos_base)
    
    # Коррекция ориентации: инвертируем компоненты X и Y кватерниона цели
    target_quat_camera_corr = [
        target_quat_camera[0],
        -target_quat_camera[1],
        -target_quat_camera[2],
         target_quat_camera[3]
    ]
    
    target_quat_base = quaternion_multiply(camera_quat, target_quat_camera_corr)
    print("Итоговый кватернион:", target_quat_base)
    print("=== Конец трансформации ===")
    
    return target_pos_base.tolist(), target_quat_base
