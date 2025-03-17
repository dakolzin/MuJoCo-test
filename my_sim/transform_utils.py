import numpy as np
from utils import quaternion_multiply, rotation_matrix_to_quaternion

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

def transform_camera_to_base(camera_pos, camera_quat, target_pos_camera, target_quat_camera):
    """
    Преобразует положение цели из системы камеры в базовую систему робота.
    Ориентация цели не изменяется и возвращается как есть.
    """
    # Поворот на 180° вокруг Z (из-за несовпадения осей камеры и робота)
    correction_quat = [0, 0, 1, 0]  # (180 градусов по Z)

    # --- ПОЛОЖЕНИЕ ---
    R_cam = quaternion_to_rotation_matrix(camera_quat)
    target_pos_camera_corr = np.array([
        -target_pos_camera[0],  # X инвертируется
        -target_pos_camera[1],  # Y инвертируется
        target_pos_camera[2]   # Z остаётся тем же
    ])
    pos_intermediate = R_cam.dot(target_pos_camera_corr)
    target_pos_base = np.array(camera_pos) + pos_intermediate

    # --- ОРИЕНТАЦИЯ ---
    # Меняем порядок умножения кватернионов
    target_quat_base = quaternion_multiply(target_quat_camera, correction_quat)

    return target_pos_base.tolist(), target_quat_base
