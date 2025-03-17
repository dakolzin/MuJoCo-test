#!/usr/bin/env python3
import time
import threading
import argparse
import socket
import json

import gymnasium
import manipulator_mujoco
import mujoco  # для mj_name2id

from transform_utils import quaternion_to_rotation_matrix  # для проверки

def simulation_loop(ip, port):
    """
    Основной цикл симуляции манипулятора с использованием force-сенсоров в захвате.
    
    Функция выполняет:
      - Инициализацию окружения.
      - Запуск потока для получения трансформации через сокет.
      - Управление состояниями симуляции: random -> pre_grasp -> final_grasp -> lift -> done.
      - Плавное закрытие схвата с остановкой по превышению порога силы.
    """
    env = gymnasium.make("manipulator_mujoco/AuboI5Env-v0", render_mode='human')
    observation, info = env.reset(seed=42)
    print("Action space:", env.action_space)

    unwrapped_env = env.unwrapped

    # Получаем индекс актуатора схвата
    gripper_actuator_name = 'fingers_actuator'
    gripper_index = mujoco.mj_name2id(
        unwrapped_env._physics.model.ptr,
        mujoco.mjtObj.mjOBJ_ACTUATOR,
        gripper_actuator_name
    )
    
    # Получаем индексы force-сенсоров для левого и правого пальцев
    left_sensor_id = mujoco.mj_name2id(
        unwrapped_env._physics.model.ptr,
        mujoco.mjtObj.mjOBJ_SENSOR,
        "left_finger_force"
    )
    right_sensor_id = mujoco.mj_name2id(
        unwrapped_env._physics.model.ptr,
        mujoco.mjtObj.mjOBJ_SENSOR,
        "right_finger_force"
    )
    
    # Параметры управления схватом
    open_command = 0.0
    closed_command = 0.943
    FORCE_THRESHOLD = 120.0  # порог силы для остановки закрытия

    gripper_closed = False
    closing_in_progress = False
    closing_start_time = None
    closing_duration = 3.0  # время на плавное закрытие
    current_gripper_value = open_command

    final_grasp_capture_time = None
    new_target_pose = None

    # Состояния: random -> pre_grasp -> final_grasp -> lift -> done
    state = "random"
    pre_grasp_start_time = None
    final_grasp_start_time = None
    lift_start_time = None
    lift_done = False
    translation_base = None
    quaternion_base = None

    def socket_worker():
        nonlocal new_target_pose
        HOST = '127.0.0.1'
        PORT = 54321
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT))
            s.listen(1)
            print(f"Ожидание подключения на {HOST}:{PORT}...")
            conn, addr = s.accept()
            print(f"Подключено: {addr}")
            data = b""
            while True:
                packet = conn.recv(4096)
                if not packet:
                    break
                data += packet
            try:
                transform = json.loads(data.decode('utf-8'))
                print("Получена трансформация base->grasp:")
                print(transform)
                new_target_pose = transform
            except Exception as e:
                print(f"Ошибка при декодировании данных: {e}")

    # Запускаем поток для приёма трансформации
    socket_thread = threading.Thread(target=socket_worker, daemon=True)
    socket_thread.start()

    while True:
        # 1. Управление схватом с использованием force-сенсоров
        if closing_in_progress:
            elapsed = time.time() - closing_start_time
            alpha = min(1.0, elapsed / closing_duration)
            current_gripper_value = open_command + alpha * (closed_command - open_command)
            unwrapped_env._physics.data.ctrl[gripper_index] = current_gripper_value

            # Считываем силу с датчиков
            left_force = unwrapped_env._physics.data.sensordata[left_sensor_id]
            right_force = unwrapped_env._physics.data.sensordata[right_sensor_id]
            total_force = left_force + right_force
            print(f"[Gripper] Сила: левая {left_force:.2f} Н, правая {right_force:.2f} Н, суммарная {total_force:.2f} Н")

            if total_force >= FORCE_THRESHOLD:
                print(f"[Gripper] Суммарная сила {total_force:.2f} Н >= порога {FORCE_THRESHOLD} Н, прекращаем закрытие.")
                closing_in_progress = False
                gripper_closed = True
            elif alpha >= 1.0:
                print("[Gripper] Закрытие завершено по времени.")
                closing_in_progress = False
                gripper_closed = True
        else:
            unwrapped_env._physics.data.ctrl[gripper_index] = current_gripper_value

        # 2. Логика состояний симуляции
        if state == "random":
            action = env.action_space.sample()
            observation, reward, terminated, truncated, info = env.step(action)
            if not closing_in_progress and not gripper_closed:
                current_gripper_value = open_command

            if new_target_pose is not None:
                translation_base = new_target_pose.get("translation")
                quaternion_base = new_target_pose.get("rotation")
                # Смещение для безопасного подхода
                safe_offset = [0, 0, 0.1]
                pre_grasp_translation = [
                    translation_base[0] + safe_offset[0],
                    translation_base[1] + safe_offset[1],
                    translation_base[2] + safe_offset[2]
                ]
                print("Переход в режим pre_grasp (безопасное приближение).")
                if hasattr(env.unwrapped, "_target") and hasattr(env.unwrapped, "_physics"):
                    env.unwrapped._target.set_mocap_pose(
                        physics=env.unwrapped._physics,
                        position=pre_grasp_translation,
                        quaternion=quaternion_base
                    )
                else:
                    print("Target не найден в env.unwrapped!")
                pre_grasp_start_time = time.time()
                state = "pre_grasp"

        elif state == "pre_grasp":
            no_op = [0] * env.action_space.shape[0]
            observation, reward, terminated, truncated, info = env.step(no_op)
            if not closing_in_progress and not gripper_closed:
                current_gripper_value = open_command

            if time.time() - pre_grasp_start_time >= 10.0:
                print("Переход к финальной позе захвата!")
                if hasattr(env.unwrapped, "_target") and hasattr(env.unwrapped, "_physics"):
                    env.unwrapped._target.set_mocap_pose(
                        physics=env.unwrapped._physics,
                        position=translation_base,
                        quaternion=quaternion_base
                    )
                    rot_check = quaternion_to_rotation_matrix(quaternion_base)
                    print("Восстановленная матрица вращения:")
                    print(rot_check)
                    print("Координаты схвата (base):")
                    print(f"x: {translation_base[0]:.6f}, y: {translation_base[1]:.6f}, z: {translation_base[2]:.6f}")
                else:
                    print("Target не найден в env.unwrapped!")
                final_grasp_start_time = time.time()
                state = "final_grasp"

        elif state == "final_grasp":
            action = env.action_space.sample()
            observation, reward, terminated, truncated, info = env.step(action)
            if final_grasp_start_time is not None and (time.time() - final_grasp_start_time >= 3.0):
                if not gripper_closed and not closing_in_progress:
                    print("Начинается плавное закрытие схвата!")
                    closing_in_progress = True
                    closing_start_time = time.time()
            if gripper_closed:
                if final_grasp_capture_time is None:
                    final_grasp_capture_time = time.time()
                if time.time() - final_grasp_capture_time >= 2.0:
                    print("Переход в режим подъёма (lift).")
                    lift_start_time = time.time()
                    state = "lift"

        elif state == "lift":
            if not lift_done:
                lift_offset = [0, 0, 0.4]
                lift_translation = [
                    translation_base[0] + lift_offset[0],
                    translation_base[1] + lift_offset[1],
                    translation_base[2] + lift_offset[2]
                ]
                print("Выполняется подъём: новая цель =", lift_translation)
                if hasattr(env.unwrapped, "_target") and hasattr(env.unwrapped, "_physics"):
                    env.unwrapped._target.set_mocap_pose(
                        physics=env.unwrapped._physics,
                        position=lift_translation,
                        quaternion=quaternion_base
                    )
                else:
                    print("Target не найден в env.unwrapped!")
                lift_done = True
                lift_start_time = time.time()
            else:
                if time.time() - lift_start_time > 5.0:
                    print("Подъём завершён, переходим в состояние done.")
                    state = "done"

        elif state == "done":
            no_op = [0] * env.action_space.shape[0]
            observation, reward, terminated, truncated, info = env.step(no_op)
            # Здесь можно завершить симуляцию, если нужно
            # break

    env.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Симуляция с манипулятором, обновлением цели через сокет и обратной связью от force-сенсоров"
    )
    parser.add_argument('--ip', type=str, default='0.0.0.0', help='IP для сокета')
    parser.add_argument('--port', type=int, default=12345, help='Порт для сокета')
    args = parser.parse_args()

    simulation_loop(args.ip, args.port)
