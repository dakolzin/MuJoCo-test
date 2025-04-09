#!/usr/bin/env python3
import time
import threading
import argparse
import socket
import json
import queue

import gymnasium
import manipulator_mujoco
import mujoco  # для mj_name2id

from transform_utils import quaternion_to_rotation_matrix  # для проверки

def simulation_loop(ip, port):
    """
    Основной цикл симуляции манипулятора:
    - Слушаем сокет в отдельном потоке, разбиваем входящие данные по \n.
    - Каждую полную JSON-строку парсим и кладём в очередь transform_queue.
    - В главном потоке блокируемся на transform_queue.get() -> запускаем state machine.
    - По окончании state machine (done) снова ждём следующего transform.
    """
    env = gymnasium.make("manipulator_mujoco/AuboI5EnvDiff-v0", render_mode='human')
    unwrapped_env = env.unwrapped

    transform_queue = queue.Queue()

    def socket_worker():
        HOST = ip
        PORT = port
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT))
            s.listen(1)
            print(f"[Socket] Ожидание подключения на {HOST}:{PORT}...")
            conn, addr = s.accept()
            print(f"[Socket] Подключено: {addr}")

            buffer = ""
            while True:
                chunk = conn.recv(4096)
                if not chunk:
                    print("[Socket] Клиент закрыл соединение. Завершаем socket_worker().")
                    break
                buffer += chunk.decode('utf-8')

                # Разбиваем по переносу строки
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        transform = json.loads(line)
                        print("[Socket] Принят JSON:", transform)
                        transform_queue.put(transform)
                    except json.JSONDecodeError as e:
                        print("[Socket] Невалидный JSON:", line, e)

    sock_thread = threading.Thread(target=socket_worker, daemon=True)
    sock_thread.start()

    # ------------ Параметры -------------
    gripper_actuator_name = 'fingers_actuator'
    left_force_sensor_name = "left_finger_force"
    right_force_sensor_name = "right_finger_force"

    def init_env():
        observation, info = env.reset(seed=42)
        print("Action space:", env.action_space)

        gripper_idx = mujoco.mj_name2id(
            unwrapped_env._physics.model.ptr,
            mujoco.mjtObj.mjOBJ_ACTUATOR,
            gripper_actuator_name
        )
        left_sensor_id = mujoco.mj_name2id(
            unwrapped_env._physics.model.ptr,
            mujoco.mjtObj.mjOBJ_SENSOR,
            left_force_sensor_name
        )
        right_sensor_id = mujoco.mj_name2id(
            unwrapped_env._physics.model.ptr,
            mujoco.mjtObj.mjOBJ_SENSOR,
            right_force_sensor_name
        )
        return gripper_idx, left_sensor_id, right_sensor_id

    open_command = 0.0
    closed_command = 0.943
    FORCE_THRESHOLD = 1200.0

    def run_state_machine(transform):
        """Запускаем цикл random->pre_grasp->final_grasp->lift->done для данного transform."""
        gripper_closed = False
        closing_in_progress = False
        closing_start_time = None
        closing_duration = 3.0
        current_gripper_value = open_command

        final_grasp_capture_time = None

        state = "random"
        pre_grasp_start_time = None
        final_grasp_start_time = None
        lift_start_time = None
        lift_done = False
        translation_base = None
        quaternion_base = None

        # Сбрасываем мир:
        gripper_index, left_sensor_id, right_sensor_id = init_env()

        print("\n=== Начинается новый прогон state machine ===")

        while True:
            # 1) Управление схватом (гладкое закрытие)
            if closing_in_progress:
                elapsed = time.time() - closing_start_time
                alpha = min(1.0, elapsed / closing_duration)
                current_gripper_value = open_command + alpha * (closed_command - open_command)
                unwrapped_env._physics.data.ctrl[gripper_index] = current_gripper_value

                left_force = unwrapped_env._physics.data.sensordata[left_sensor_id]
                right_force = unwrapped_env._physics.data.sensordata[right_sensor_id]
                total_force = left_force + right_force

                if total_force >= FORCE_THRESHOLD:
                    print(f"[Gripper] Превышен порог {FORCE_THRESHOLD}. Останавливаемся.")
                    closing_in_progress = False
                    gripper_closed = True
                elif alpha >= 1.0:
                    print("[Gripper] Закрытие завершено по времени")
                    closing_in_progress = False
                    gripper_closed = True
            else:
                unwrapped_env._physics.data.ctrl[gripper_index] = current_gripper_value

            # 2) Логика состояний
            if state == "random":
                action = env.action_space.sample()
                observation, reward, terminated, truncated, info = env.step(action)
                # -- Выводим в консоль текущую позу MOCAP
                print_mocap_pose(unwrapped_env)

                if not closing_in_progress and not gripper_closed:
                    current_gripper_value = open_command

                if transform is not None:
                    translation_base = transform.get("translation")
                    quaternion_base = transform.get("rotation")
                    safe_offset = [0,0,0.1]
                    pre_grasp_translation = [
                        translation_base[0]+safe_offset[0],
                        translation_base[1]+safe_offset[1],
                        translation_base[2]+safe_offset[2]
                    ]
                    print("[State] Переход -> pre_grasp")
                    if hasattr(env.unwrapped, "_target") and hasattr(env.unwrapped, "_physics"):
                        env.unwrapped._target.set_mocap_pose(
                            physics=env.unwrapped._physics,
                            position=pre_grasp_translation,
                            quaternion=quaternion_base
                        )
                    pre_grasp_start_time = time.time()
                    state = "pre_grasp"

            elif state == "pre_grasp":
                no_op = [0]*env.action_space.shape[0]
                observation, reward, terminated, truncated, info = env.step(no_op)
                # -- Выводим в консоль текущую позу MOCAP
                print_mocap_pose(unwrapped_env)

                if not closing_in_progress and not gripper_closed:
                    current_gripper_value = open_command

                if time.time()-pre_grasp_start_time >= 10.0:
                    print("[State] Переход -> final_grasp")
                    if hasattr(env.unwrapped, "_target") and hasattr(env.unwrapped, "_physics"):
                        env.unwrapped._target.set_mocap_pose(
                            physics=env.unwrapped._physics,
                            position=translation_base,
                            quaternion=quaternion_base
                        )
                        rot_check = quaternion_to_rotation_matrix(quaternion_base)
                        print("[State] Восстановленная матрица вращения:\n", rot_check)
                    final_grasp_start_time = time.time()
                    state = "final_grasp"

            elif state == "final_grasp":
                action = env.action_space.sample()
                observation, reward, terminated, truncated, info = env.step(action)
                # -- Выводим в консоль текущую позу MOCAP
                print_mocap_pose(unwrapped_env)

                if final_grasp_start_time and (time.time()-final_grasp_start_time >= 3.0):
                    if not gripper_closed and not closing_in_progress:
                        print("[State] Начинаем закрытие схвата.")
                        closing_in_progress = True
                        closing_start_time = time.time()
                if gripper_closed:
                    if final_grasp_capture_time is None:
                        final_grasp_capture_time = time.time()
                    if time.time()-final_grasp_capture_time >= 2.0:
                        print("[State] Переход -> lift")
                        lift_start_time = time.time()
                        state = "lift"

            elif state == "lift":
                action = env.action_space.sample()
                observation, reward, terminated, truncated, info = env.step(action)
                # -- Выводим в консоль текущую позу MOCAP
                print_mocap_pose(unwrapped_env)

                if not lift_done:
                    lift_offset = [0,0,0.4]
                    lift_translation = [
                        translation_base[0] + lift_offset[0],
                        translation_base[1] + lift_offset[1],
                        translation_base[2] + lift_offset[2]
                    ]
                    print("[State] Подъём, target=", lift_translation)
                    if hasattr(env.unwrapped, "_target") and hasattr(env.unwrapped, "_physics"):
                        env.unwrapped._target.set_mocap_pose(
                            physics=env.unwrapped._physics,
                            position=lift_translation,
                            quaternion=quaternion_base
                        )
                    lift_done = True
                else:
                    if time.time()-lift_start_time > 5.0:
                        print("[State] Подъём завершен -> done")
                        state = "done"

            elif state == "done":
                no_op = [0]*env.action_space.shape[0]
                observation, reward, terminated, truncated, info = env.step(no_op)
                # -- Выводим в консоль текущую позу MOCAP
                print_mocap_pose(unwrapped_env)

                print("[State] Прогон завершен, ждем следующую трансформацию.")
                break

    # Вспомогательная функция, которая печатает текущий mocap-позу
    def print_mocap_pose(env_unwrapped):
        if hasattr(env_unwrapped, "_target") and hasattr(env_unwrapped, "_physics"):
            pose = env_unwrapped._target.get_mocap_pose(env_unwrapped._physics)
            # pose это массив из 7 чисел: [px, py, pz, qx, qy, qz, qw]
            pos = pose[:3]
            quat = pose[3:]
            #print(f"[MocapPose] pos={pos}, quat={quat}")

    # Главный цикл: ждём трансформы -> run_state_machine() -> снова ждём
    print("[Main] Готов к получению трансформаций.")
    while True:
        transform = transform_queue.get()  # блокирующий
        if transform is None:
            print("[Main] Получен None, завершаем.")
            break
        run_state_machine(transform)

    env.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', type=str, default='127.0.0.1', help='IP для сокета')
    parser.add_argument('--port', type=int, default=54321, help='Порт для сокета')
    args = parser.parse_args()

    simulation_loop(args.ip, args.port)
