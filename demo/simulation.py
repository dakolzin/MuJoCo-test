import time
import threading
import argparse
import gymnasium
import manipulator_mujoco

from socket_server import run_socket_server
from transform_utils import transform_camera_to_base

def simulation_loop(ip, port):
    env = gymnasium.make("manipulator_mujoco/AuboI5Env-v0", render_mode='human')
    observation, info = env.reset(seed=42)
    print("Action space:", env.action_space)

    # Параметры положения/ориентации камеры в базовой системе
    camera_pos = [0, 0.3, 0.26]
    camera_quat = [0.44499672, 0.54952518, 0.54952518, 0.44499672]  # [w, x, y, z]

    new_target_pose = None
    # Состояния: "random", "pre_grasp", "final_grasp"
    state = "random"
    pre_grasp_start_time = None
    translation_base = None
    quaternion_base = None

    def socket_worker():
        nonlocal new_target_pose
        # Сбор данных из сокета в течение 5 секунд
        new_target_pose = run_socket_server(ip, port)

    # Запуск потока для сокет-сервера
    socket_thread = threading.Thread(target=socket_worker, daemon=True)
    socket_thread.start()

    while True:
        if state == "random":
            # В режиме "random" выполняем случайное действие
            action = env.action_space.sample()
            observation, reward, terminated, truncated, info = env.step(action)

            # Если получена новая цель, переходим в режим pre_grasp
            if new_target_pose is not None:
                translation_camera, quaternion_camera = new_target_pose
                translation_base, quaternion_base = transform_camera_to_base(
                    camera_pos,
                    camera_quat,
                    translation_camera,
                    quaternion_camera
                )
                # Вычисляем pre-grasp позицию: добавляем смещение вверх (+0.1 м по Z)
                safe_offset = [0, 0, 0.1]
                pre_grasp_translation = [
                    translation_base[0] + safe_offset[0],
                    translation_base[1] + safe_offset[1],
                    translation_base[2] + safe_offset[2]
                ]

                print("Moving to pre-grasp pose to avoid collision!")
                if hasattr(env.unwrapped, "_target") and hasattr(env.unwrapped, "_physics"):
                    env.unwrapped._target.set_mocap_pose(
                        physics=env.unwrapped._physics,
                        position=pre_grasp_translation,
                        quaternion=quaternion_base
                    )
                else:
                    print("Target not found in env.unwrapped!")
                pre_grasp_start_time = time.time()
                state = "pre_grasp"

        elif state == "pre_grasp":
            # В режиме pre_grasp не применяем случайное действие, а выполняем no-op,
            # чтобы симуляция шла, и робот мог перейти в предвариельную позицию.
            no_op = [0] * env.action_space.shape[0]
            observation, reward, terminated, truncated, info = env.step(no_op)
            # Ждем 5 секунд (с шагами симуляции) для завершения движения
            if time.time() - pre_grasp_start_time >= 5.0:
                print("Moving target to final grasp pose!")
                if hasattr(env.unwrapped, "_target") and hasattr(env.unwrapped, "_physics"):
                    env.unwrapped._target.set_mocap_pose(
                        physics=env.unwrapped._physics,
                        position=translation_base,
                        quaternion=quaternion_base
                    )
                else:
                    print("Target not found in env.unwrapped!")
                state = "final_grasp"

        elif state == "final_grasp":
            # В режиме final_grasp продолжаем симуляцию, например, с случайными действиями.
            action = env.action_space.sample()
            observation, reward, terminated, truncated, info = env.step(action)
            # Когда эпизод завершён, сбрасываем состояние
            if terminated or truncated:
                observation, info = env.reset()
                new_target_pose = None
                state = "random"
                # Перезапускаем сбор данных из сокета
                socket_thread = threading.Thread(target=socket_worker, daemon=True)
                socket_thread.start()

        # Если эпизод завершён, сбрасываем окружение и состояние
        if terminated or truncated:
            observation, info = env.reset()
            new_target_pose = None
            state = "random"
            socket_thread = threading.Thread(target=socket_worker, daemon=True)
            socket_thread.start()

    env.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Симуляция с манипулятором и обновлением цели через сокет")
    parser.add_argument('--ip', type=str, default='0.0.0.0', help='IP для сокета')
    parser.add_argument('--port', type=int, default=12345, help='Порт для сокета')
    args = parser.parse_args()

    simulation_loop(args.ip, args.port)
