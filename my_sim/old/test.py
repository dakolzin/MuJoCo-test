import time
import gymnasium
import manipulator_mujoco
import numpy as np

def main():
    # Создаём окружение с визуализацией
    env = gymnasium.make("manipulator_mujoco/AuboI5Env-v0", render_mode='human')
    observation, info = env.reset(seed=42)
    print(env.action_space)  # Например, Box(-0.1, 0.1, (6,), float64)

    start_time = time.time()
    target_moved = False

    while True:
        # Делаем случайное действие
        action = env.action_space.sample()
        observation, reward, terminated, truncated, info = env.step(action)

        # Через 5 секунд меняем целевое положение
        current_time = time.time()
        if not target_moved and (current_time - start_time >= 5.0):
            print("Moving target!")
            # Обращаемся к целевому объекту через env.unwrapped, если он существует
            if hasattr(env.unwrapped, "_target"):
                env.unwrapped._target.set_mocap_pose(
                    physics=env.unwrapped._physics,  # убедитесь, что _physics существует
                    position=[0.3, 0.2, 0.45],        # новая позиция
                    quaternion=[0, 0, 0, 1]           # ориентация без поворота
                )
            else:
                print("Target not found in env.unwrapped!")
            target_moved = True

        # Если эпизод завершился — сбрасываем окружение
        if terminated or truncated:
            observation, info = env.reset()
            start_time = time.time()
            target_moved = False

    env.close()

if __name__ == "__main__":
    main()
