import time
import gymnasium
import numpy as np
import manipulator_mujoco

def main():
    # Создаем окружение с визуализацией в режиме "human"
    env = gymnasium.make('manipulator_mujoco/AuboI5Env-v0', render_mode='human')
    observation, info = env.reset(seed=42)
    print("Action space:", env.action_space)

    # Задаем значение положения, соответствующее закрытому схвату.
    # Значение нужно подбирать в зависимости от модели. Здесь 0.05 – пример.
    closed_position = 0.05

    # Прямо изменяем положение сустава схвата:
    env.unwrapped._physics.bind([env.unwrapped._gripper.joint]).qpos = [closed_position]

    # Шагаем симуляцию несколько раз, чтобы изменения "применились"
    for _ in range(50):
        observation, reward, terminated, truncated, info = env.step(np.zeros(7, dtype=np.float64))
        time.sleep(env.unwrapped._timestep)
        if terminated or truncated:
            break

    env.close()

if __name__ == "__main__":
    main()
