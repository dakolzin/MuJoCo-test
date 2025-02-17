import numpy as np

class GripperPositionController:
    def __init__(self, physics, gripper, kp=10.0):
        """
        Инициализация позиционного контроллера для схвата.
        
        Parameters:
            physics: объект симуляции (например, env.unwrapped._physics)
            gripper: объект схвата (например, экземпляр класса AG95)
            kp: пропорциональный коэффициент контроля
        """
        self._physics = physics
        self._gripper = gripper
        self._kp = kp
        self._target_position = None

    def set_target(self, target_position):
        """
        Задает желаемое положение (qpos) для управления схватом.
        
        Parameters:
            target_position: желаемое значение joint qpos (скаляр)
        """
        self._target_position = target_position

    def update(self):
        """
        Вызывайте эту функцию на каждом шаге симуляции, чтобы обновить управление схватом.
        Вычисляется ошибка между текущим положением и целевым, и подается командное усилие.
        """
        if self._target_position is None:
            return
        
        # Получаем текущее положение сустава схвата.
        current_pos = self._physics.bind(self._gripper.joint).qpos[0]
        error = self._target_position - current_pos
        
        # Пропорциональный контроль.
        command = self._kp * error
        
        # (Опционально) можно добавить ограничение по величине команды.
        command = np.clip(command, -0.1, 0.1)
        
        # Отправляем команду на схват через qfrc_applied.
        self._physics.bind(self._gripper.joint).qfrc_applied = np.array([command])
