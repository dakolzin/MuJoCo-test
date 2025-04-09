import math

def euler_to_quaternion(roll, pitch, yaw):
    # Переводим углы в радианы
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    
    # Вычисляем полууглы
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    # Формулы преобразования
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)

# Задаём углы: roll=90°, pitch=180°, yaw=0°
quat = euler_to_quaternion(0, 90, 0)
print("Кватернион (w, x, y, z):", quat)