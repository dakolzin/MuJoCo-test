from scipy.spatial.transform import Rotation as R
import numpy as np

# Определяем углы Эйлера в градусах (например, по оси XYZ)
euler_angles = [90, 0, 0]  # задайте свои углы
# Преобразуем в кватернион; результат по умолчанию в порядке [x, y, z, w]
r = R.from_euler('xyz', euler_angles, degrees=True)
quat_xyzw = r.as_quat()

# MuJoCo обычно использует порядок кватерниона [w, x, y, z]
quat_wxyz = np.concatenate(([quat_xyzw[3]], quat_xyzw[:3]))

print("Кватернион (w, x, y, z):", quat_wxyz)
