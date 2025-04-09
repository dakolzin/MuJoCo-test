import pybullet as p

# Запускаем PyBullet без GUI:
p.connect(p.DIRECT)

# Создаем шейпы (коллизию и визуальную геометрию):
meshScale = [1, 1, 1]  # тот же масштаб, что и в Blender
collision_id = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName="VASE.stl",
    meshScale=meshScale
)
visual_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="VASE.stl",
    meshScale=meshScale
)

# Задаем массу (m = 3 кг, к примеру)
mass = 1.0

# Создаем тело
body_id = p.createMultiBody(
    baseMass=mass,
    baseCollisionShapeIndex=collision_id,
    baseVisualShapeIndex=visual_id
)

# Спрашиваем PyBullet про динамику
dinfo = p.getDynamicsInfo(body_id, -1)
print("mass =", dinfo[0])
print("localInertiaDiagonal =", dinfo[2])
print("localInertialPos =", dinfo[3])  # центр масс
print("localInertialOrientation =", dinfo[4])  # кватернион ориентации инерции
