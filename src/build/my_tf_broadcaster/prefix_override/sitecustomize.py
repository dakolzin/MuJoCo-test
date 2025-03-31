import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/danil/Manipulator-Mujoco/src/install/my_tf_broadcaster'
