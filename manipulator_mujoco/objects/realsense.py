import os
from dm_control import mjcf

_D435I_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/objects/realsense_d435i/d435i.xml',
)

class RealsenseD435i:
    def __init__(self, name: str = None):
        self.name = name if name is not None else "realsense_d435i"
        # Загружаем MJCF-модель камеры из XML
        self.mjcf_model = mjcf.from_path(_D435I_XML)

