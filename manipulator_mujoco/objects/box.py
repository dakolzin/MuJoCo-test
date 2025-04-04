#box.py

import os
from dm_control import mjcf

_BOX_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/objects/box/box.xml',
)

class BOX:
    def __init__(self, name: str = None):
        self.name = name if name is not None else "BOX"
        # Загружаем MJCF-модель камеры из XML
        self.mjcf_model = mjcf.from_path(_BOX_XML)