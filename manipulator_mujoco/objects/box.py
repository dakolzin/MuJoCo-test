#box.py

import os
from dm_control import mjcf

_box_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/objects/box/box.xml',
)

class box:
    def __init__(self, name: str = None):
        self.name = name if name is not None else "box"
        # Загружаем MJCF-модель камеры из XML
        self.mjcf_model = mjcf.from_path(_box_XML)