#stand.py

import os
from dm_control import mjcf

_STAND_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/objects/stand/stand.xml',
)

class STAND:
    def __init__(self, name: str = None):
        self.name = name if name is not None else "STAND"
        # Загружаем MJCF-модель камеры из XML
        self.mjcf_model = mjcf.from_path(_STAND_XML)