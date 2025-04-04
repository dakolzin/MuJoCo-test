#desk.py

import os
from dm_control import mjcf

_DESK_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/objects/desk/desk.xml',
)

class DESK:
    def __init__(self, name: str = None):
        self.name = name if name is not None else "DESK"
        # Загружаем MJCF-модель камеры из XML
        self.mjcf_model = mjcf.from_path(_DESK_XML)