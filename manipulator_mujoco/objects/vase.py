import os
from dm_control import mjcf

_VASE_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/objects/vase/vase.xml',
)

class VASE:
    def __init__(self, name: str = None):
        self.name = name if name is not None else "VASE"
        # Загружаем MJCF-модель камеры из XML
        self.mjcf_model = mjcf.from_path(_VASE_XML)

