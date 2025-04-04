#diff.py

import os
from dm_control import mjcf

_DIFF_XML = os.path.join(
    os.path.dirname(__file__),
    '../assets/objects/diff/diff.xml',
)

class DIFF:
    def __init__(self, name: str = None):
        self.name = name if name is not None else "DIFF"
        # Загружаем MJCF-модель камеры из XML
        self.mjcf_model = mjcf.from_path(_DIFF_XML)