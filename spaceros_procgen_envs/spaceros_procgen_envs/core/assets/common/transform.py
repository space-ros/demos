from typing import Tuple

from pydantic import BaseModel


class TransformCfg(BaseModel):
    translation: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)  # w, x, y, z


class FrameCfg(BaseModel):
    prim_relpath: str
    offset: TransformCfg = TransformCfg()
