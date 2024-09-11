from pydantic import BaseModel

from spaceros_procgen_envs.core.assets import AssetBaseCfg


class AssetCfg(BaseModel):
    ## Model
    asset_cfg: AssetBaseCfg
