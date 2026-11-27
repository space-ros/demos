from os import path

from omni.isaac.lab.utils import configclass

import spaceros_procgen_envs.core.sim as sim_utils


@configclass
class SampleTubeCfg(sim_utils.UsdFileCfg):
    usd_path = path.join(path.dirname(path.realpath(__file__)), "sample_tube.usdc")
