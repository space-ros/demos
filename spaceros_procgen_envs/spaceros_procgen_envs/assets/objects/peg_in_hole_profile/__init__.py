from os import path

from omni.isaac.lab.utils import configclass

import spaceros_procgen_envs.core.sim as sim_utils


@configclass
class ProfilePegCfg(sim_utils.UsdFileCfg):
    usd_path = path.join(path.dirname(path.realpath(__file__)), "profile.usdc")


@configclass
class ProfileHoleCfg(sim_utils.UsdFileCfg):
    usd_path = path.join(
        path.dirname(path.realpath(__file__)),
        "hole.usdc",
    )
