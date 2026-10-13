from dataclasses import MISSING

from omni.isaac.lab.utils import configclass

from spaceros_procgen_envs.core.actions import (
    BinaryJointPositionActionCfg,
    DifferentialInverseKinematicsActionCfg,
)


@configclass
class ManipulatorTaskSpaceActionCfg:
    arm: DifferentialInverseKinematicsActionCfg = MISSING
    hand: BinaryJointPositionActionCfg = MISSING
