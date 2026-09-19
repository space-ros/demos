from spaceros_procgen_envs.core.markers import FRAME_MARKER_CFG

FRAME_MARKER_SMALL_CFG = FRAME_MARKER_CFG.copy()
FRAME_MARKER_SMALL_CFG.markers["frame"].scale = (0.025, 0.025, 0.025)
