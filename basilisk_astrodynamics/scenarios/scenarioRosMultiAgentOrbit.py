# SPDX-License-Identifier: Apache-2.0
import argparse

from Basilisk.simulation import spacecraft, thrusterDynamicEffector, simSynch, simpleNav
from Basilisk.utilities import SimulationBaseClass, macros, vizSupport, simIncludeThruster, simIncludeGravBody, orbitalMotion
from Basilisk.fswAlgorithms import thrFiringSchmitt, hillStateConverter, hillPoint, attTrackingError
from Basilisk.architecture import bskLogging, sysModel, messaging
from Basilisk.utilities.simHelpers import np2EigenMatrix3d
from bsk_ros2_bridge import RosBridgeHandler
from bsk_ros2_bridge.modules import forceTorqueThrForceMapping
from examples.utils.tools import get_initial_conditions_from_hill, get_hill_frame_attitude

def run(liveStream=False, broadcastStream=False, simTime=60.0, simRate=100.0, thrRate=10.0, vizRate=30.0, accelFactor=1.0):
    # --- Set up simulation classes and processes ---
    scSim = SimulationBaseClass.SimBaseClass()
    simTaskName = "simTask"
    thrTaskName = "thrTask"
    vizTaskName = "vizTask"
    simProcess = scSim.CreateNewProcess("simProcess", 200)
    thrProcess = scSim.CreateNewProcess("thrProcess", 100)
    vizProcess = scSim.CreateNewProcess("vizProcess", 50)
    simTimeStep = macros.sec2nano(1.0 / simRate)
    simProcess.addTask(scSim.CreateNewTask(simTaskName, simTimeStep))
    thrTimeStep = macros.sec2nano(1.0 / thrRate)
    thrProcess.addTask(scSim.CreateNewTask(thrTaskName, thrTimeStep))
    vizTimeStep = macros.sec2nano(accelFactor / vizRate)
    vizProcess.addTask(scSim.CreateNewTask(vizTaskName, vizTimeStep))

    # --- Set up clock synchronization module ---
    clockSync = simSynch.ClockSynch()
    clockSync.accelFactor = accelFactor

    # --- Create the ROS 2 bridge handler ---
    ros_bridge = RosBridgeHandler(accelFactor=accelFactor, requireBridge=True, clockSync=clockSync)
    ros_bridge.ModelTag = "ros_bridge"
    ros_bridge.bskLogger = sysModel.BSKLogger(bskLogging.BSK_DEBUG)

    # --- Set up gravitational bodies and orbit ---
    # Create gravitational bodies
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True

    # Orbit initial conditions
    mu = planet.mu
    oe = orbitalMotion.ClassicElements()
    oe.a = (6378 + 400) * 1000 # m, Semi-major axis
    oe.e = 0.001
    oe.i = 60 * macros.D2R
    oe.Omega = 270 * macros.D2R
    oe.omega = 90 * macros.D2R
    oe.f = 5 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)

    # Create virtual spacecraft to define local Hill frame reference
    scHillObj = spacecraft.Spacecraft()
    scHillObj.ModelTag = "bskSatHill"
    scHillObj.hub.r_CN_NInit = rN  # m   - r_BN_N
    scHillObj.hub.v_CN_NInit = vN  # m/s - v_BN_N
    hillFrameMRP = get_hill_frame_attitude(rN, vN)
    scHillObj.hub.sigma_BNInit = hillFrameMRP
    gravFactory.addBodiesTo(scHillObj)
    hillNavObj = simpleNav.SimpleNav()
    hillNavObj.ModelTag = "chiefNav"
    hillPointObj = hillPoint.hillPoint()
    hillPointObj.ModelTag = "hillPoint"
    celBodyData = messaging.EphemerisMsgPayload()
    celBodyInMsg = messaging.EphemerisMsg().write(celBodyData)

    # --- Set up all spacecraft ---
    # Number of spacecraft are defined by relative positions in Hill frame
    relative_positions = [
        [0, 0, 0],
        [0, 3, 0],
        [0, -3, 0]
    ]
    num_spacecraft = len(relative_positions)

    # Shared spacecraft inertial properties
    m = 12.0  # kg, spacecraft mass
    I = [0.100, 0, 0, 0, 0.130, 0, 0, 0, 0.050]  # kg m^2, inertia tensor

    # Box dimensions are 0.2 x 0.1 x 0.3 m, centered at the body origin.
    thruster_offset_x, thruster_offset_y, thruster_offset_z = 0.1, 0.05, 0.15

    # Thrusters are placed on the box surface with opposing pairs for force and torque control.
    thruster_defs = [
        ([0, 0, thruster_offset_z], [1, 0, 0]),
        ([0, 0, thruster_offset_z], [-1, 0, 0]),
        ([0, 0, -thruster_offset_z], [1, 0, 0]),
        ([0, 0, -thruster_offset_z], [-1, 0, 0]),
        ([thruster_offset_x, 0, 0], [0, 1, 0]),
        ([thruster_offset_x, 0, 0], [0, -1, 0]),
        ([-thruster_offset_x, 0, 0], [0, 1, 0]),
        ([-thruster_offset_x, 0, 0], [0, -1, 0]),
        ([0, thruster_offset_y, 0], [0, 0, 1]),
        ([0, thruster_offset_y, 0], [0, 0, -1]),
        ([0, -thruster_offset_y, 0], [0, 0, 1]),
        ([0, -thruster_offset_y, 0], [0, 0, -1]),
    ]

    # Loop over each spacecraft to set up properties and modules
    scObject = []
    thFactory = []
    thrusterSet = []
    thrFiringSchmittObj = []
    thrConfigMsg = []
    hillStateNavObj = []
    scNavObj = []
    thrForceMapping = []
    attTrackObj = []
    for i in range(num_spacecraft):
        scObject_i = spacecraft.Spacecraft()
        scObject_i.ModelTag = f"bskSat{i}"
        scObject.append(scObject_i)
        gravFactory.addBodiesTo(scObject_i)
        scObject_i.hub.mHub = m
        scObject_i.hub.IHubPntBc_B = np2EigenMatrix3d(I)
        r, v = get_initial_conditions_from_hill(mu, rN, vN, relative_positions[i], [0, 0, 0])
        scObject_i.hub.r_CN_NInit = r
        scObject_i.hub.v_CN_NInit = v
        hillFrameMRP = get_hill_frame_attitude(rN, vN)
        scObject_i.hub.sigma_BNInit = hillFrameMRP

        # Setup thrusters for spacecraft
        thrusterSet_i = thrusterDynamicEffector.ThrusterDynamicEffector()
        thrusterSet.append(thrusterSet_i)
        thFactory_i = simIncludeThruster.thrusterFactory()
        for loc, dir in thruster_defs:
            thFactory_i.create(
                'MOOG_Monarc_1',
                loc,
                dir,
                MaxThrust=1.0,
                cutoffFrequency=3141.6,
                MinOnTime=0.0,
            )
        thFactory_i.addToSpacecraft(scObject_i.ModelTag, thrusterSet_i, scObject_i)
        thFactory.append(thFactory_i)
        thrConfigMsg_i = thFactory_i.getConfigMessage()
        thrConfigMsg.append(thrConfigMsg_i)

        # Setup force/torque to thruster mapping
        thrForceMapping_i = forceTorqueThrForceMapping()
        thrForceMapping_i.ModelTag = f"thrForceMapping{i}"
        thrForceMapping_i.setThrusterGeometryFromDefs(thruster_defs)
        thrForceMapping.append(thrForceMapping_i)

        # Setup ROS bridge - Set up subscribers and publishers
        ros_bridge.add_ros_subscriber('CmdForceBodyMsgPayload', 'CmdForceBodyMsgOut', 'cmd_force', f'bskSat{i}')
        ros_bridge.add_ros_subscriber('CmdTorqueBodyMsgPayload', 'CmdTorqueBodyMsgOut', 'cmd_torque', f'bskSat{i}')
        ros_bridge.add_ros_publisher('SCStatesMsgPayload', 'SCStatesMsgIn', 'sc_states', f'bskSat{i}', max_rate=10)
        ros_bridge.add_ros_publisher('THRArrayCmdForceMsgPayload', 'THRArrayCmdForceMsgIn', 'thr_array_cmd_force', f'bskSat{i}', max_rate=10)
        ros_bridge.add_ros_publisher('HillRelStateMsgPayload', 'HillRelStateMsgIn', 'hill_trans_state', f'bskSat{i}', max_rate=20)
        ros_bridge.add_ros_publisher('AttGuidMsgPayload', 'AttGuidMsgIn', 'hill_rot_state', f'bskSat{i}', max_rate=20)

        # Setup the Schmitt trigger thruster firing logic module
        thrFiringSchmittObj_i = thrFiringSchmitt.thrFiringSchmitt()
        thrFiringSchmittObj_i.ModelTag = f"thrFiringSchmitt{i}"
        thrFiringSchmittObj_i.thrMinFireTime = 0.0
        thrFiringSchmittObj_i.level_on = 0.998
        thrFiringSchmittObj_i.level_off = 0.002
        thrFiringSchmittObj.append(thrFiringSchmittObj_i)

        # Navigation modules with proper priorities to avoid NavTransMsg errors
        scNavObj_i = simpleNav.SimpleNav()
        scNavObj_i.ModelTag = f"nav{i}"
        scNavObj.append(scNavObj_i)

        hillStateNavObj_i = hillStateConverter.hillStateConverter()
        hillStateNavObj_i.ModelTag = f"hillStateNavObj{i}"
        hillStateNavObj.append(hillStateNavObj_i)

        # Module to compute attitude tracking error sigma_B/R --> R is Hill-frame in our case.
        attTrackObj_i = attTrackingError.attTrackingError()
        attTrackObj_i.ModelTag = f"attTrackObj{i}"
        attTrackObj.append(attTrackObj_i)

    # Vehicle configuration message (shared)
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    # --- Connect messages ---
    hillNavObj.scStateInMsg.subscribeTo(scHillObj.scStateOutMsg)
    hillPointObj.transNavInMsg.subscribeTo(hillNavObj.transOutMsg)
    hillPointObj.celBodyInMsg.subscribeTo(celBodyInMsg)

    for i in range(num_spacecraft):
        # Get ROS bridge spacecraft interface
        ros_bridge_i = getattr(ros_bridge, f'bskSat{i}')

        # Connect spacecraft state messages using namespace syntax
        ros_bridge_i.SCStatesMsgIn.subscribeTo(scObject[i].scStateOutMsg)
        ros_bridge_i.THRArrayCmdForceMsgIn.subscribeTo(thrForceMapping[i].thrForceCmdOutMsg)
        ros_bridge_i.HillRelStateMsgIn.subscribeTo(hillStateNavObj[i].hillStateOutMsg)
        ros_bridge_i.AttGuidMsgIn.subscribeTo(attTrackObj[i].attGuidOutMsg)

        # Connect force/torque mapping
        thrForceMapping[i].thrConfigInMsg.subscribeTo(thrConfigMsg[i])
        thrForceMapping[i].vehConfigInMsg.subscribeTo(vcMsg)
        thrForceMapping[i].cmdForceInMsg.subscribeTo(ros_bridge_i.CmdForceBodyMsgOut)
        thrForceMapping[i].cmdTorqueInMsg.subscribeTo(ros_bridge_i.CmdTorqueBodyMsgOut)

        # Connect thruster logic
        thrFiringSchmittObj[i].thrConfInMsg.subscribeTo(thrConfigMsg[i])
        thrFiringSchmittObj[i].thrForceInMsg.subscribeTo(thrForceMapping[i].thrForceCmdOutMsg)
        thrusterSet[i].cmdsInMsg.subscribeTo(thrFiringSchmittObj[i].onTimeOutMsg)

        # Navigation connections
        scNavObj[i].scStateInMsg.subscribeTo(scObject[i].scStateOutMsg)
        hillStateNavObj[i].depStateInMsg.subscribeTo(scNavObj[i].transOutMsg)
        hillStateNavObj[i].chiefStateInMsg.subscribeTo(hillNavObj.transOutMsg)
        attTrackObj[i].attNavInMsg.subscribeTo(scNavObj[i].attOutMsg)
        attTrackObj[i].attRefInMsg.subscribeTo(hillPointObj.attRefOutMsg)

    # --- Add models to simulation tasks ---
    # Higher priority value -> earlier execution in the same sim step
    scSim.AddModelToTask(simTaskName, clockSync, 1000)
    scSim.AddModelToTask(simTaskName, scHillObj, 92)
    scSim.AddModelToTask(simTaskName, hillNavObj, 91)
    scSim.AddModelToTask(simTaskName, hillPointObj, 90)
    scSim.AddModelToTask(simTaskName, ros_bridge, 1)
    for i in range(num_spacecraft):
        scSim.AddModelToTask(simTaskName, thrusterSet[i], 80)
        scSim.AddModelToTask(simTaskName, scObject[i], 60)
        scSim.AddModelToTask(simTaskName, scNavObj[i], 40)
        scSim.AddModelToTask(simTaskName, hillStateNavObj[i], 35)
        scSim.AddModelToTask(simTaskName, attTrackObj[i], 30)

        scSim.AddModelToTask(thrTaskName, thrForceMapping[i], 20)
        scSim.AddModelToTask(thrTaskName, thrFiringSchmittObj[i], 10)

    # --- Set up Vizard support ---
    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(
            scSim,
            vizTaskName,
            [scHillObj] + scObject,
            liveStream=liveStream,
            broadcastStream=broadcastStream,
        )

        # Add marker for Hill frame origin
        vizSupport.createCustomModel(viz, simBodiesToModify=[scHillObj.ModelTag],
                                    modelPath='SPHERE', scale=[0.05]*3)

        # Add custom models for each spacecraft entry
        for scObject_i in scObject:
            vizSupport.createCustomModel(viz, simBodiesToModify=[scObject_i.ModelTag],
                                        modelPath='6USat')

    # --- Run the simulation ---
    scSim.InitializeSimulation()
    incrementalStopTime = 0
    while incrementalStopTime < macros.sec2nano(simTime):
        incrementalStopTime += simTimeStep
        scSim.ConfigureStopTime(incrementalStopTime)
        scSim.ExecuteSimulation()
    return

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the multi-agent Basilisk orbit scenario.")
    parser.add_argument(
        "--stream-mode",
        choices=("live", "broadcast"),
        default="live",
        help="Vizard stream mode (default: live).",
    )
    args = parser.parse_args()

    run(
        liveStream=args.stream_mode == "live",
        broadcastStream=args.stream_mode == "broadcast",
        simTime=1e6,
        simRate=200.0,
        accelFactor=1.0,
    )
