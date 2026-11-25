# ============================================================
# Isaac Sim + JetBot + RTX LiDAR + ROS 2
#
# ROS 2 topics / TF:
#   /scan       : sensor_msgs/msg/LaserScan
#   /odom       : nav_msgs/msg/Odometry
#   /tf         : odom -> base_link
#   /tf_static              : base_link -> lidar_link
#   /camera/image_raw        : sensor_msgs/msg/Image (RGB)
#   /camera/depth/image_raw  : sensor_msgs/msg/Image (Depth)
#   /camera/camera_info      : sensor_msgs/msg/CameraInfo
#   /cmd_vel                 : geometry_msgs/msg/Twist (SUBSCRIBE)
# ============================================================


# ============================================================
# 0. ROS 2 environment
#    Must be configured BEFORE importing / starting SimulationApp
# ============================================================

import os
import time
import math

ROS2_LIB_PATH = r"C:\isaacsim\exts\isaacsim.ros2.core\humble\lib"

os.environ["ROS_DISTRO"] = "humble"
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["ROS_DOMAIN_ID"] = "0"

# Let Windows find the ROS 2 DLLs bundled with Isaac Sim.
if ROS2_LIB_PATH not in os.environ.get("PATH", ""):
    os.environ["PATH"] = os.environ.get("PATH", "") + os.pathsep + ROS2_LIB_PATH

_dll_directory_handle = None
if hasattr(os, "add_dll_directory"):
    _dll_directory_handle = os.add_dll_directory(ROS2_LIB_PATH)


# ============================================================
# 1. Start Isaac Sim
# ============================================================

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
})


# ============================================================
# 2. Enable ROS 2 Bridge
# ============================================================

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")

# Give the extension manager one update to finish loading.
simulation_app.update()


# ============================================================
# 3. Isaac Sim imports
# ============================================================

import carb
import numpy as np
import omni.replicator.core as rep
import omni.graph.core as og
import omni.usd
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd

from pxr import Gf, Usd, UsdGeom, UsdPhysics

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils

from isaacsim.core.experimental.objects import Cube, DomeLight
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.prims import set_targets
from isaacsim.storage.native import get_assets_root_path
from isaacsim.ros2.core import read_camera_info

from isaacsim.sensors.experimental.rtx import (
    Lidar,
    LidarSensor,
    parse_generic_model_output_data,
)

from omni.replicator.core import Writer


# ============================================================
# 4. Project constants
# ============================================================

JETBOT_PATH = "/World/JetBot"
BASE_PRIM_PATH = "/World/JetBot/chassis"
LIDAR_PRIM_PATH = "/World/JetBot/chassis/Lidar"

ROS2_GRAPH_PATH = "/World/ROS2_Graph"

ROS_DOMAIN_ID = 0

# LiDAR pose relative to chassis.
LIDAR_TRANSLATION = np.array([0.05, 0.0, 0.05])

# Isaac Sim sensor orientation uses W, X, Y, Z.
LIDAR_ORIENTATION_WXYZ = np.array([1.0, 0.0, 0.0, 0.0])

# ROS2PublishRawTransformTree rotation input is X, Y, Z, W.
LIDAR_STATIC_TF_XYZW = [0.0, 0.0, 0.0, 1.0]

LIDAR_CONFIG = "Example_Rotary_2D"
LIDAR_RATE_HZ = 10.0

PHYSICS_RATE_HZ = 60.0

# Differential drive /cmd_vel controller (SI units).
# These are approximate JetBot values; measure your robot for accurate odometry.
CMD_VEL_TOPIC = "/cmd_vel"
WHEEL_RADIUS_M = 0.0325             # wheel radius (m)
WHEEL_SEPARATION_M = 0.1125         # distance between left/right wheel centers (m)
CMD_VEL_TIMEOUT_SEC = 0.5           # stop if the last command is older than this
MAX_LINEAR_SPEED_MPS = 0.25
MAX_ANGULAR_SPEED_RADPS = 2.0
MAX_WHEEL_SPEED_RADPS = 10.0
CMD_LOG_PERIOD_SEC = 2.0

# Optional front-obstacle stop. Off for initial /cmd_vel / SLAM testing.
ENABLE_FORWARD_LIDAR_STOP = False
SAFE_DISTANCE = 1.1

# Writer updates this value continuously.
latest_front_distance = np.inf

# ============================================================
# Camera
# ============================================================

CAMERA_PRIM_PATH = (
    "/World/JetBot/chassis/rgb_camera/jetbot_camera"
)

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240


# ============================================================
# Optional YOLO test asset: YCB banana (visual-only test)
# Source: Isaac Sim Content > Props > YCB > Axis_Aligned > 011_banana.usd
# ============================================================

ENABLE_YOLO_TEST_BANANA = True
BANANA_PRIM_PATH = "/World/YOLO_Test/Banana"
BANANA_RELATIVE_USD = "/Isaac/Props/YCB/Axis_Aligned/011_banana.usd"
# Optional: paste the full Copy URL from Content Browser if asset loading fails.
BANANA_USD_OVERRIDE = ""

# Initial position is computed from the built-in camera's optical direction.
# Values are in meters. The banana is deliberately enlarged for 320x240 YOLO.
BANANA_CAMERA_DISTANCE_M = 0.85
BANANA_VISIBLE_MAX_DIM_M = 0.40
BANANA_MIN_CENTER_Z_M = 0.12



# ============================================================
# 5. Scene setup
# ============================================================

def setup_scene():
    """Create the environment, light, JetBot, and a test obstacle."""

    assets_root_path = get_assets_root_path()

    if assets_root_path is None:
        raise RuntimeError("Could not find Isaac Sim assets root path.")

    # Ground / environment
    environment_path = (
        assets_root_path
        + "/Isaac/Environments/Grid/default_environment.usd"
    )

    stage_utils.add_reference_to_stage(
        usd_path=environment_path,
        path="/World/ground",
    )

    # Light
    dome_light = DomeLight("/World/DomeLight")
    dome_light.set_intensities(1000)

    # JetBot
    jetbot_asset_path = (
        assets_root_path
        + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
    )

    stage_utils.add_reference_to_stage(
        usd_path=jetbot_asset_path,
        path=JETBOT_PATH,
    )

    room_half_size = 5.0
    wall_thickness = 0.2
    wall_height = 2.0


    # 前牆
    wall_front = Cube(
        "/World/wall_front",
        positions=np.array([room_half_size, 0.0, wall_height / 2]),
        scales=np.array([wall_thickness, room_half_size * 2, wall_height])
    )

    # 後牆
    wall_back = Cube(
        "/World/wall_back",
        positions=np.array([-room_half_size, 0.0, wall_height / 2]),
        scales=np.array([wall_thickness, room_half_size * 2, wall_height])
    )

    # 左牆
    wall_left = Cube(
        "/World/wall_left",
        positions=np.array([0.0, room_half_size, wall_height / 2]),
        scales=np.array([room_half_size * 2, wall_thickness, wall_height])
    )

    # 右牆
    wall_right = Cube(
        "/World/wall_right",
        positions=np.array([0.0, -room_half_size, wall_height / 2]),
        scales=np.array([room_half_size * 2, wall_thickness, wall_height])
    )


    # ==================================================
    # Add collision
    # ==================================================

    wall_paths = [
        "/World/wall_front",
        "/World/wall_back",
        "/World/wall_left",
        "/World/wall_right"
    ]

    stage = omni.usd.get_context().get_stage()
    for wall_path in wall_paths:
        wall_prim = stage.GetPrimAtPath(wall_path)
        UsdPhysics.CollisionAPI.Apply(wall_prim)

    return stage


# ============================================================
# 5a. Add a YCB banana in front of the JetBot RGB camera
# ============================================================

def add_yolo_test_banana(stage):
    """Reference a banana USD; position its center in the camera view.

    This is intentionally a *visual detection* test, not a grasping or
    physical-object test. It does not create a rigid body or collider.
    """
    if not ENABLE_YOLO_TEST_BANANA:
        print("[Banana] Disabled by configuration.")
        return

    assets_root = get_assets_root_path()
    if not assets_root:
        print("[Banana] WARNING: Asset root unavailable; skipped banana.")
        return

    camera_prim = stage.GetPrimAtPath(CAMERA_PRIM_PATH)
    if not camera_prim.IsValid():
        print("[Banana] WARNING: Camera prim missing; skipped banana.")
        return

    banana_usd = (
        BANANA_USD_OVERRIDE or (assets_root.rstrip("/") + BANANA_RELATIVE_USD)
    )
    model_path = BANANA_PRIM_PATH + "/Model"

    # A wrapper Xform keeps our test-object transform independent of any
    # transforms authored inside the referenced YCB asset.
    UsdGeom.Xform.Define(stage, "/World/YOLO_Test")
    banana_root = UsdGeom.Xform.Define(stage, BANANA_PRIM_PATH)
    # Do not force a type on the referenced root; it may be a Mesh or Xform.
    model_prim = stage.DefinePrim(model_path)
    model_prim.GetReferences().AddReference(banana_usd)

    # Give USD composition time to resolve a possibly remote USD reference.
    # The Content Browser may fetch this asset on first use.
    mesh_count = 0
    for _ in range(12):
        app_utils.update_app(steps=5)
        mesh_count = sum(
            1 for prim in Usd.PrimRange(model_prim)
            if prim.IsA(UsdGeom.Mesh)
        )
        if mesh_count:
            break

    # USD cameras look in local -Z; transform that direction to world space.
    camera_to_world = UsdGeom.Xformable(camera_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    camera_position = camera_to_world.ExtractTranslation()
    camera_forward = camera_to_world.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
    if camera_forward.GetLength() < 1e-6:
        camera_forward = Gf.Vec3d(1.0, 0.0, 0.0)
    camera_forward.Normalize()

    # World-space center where the object should appear in the camera view.
    center_target = camera_position + camera_forward * BANANA_CAMERA_DISTANCE_M
    center_target = Gf.Vec3d(
        center_target[0],
        center_target[1],
        max(BANANA_MIN_CENTER_Z_M, center_target[2]),
    )

    # Center and scale the referenced object using its actual bounding box.
    # If the remote asset is unavailable, keep safe fallback values and log it.
    model_center = Gf.Vec3d(0.0, 0.0, 0.0)
    model_size_max = 0.0
    try:
        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
            useExtentsHint=True,
        )
        world_range = bbox_cache.ComputeWorldBound(model_prim).ComputeAlignedRange()
        if not world_range.IsEmpty():
            model_center = world_range.GetMidpoint()
            model_size = world_range.GetSize()
            model_size_max = max(model_size[0], model_size[1], model_size[2])
    except Exception as exc:
        print("[Banana] Bounding-box warning:", exc)

    if model_size_max > 1e-5:
        scale = BANANA_VISIBLE_MAX_DIM_M / model_size_max
    else:
        scale = 1.0
        print("[Banana] WARNING: No usable geometry bounds yet. Check asset access.")

    # Translation accounts for any offset authored inside the asset.
    position = center_target - model_center * scale
    transform = UsdGeom.XformCommonAPI(banana_root)
    transform.SetScale(Gf.Vec3f(scale, scale, scale))
    transform.SetTranslate(position)

    if mesh_count == 0:
        print("[Banana] WARNING: No mesh resolved under model. Check USD link.")
    print("[Banana] USD:", banana_usd)
    print("[Banana] Prim:", BANANA_PRIM_PATH)
    print("[Banana] Camera position:", tuple(round(x, 3) for x in camera_position))
    print("[Banana] Target center:", tuple(round(x, 3) for x in center_target))
    print("[Banana] Scale: %.4f | Mesh count: %d" % (scale, mesh_count))
    print("[Banana] Pure visual test: no new rigid body or collision authoring.")


# ============================================================
# 6. Camera render product
# ============================================================

def create_camera_render_product(stage):
    """
    Wait until the built-in JetBot camera prim is available,
    then create one persistent render product for ROS 2 RGB
    and CameraInfo publishers.
    """

    # The JetBot USD reference may need several app updates before
    # nested prims such as jetbot_camera are available.
    app_utils.update_app(steps=10)

    camera_prim = stage.GetPrimAtPath(CAMERA_PRIM_PATH)

    print(
        "Camera prim valid:",
        camera_prim.IsValid()
    )

    if not camera_prim.IsValid():
        raise RuntimeError(
            f"Camera prim not found: {CAMERA_PRIM_PATH}"
        )

    camera_render_product = rep.create.render_product(
        CAMERA_PRIM_PATH,
        resolution=(CAMERA_WIDTH, CAMERA_HEIGHT),
    )

    camera_render_product_path = camera_render_product.path

    print(
        "Camera Render Product:",
        camera_render_product_path
    )

    return camera_render_product, camera_render_product_path



# ============================================================
# 7. RTX LiDAR
# ============================================================

def create_lidar(stage):
    """Create RTX LiDAR and its LidarSensor wrapper."""

    lidar = Lidar.create(
        path=LIDAR_PRIM_PATH,
        config=LIDAR_CONFIG,

        translations=LIDAR_TRANSLATION,
        orientations=LIDAR_ORIENTATION_WXYZ,

        tick_rate=LIDAR_RATE_HZ,
        accumulate_outputs=True,
        aux_output_level="BASIC",

        attributes={
            "omni:sensor:Core:scanRateBaseHz": LIDAR_RATE_HZ,
        },
    )

    print("RTX LiDAR created.")

    # The custom Writer below creates the GenericModelOutput annotator.
    sensor = LidarSensor(
        lidar,
        annotators=[],
    )

    print("LidarSensor created.")

    # Optional diagnostics: print LiDAR range-related attributes.
    lidar_prim = stage.GetPrimAtPath(LIDAR_PRIM_PATH)

    print(f"LiDAR prim valid: {lidar_prim.IsValid()}")

    for attr in lidar_prim.GetAttributes():
        name = attr.GetName()

        if (
            "range" in name.lower()
            or "min" in name.lower()
            or "max" in name.lower()
        ):
            print(name, "=", attr.Get())

    return lidar, sensor


# ============================================================
# 8. ROS 2 Graph
# ============================================================

def setup_ros2_graph(stage):
    """
    Create one ROS 2 graph that publishes:

        /scan
        /odom
        /tf         : odom -> base_link
        /tf_static  : base_link -> lidar_link

    Camera topics are published separately by Replicator Writers:
        /camera/image_raw
        /camera/depth/image_raw
        /camera/camera_info
    """

    keys = og.Controller.Keys

    og.Controller.edit(
        {
            "graph_path": ROS2_GRAPH_PATH,
            "evaluator_name": "execution",
        },
        {
            # ------------------------------------------------
            # Nodes
            # ------------------------------------------------
            keys.CREATE_NODES: [
                (
                    "OnPlaybackTick",
                    "omni.graph.action.OnPlaybackTick",
                ),

                (
                    "ReadWorldPose",
                    "isaacsim.core.nodes.IsaacReadWorldPose",
                ),

                # Derive the chassis linear/angular velocity from PhysX.
                # Previously /odom published pose only; Twist stayed zero.
                (
                    "ComputeOdom",
                    "isaacsim.core.nodes.IsaacComputeOdometry",
                ),

                (
                    "ReadSimTime",
                    "isaacsim.core.nodes.IsaacReadSimulationTime",
                ),

                # /odom
                (
                    "PublishOdom",
                    "isaacsim.ros2.bridge.ROS2PublishOdometry",
                ),

                # Dynamic TF: odom -> base_link
                (
                    "PublishOdomTF",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                ),

                # Static TF: base_link -> lidar_link
                (
                    "PublishLidarStaticTF",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                ),

                # RTX LiDAR Render Product
                (
                    "CreateRenderProduct",
                    "isaacsim.core.nodes.IsaacCreateRenderProduct",
                ),

                # /scan
                (
                    "PublishScan",
                    "isaacsim.ros2.bridge.ROS2RtxLidarHelper",
                ),

                (
                    "PublishClock",
                    "isaacsim.ros2.bridge.ROS2PublishClock",
                ),

            ],

            # ------------------------------------------------
            # Values
            # ------------------------------------------------
            keys.SET_VALUES: [

                # ========== /odom ==========
                (
                    "PublishOdom.inputs:chassisFrameId",
                    "base_link",
                ),
                (
                    "PublishOdom.inputs:odomFrameId",
                    "odom",
                ),
                (
                    "PublishOdom.inputs:topicName",
                    "odom",
                ),
                (
                    "PublishOdom.inputs:queueSize",
                    10,
                ),
                (
                    "PublishOdom.inputs:robotFront",
                    [1.0, 0.0, 0.0],
                ),

                # ========== Dynamic TF: odom -> base_link ==========
                (
                    "PublishOdomTF.inputs:parentFrameId",
                    "odom",
                ),
                (
                    "PublishOdomTF.inputs:childFrameId",
                    "base_link",
                ),
                (
                    "PublishOdomTF.inputs:topicName",
                    "tf",
                ),
                (
                    "PublishOdomTF.inputs:queueSize",
                    10,
                ),
                (
                    "PublishOdomTF.inputs:staticPublisher",
                    False,
                ),

                # ========== Static TF: base_link -> lidar_link ==========
                (
                    "PublishLidarStaticTF.inputs:parentFrameId",
                    "base_link",
                ),
                (
                    "PublishLidarStaticTF.inputs:childFrameId",
                    "lidar_link",
                ),
                (
                    "PublishLidarStaticTF.inputs:topicName",
                    "tf_static",
                ),
                (
                    "PublishLidarStaticTF.inputs:queueSize",
                    10,
                ),
                (
                    "PublishLidarStaticTF.inputs:staticPublisher",
                    True,
                ),
                (
                    "PublishLidarStaticTF.inputs:translation",
                    LIDAR_TRANSLATION.tolist(),
                ),
                (
                    "PublishLidarStaticTF.inputs:rotation",
                    LIDAR_STATIC_TF_XYZW,
                ),

                # ========== RTX LiDAR Render Product ==========
                (
                    "CreateRenderProduct.inputs:cameraPrim",
                    LIDAR_PRIM_PATH,
                ),
                (
                    "CreateRenderProduct.inputs:width",
                    1,
                ),
                (
                    "CreateRenderProduct.inputs:height",
                    1,
                ),

                # ========== /scan ==========
                (
                    "PublishScan.inputs:topicName",
                    "scan",
                ),
                (
                    "PublishScan.inputs:frameId",
                    "lidar_link",
                ),
                (
                    "PublishScan.inputs:type",
                    "laser_scan",
                ),

            ],

            # ------------------------------------------------
            # Connections
            # ------------------------------------------------
            keys.CONNECT: [

                # Evaluate chassis odometry every simulation playback tick.
                (
                    "OnPlaybackTick.outputs:tick",
                    "ComputeOdom.inputs:execIn",
                ),
                # Publish only after ComputeOdom has evaluated for this tick.
                (
                    "ComputeOdom.outputs:execOut",
                    "PublishOdom.inputs:execIn",
                ),

                # chassis pose -> /odom
                (
                    "ReadWorldPose.outputs:translation",
                    "PublishOdom.inputs:position",
                ),
                (
                    "ReadWorldPose.outputs:orientation",
                    "PublishOdom.inputs:orientation",
                ),
                # The Odometry message's twist is velocity in base_link.
                # IsaacComputeOdometry provides these dynamic values.
                (
                    "ComputeOdom.outputs:linearVelocity",
                    "PublishOdom.inputs:linearVelocity",
                ),
                (
                    "ComputeOdom.outputs:angularVelocity",
                    "PublishOdom.inputs:angularVelocity",
                ),

                # Dynamic TF execution
                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishOdomTF.inputs:execIn",
                ),

                # chassis pose -> odom -> base_link TF
                (
                    "ReadWorldPose.outputs:translation",
                    "PublishOdomTF.inputs:translation",
                ),
                (
                    "ReadWorldPose.outputs:orientation",
                    "PublishOdomTF.inputs:rotation",
                ),

                # Static TF execution
                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishLidarStaticTF.inputs:execIn",
                ),

                # Simulation timestamps
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishOdom.inputs:timeStamp",
                ),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishOdomTF.inputs:timeStamp",
                ),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishLidarStaticTF.inputs:timeStamp",
                ),

                # RTX LiDAR -> /scan
                (
                    "OnPlaybackTick.outputs:tick",
                    "CreateRenderProduct.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:execOut",
                    "PublishScan.inputs:execIn",
                ),
                (
                    "CreateRenderProduct.outputs:renderProductPath",
                    "PublishScan.inputs:renderProductPath",
                ),

                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishClock.inputs:execIn",
                ),

                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishClock.inputs:timeStamp",
                ),

            ],
        },
    )

    # ReadWorldPose.inputs:prim is a USD target relationship,
    # so set it with set_targets() instead of SET_VALUES.
    read_pose_prim = stage.GetPrimAtPath(
        f"{ROS2_GRAPH_PATH}/ReadWorldPose"
    )

    set_targets(
        prim=read_pose_prim,
        attribute="inputs:prim",
        target_prim_paths=[
            BASE_PRIM_PATH,
        ],
    )

    # IsaacComputeOdometry also needs to know which rigid body to read.
    # chassisPrim is a USD relationship, not a plain string input.
    compute_odom_prim = stage.GetPrimAtPath(
        f"{ROS2_GRAPH_PATH}/ComputeOdom"
    )
    if not compute_odom_prim.IsValid():
        raise RuntimeError("ComputeOdom OmniGraph node was not created")

    set_targets(
        prim=compute_odom_prim,
        attribute="inputs:chassisPrim",
        target_prim_paths=[BASE_PRIM_PATH],
    )

    # Fail early if the target is not established instead of publishing
    # apparently valid /odom messages with permanent zero velocity.
    # USD relationship targets are queried through GetRelationship, not GetAttribute.
    chassis_rel = compute_odom_prim.GetRelationship("inputs:chassisPrim")
    if not chassis_rel.IsValid() or not chassis_rel.GetTargets():
        raise RuntimeError("ComputeOdom chassisPrim target is missing")
    print("[odom] ComputeOdom chassis target:", chassis_rel.GetTargets())

    print("ROS 2 graph created.")
    print("  /scan")
    print("  /odom      : pose + PhysX linear/angular velocity")
    print("  /tf        : odom -> base_link")
    print("  /tf_static : base_link -> lidar_link")
    print("Camera topics are published by Replicator Writers.")
    print("/cmd_vel is received separately by the rclpy CmdVelReceiver.")


# ============================================================
# 9. LiDAR Generic Model Output helper
# ============================================================

def get_front_distance(gmo, front_angle_deg=10.0):
    """
    Return the closest valid LiDAR range in the forward angular sector.

    In GenericModelOutput used here:
        gmo.x -> azimuth
        gmo.z -> range
    """

    if gmo.numElements == 0:
        return np.inf

    azimuth = gmo.x
    distance = gmo.z

    valid_mask = (
        np.isfinite(distance)
        & (distance > 0.0)
    )

    front_mask = (
        valid_mask
        & (np.abs(azimuth) <= front_angle_deg)
    )

    front_distances = distance[front_mask]

    if len(front_distances) == 0:
        return np.inf

    return np.min(front_distances)


# ============================================================
# 10. Custom LiDAR Writer
# ============================================================

class JetBotLidarWriter(Writer):

    def __init__(self):

        self.data_structure = "renderProduct"

        self.annotators = [
            rep.annotators.get("GenericModelOutput"),
        ]

        self._frame_count = 0
        self._last_print_frame = -1

    def write(self, data):

        global latest_front_distance

        if "renderProducts" not in data:
            return

        for _, rp_data in data["renderProducts"].items():

            gmo_raw = rp_data.get("GenericModelOutput")

            if gmo_raw is None:
                continue

            if isinstance(gmo_raw, dict):
                gmo_raw = gmo_raw.get("data")

            if gmo_raw is None:
                continue

            gmo = parse_generic_model_output_data(gmo_raw)

            if gmo.numElements <= 0:
                latest_front_distance = np.inf
                continue

            latest_front_distance = get_front_distance(gmo)

            # Print diagnostics roughly every 60 callbacks.
            if self._frame_count - self._last_print_frame >= 60:

                self._last_print_frame = self._frame_count

                print("\n======================================")
                print("LiDAR GMO")
                print("======================================")

                print("numElements:", gmo.numElements)
                print("scanComplete:", gmo.scanComplete)
                print("frameOfReference:", gmo.frameOfReference)

                print(
                    "x range:",
                    np.min(gmo.x),
                    np.max(gmo.x),
                )
                print(
                    "y range:",
                    np.min(gmo.y),
                    np.max(gmo.y),
                )
                print(
                    "z range:",
                    np.min(gmo.z),
                    np.max(gmo.z),
                )

                sample_count = min(5, gmo.numElements)

                print("\nFirst 5 points:")

                for i in range(sample_count):
                    print(
                        f"Point {i}: "
                        f"x={gmo.x[i]:.3f}, "
                        f"y={gmo.y[i]:.3f}, "
                        f"z={gmo.z[i]:.3f}"
                    )

        self._frame_count += 1


def attach_lidar_writer(sensor):
    """Register and attach the custom GMO Writer."""

    rep.WriterRegistry.register(
        JetBotLidarWriter
    )

    sensor.attach_writer(
        "JetBotLidarWriter"
    )

    print("JetBotLidarWriter attached.")


# ============================================================
# 11. Robot helpers
# ============================================================

def create_jetbot_articulation():

    jetbot = Articulation(
        JETBOT_PATH
    )

    print(
        "Number of DOFs:",
        jetbot.num_dofs,
    )

    print(
        "DOF names:",
        jetbot.dof_names,
    )

    wheel_indices = jetbot.get_dof_indices(
        [
            "left_wheel_joint",
            "right_wheel_joint",
        ]
    ).numpy()

    print(
        "Wheel indices:",
        wheel_indices,
    )

    return jetbot, wheel_indices


def get_base_yaw(stage):
    """
    Read JetBot chassis world orientation and return yaw in radians.
    This makes the TURN state usable without relying on an undefined variable.
    """

    base_prim = stage.GetPrimAtPath(
        BASE_PRIM_PATH
    )

    if not base_prim.IsValid():
        return 0.0

    xform = UsdGeom.Xformable(base_prim)

    matrix = xform.ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )

    quat = matrix.ExtractRotationQuat()

    w = float(quat.GetReal())
    imag = quat.GetImaginary()

    x = float(imag[0])
    y = float(imag[1])
    z = float(imag[2])

    yaw = np.arctan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )

    return yaw

def create_camera_writers(camera_render_product_path):
    """
    Publish RGB, depth, and CameraInfo from the same JetBot camera
    Render Product using Isaac Sim Replicator ROS 2 Writers.

    Topics:
        /camera/image_raw
        /camera/depth/image_raw
        /camera/camera_info
    """

    # --------------------------------------------------------
    # CameraInfo
    # --------------------------------------------------------
    camera_info, _ = read_camera_info(
        render_product_path=camera_render_product_path
    )

    print("\n===== Camera Info =====")
    print("width :", camera_info.width)
    print("height:", camera_info.height)
    print("K:")
    print(camera_info.k)
    print("=======================\n")

    camera_info_writer = rep.writers.get(
        "ROS2PublishCameraInfo"
    )

    camera_info_writer.initialize(
        frameId="camera_link",
        nodeNamespace="",
        queueSize=1,
        topicName="camera/camera_info",
        width=camera_info.width,
        height=camera_info.height,
        projectionType=camera_info.distortion_model,
        k=camera_info.k.reshape((1, 9)),
        r=camera_info.r.reshape((1, 9)),
        p=camera_info.p.reshape((1, 12)),
        physicalDistortionModel=camera_info.distortion_model,
        physicalDistortionCoefficients=camera_info.d,
    )

    camera_info_writer.attach(
        [camera_render_product_path]
    )

    # --------------------------------------------------------
    # RGB Image
    # --------------------------------------------------------
    rgb_render_var = (
        omni.syntheticdata.SyntheticData
        .convert_sensor_type_to_rendervar(
            sd.SensorType.Rgb.name
        )
    )

    rgb_writer = rep.writers.get(
        rgb_render_var + "ROS2PublishImage"
    )

    rgb_writer.initialize(
        frameId="camera_link",
        nodeNamespace="",
        queueSize=1,
        topicName="camera/image_raw",
    )

    rgb_writer.attach(
        [camera_render_product_path]
    )

    # --------------------------------------------------------
    # Depth Image
    # --------------------------------------------------------
    depth_render_var = (
        omni.syntheticdata.SyntheticData
        .convert_sensor_type_to_rendervar(
            sd.SensorType.DistanceToImagePlane.name
        )
    )

    depth_writer = rep.writers.get(
        depth_render_var + "ROS2PublishImage"
    )

    depth_writer.initialize(
        frameId="camera_link",
        nodeNamespace="",
        queueSize=10,
        topicName="camera/depth/image_raw",
    )

    #depth_writer.attach(
    #    [camera_render_product_path]
    #)

    print("ROS 2 camera Writers attached:")
    print("  /camera/image_raw")
    print("  /camera/depth/image_raw")
    print("  /camera/camera_info")

    # Keep references alive for the application lifetime.
    return {
        "rgb": rgb_writer,
        "depth": depth_writer,
        "camera_info": camera_info_writer,
    }


# ============================================================
# 12. ROS 2 /cmd_vel subscriber + differential-drive conversion
# ============================================================

class CmdVelReceiver:
    """Receive Twist with rclpy and use wall-time freshness for safe stopping.

    Note: 'rclpy' must be importable inside Isaac Sim's Python environment.
    The original ROS 2 Bridge publishers remain in the existing OmniGraph.
    """

    def __init__(self):
        try:
            import rclpy
            from geometry_msgs.msg import Twist
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        except ImportError as exc:
            raise RuntimeError(
                "Unable to import rclpy / geometry_msgs in Isaac Sim Python. "
                "This Windows Python environment may only have the C++ ROS2 Bridge. "
                "Check whether ROS2 Python packages are available for its Python "
                "version. Do not install Linux ROS2 wheels into Windows Python. "
                "A ROS2SubscribeTwist OmniGraph fallback would be needed."
            ) from exc

        self.rclpy = rclpy
        self._owns_rclpy = not rclpy.ok()
        if self._owns_rclpy:
            rclpy.init(args=None)

        self.node = rclpy.create_node("isaac_jetbot_cmd_vel")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.node.create_subscription(
            Twist, CMD_VEL_TOPIC, self._callback, qos
        )

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_received = None
        self.received_count = 0

    def _callback(self, msg):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)
        if not (math.isfinite(vx) and math.isfinite(wz)):
            print("[cmd_vel] Ignoring non-finite velocity command")
            return

        self.linear_x = float(np.clip(
            vx, -MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS
        ))
        self.angular_z = float(np.clip(
            wz, -MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS
        ))
        self.last_received = time.monotonic()
        self.received_count += 1

    def spin_once(self):
        # Never block the Isaac Sim simulation / rendering loop.
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def current_velocity(self):
        if self.last_received is None:
            return 0.0, 0.0, "waiting"
        if time.monotonic() - self.last_received > CMD_VEL_TIMEOUT_SEC:
            return 0.0, 0.0, "timeout"
        return self.linear_x, self.angular_z, "active"

    def close(self):
        self.node.destroy_node()
        if self._owns_rclpy and self.rclpy.ok():
            self.rclpy.shutdown()


def twist_to_wheel_velocities(vx, wz):
    """Differential drive: (vx [m/s], wz [rad/s]) -> (left, right) [rad/s]."""
    half_track = 0.5 * WHEEL_SEPARATION_M
    left = (vx - wz * half_track) / WHEEL_RADIUS_M
    right = (vx + wz * half_track) / WHEEL_RADIUS_M
    wheel_speeds = np.array([left, right], dtype=np.float64)

    # Preserve curvature when clipping wheel speed limits.
    peak = float(np.max(np.abs(wheel_speeds)))
    if peak > MAX_WHEEL_SPEED_RADPS:
        wheel_speeds *= (MAX_WHEEL_SPEED_RADPS / peak)

    return wheel_speeds.reshape(1, 2)


# ============================================================
# 13. Main
# ============================================================
import traceback


def main():
    cmd_receiver = None
    jetbot = None
    wheel_indices = None

    try:
        # ----------------------------------------------------
        # Scene / sensors / ROS2 topics
        # ----------------------------------------------------
        stage = setup_scene()
        _, camera_render_product_path = create_camera_render_product(stage)
        add_yolo_test_banana(stage)
        _, sensor = create_lidar(stage)
        setup_ros2_graph(stage)
        camera_writers = create_camera_writers(camera_render_product_path)
        attach_lidar_writer(sensor)

        # ----------------------------------------------------
        # /cmd_vel subscription
        # ----------------------------------------------------
        cmd_receiver = CmdVelReceiver()
        print("[cmd_vel] Subscribed:", CMD_VEL_TOPIC)
        print("[cmd_vel] Timeout: %.2f s" % CMD_VEL_TIMEOUT_SEC)
        print("[cmd_vel] Wheel radius: %.4f m" % WHEEL_RADIUS_M)
        print("[cmd_vel] Wheel separation: %.4f m" % WHEEL_SEPARATION_M)
        print("[cmd_vel] Robot will remain stopped until a Twist arrives.")

        # ----------------------------------------------------
        # Physics
        # ----------------------------------------------------
        SimulationManager.setup_simulation(
            dt=1.0 / PHYSICS_RATE_HZ,
            device="cpu",
        )
        app_utils.play()
        app_utils.update_app(steps=20)

        jetbot, wheel_indices = create_jetbot_articulation()

        # Original FORWARD/TURN autonomous FSM removed on purpose:
        # /cmd_vel is now the *only* controller of the wheel joints.
        zero_wheels = np.zeros((1, 2), dtype=np.float64)
        jetbot.set_dof_velocity_targets(
            zero_wheels, dof_indices=wheel_indices
        )

        prev_log = 0.0
        prev_state = None
        print("[cmd_vel] Ready. Publish Twist from ROS2/WSL at 10 Hz.")

        # ----------------------------------------------------
        # Simulation loop
        # ----------------------------------------------------
        while simulation_app.is_running():
            simulation_app.update()
            cmd_receiver.spin_once()
            vx, wz, state = cmd_receiver.current_velocity()

            # Optional safety check: only prevents forward driving.
            if (
                ENABLE_FORWARD_LIDAR_STOP
                and vx > 0.0
                and latest_front_distance < SAFE_DISTANCE
            ):
                vx = 0.0
                state = "lidar_stop"

            wheel_velocities = twist_to_wheel_velocities(vx, wz)
            jetbot.set_dof_velocity_targets(
                wheel_velocities,
                dof_indices=wheel_indices,
            )

            now = time.monotonic()
            if state != prev_state or now - prev_log >= CMD_LOG_PERIOD_SEC:
                print(
                    "[cmd_vel] state=%s received=%d vx=%+.3f m/s "
                    "wz=%+.3f rad/s wheel=[%+.3f, %+.3f] rad/s"
                    % (
                        state,
                        cmd_receiver.received_count,
                        vx,
                        wz,
                        wheel_velocities[0, 0],
                        wheel_velocities[0, 1],
                    )
                )
                prev_log = now
                prev_state = state

    except Exception as exc:
        print("\n===== PROGRAM ERROR =====")
        print(exc)
        traceback.print_exc()
        print("=========================\n")

    finally:
        # Stop wheels first, even if /cmd_vel was previously nonzero.
        if jetbot is not None and wheel_indices is not None:
            try:
                jetbot.set_dof_velocity_targets(
                    np.zeros((1, 2), dtype=np.float64),
                    dof_indices=wheel_indices,
                )
                app_utils.update_app(steps=2)
            except Exception:
                pass

        if cmd_receiver is not None:
            try:
                cmd_receiver.close()
            except Exception as exc:
                print("[cmd_vel] ROS2 shutdown warning:", exc)

        try:
            app_utils.stop()
        except Exception:
            pass

        simulation_app.close()


if __name__ == "__main__":
    main()
