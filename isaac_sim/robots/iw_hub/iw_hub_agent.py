"""Spawn an Idealworks IW Hub and bind its embedded ROS 2 graph."""

import carb
import omni.graph.core as og
import omni.usd
from pxr import Gf, Sdf, UsdGeom

from robots.base_robot import BaseRobotAgent


_SENSORS_REL = "iw_hub_sensors"


def _set_relationship_targets(stage, prim_path, relationship_name, targets):
    relationship = stage.GetPrimAtPath(prim_path).GetRelationship(
        relationship_name
    )
    if relationship:
        relationship.SetTargets([Sdf.Path(target) for target in targets])
    else:
        carb.log_warn(
            f"[IW HUB] missing relationship: "
            f"{prim_path}.{relationship_name}"
        )


def _set_graph_attribute(stage, graph_path, node_name, attribute, value):
    node_prim = stage.GetPrimAtPath(f"{graph_path}/{node_name}")
    if not node_prim.IsValid():
        carb.log_warn(f"[IW HUB] missing graph node: {node_name}")
        return

    graph_attribute = node_prim.GetAttribute(attribute)
    if not graph_attribute:
        carb.log_warn(
            f"[IW HUB] missing graph attribute: {node_name}.{attribute}"
        )
        return
    graph_attribute.Set(value)


def _configure_embedded_graph(stage, graph_path, robot_name):
    """Write topic names before World.reset creates ROS publishers."""

    values = (
        (
            "ros2_subscribe_twist",
            "inputs:topicName",
            f"/{robot_name}/drive_cmd_vel",
        ),
        (
            "ros2_subscribe_joint_state",
            "inputs:topicName",
            f"/{robot_name}/lift_cmd",
        ),
        (
            "ros2_publish_odometry",
            "inputs:topicName",
            f"/{robot_name}/odom",
        ),
        (
            "ros2_publish_odometry",
            "inputs:chassisFrameId",
            f"{robot_name}/base_link",
        ),
        (
            "ros2_publish_odometry",
            "inputs:odomFrameId",
            f"{robot_name}/odom",
        ),
        (
            "ros2_publish_transform_tree",
            "inputs:topicName",
            f"/{robot_name}/tf_model",
        ),
    )
    for node_name, attribute, value in values:
        _set_graph_attribute(
            stage, graph_path, node_name, attribute, value
        )


def _build_fallback_graph(robot_root, robot_name):
    """Create the drive/odom graph if the referenced graph is unavailable."""

    sensor_prim = f"{robot_root}/{_SENSORS_REL}"
    graph_path = f"{robot_root}/ActionGraph"
    graph, _, _, _ = og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": (
                og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
            ),
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("context", "isaacsim.ros2.bridge.ROS2Context"),
                (
                    "sim_time",
                    "isaacsim.core.nodes.IsaacReadSimulationTime",
                ),
                (
                    "twist",
                    "isaacsim.ros2.bridge.ROS2SubscribeTwist",
                ),
                ("linear", "omni.graph.nodes.BreakVector3"),
                ("angular", "omni.graph.nodes.BreakVector3"),
                (
                    "differential",
                    "isaacsim.robot.wheeled_robots."
                    "DifferentialController",
                ),
                (
                    "articulation",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
                (
                    "odometry",
                    "isaacsim.core.nodes.IsaacComputeOdometry",
                ),
                (
                    "publish_odometry",
                    "isaacsim.ros2.bridge.ROS2PublishOdometry",
                ),
            ],
            og.Controller.Keys.CONNECT: [
                ("tick.outputs:tick", "twist.inputs:execIn"),
                ("context.outputs:context", "twist.inputs:context"),
                (
                    "twist.outputs:linearVelocity",
                    "linear.inputs:tuple",
                ),
                (
                    "twist.outputs:angularVelocity",
                    "angular.inputs:tuple",
                ),
                ("tick.outputs:tick", "differential.inputs:execIn"),
                (
                    "linear.outputs:x",
                    "differential.inputs:linearVelocity",
                ),
                (
                    "angular.outputs:z",
                    "differential.inputs:angularVelocity",
                ),
                ("tick.outputs:tick", "articulation.inputs:execIn"),
                (
                    "differential.outputs:velocityCommand",
                    "articulation.inputs:velocityCommand",
                ),
                ("tick.outputs:tick", "odometry.inputs:execIn"),
                (
                    "odometry.outputs:execOut",
                    "publish_odometry.inputs:execIn",
                ),
                (
                    "context.outputs:context",
                    "publish_odometry.inputs:context",
                ),
                (
                    "odometry.outputs:angularVelocity",
                    "publish_odometry.inputs:angularVelocity",
                ),
                (
                    "odometry.outputs:linearVelocity",
                    "publish_odometry.inputs:linearVelocity",
                ),
                (
                    "odometry.outputs:orientation",
                    "publish_odometry.inputs:orientation",
                ),
                (
                    "odometry.outputs:position",
                    "publish_odometry.inputs:position",
                ),
                (
                    "sim_time.outputs:simulationTime",
                    "publish_odometry.inputs:timeStamp",
                ),
            ],
            og.Controller.Keys.SET_VALUES: [
                (
                    "twist.inputs:topicName",
                    f"/{robot_name}/drive_cmd_vel",
                ),
                ("differential.inputs:maxLinearSpeed", 1.2),
                ("differential.inputs:wheelRadius", 0.08),
                ("differential.inputs:wheelDistance", 0.58),
                (
                    "articulation.inputs:jointNames",
                    ["left_wheel_joint", "right_wheel_joint"],
                ),
                (
                    "publish_odometry.inputs:topicName",
                    f"/{robot_name}/odom",
                ),
                (
                    "publish_odometry.inputs:chassisFrameId",
                    f"{robot_name}/base_link",
                ),
                (
                    "publish_odometry.inputs:odomFrameId",
                    f"{robot_name}/odom",
                ),
            ],
        },
    )

    stage = omni.usd.get_context().get_stage()
    _set_relationship_targets(
        stage,
        f"{graph_path}/articulation",
        "inputs:targetPrim",
        [sensor_prim],
    )
    _set_relationship_targets(
        stage,
        f"{graph_path}/odometry",
        "inputs:chassisPrim",
        [sensor_prim],
    )
    return graph


class IwHubAgent(BaseRobotAgent):
    """IW Hub model, drive graph, odometry, and lift command binding."""

    def __init__(self, cfg, world, usd_path):
        super().__init__(cfg, world)
        self.usd_path = str(usd_path)
        self.spawn_xyz = tuple(cfg["spawn_xyz"])
        self.spawn_yaw = float(cfg.get("spawn_yaw", 0.0))
        self.prim_path = f"/World/Robots/{self.name}"
        self.sensor_prim_path = f"{self.prim_path}/{_SENSORS_REL}"

    def setup(self):
        stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(stage, "/World/Robots")

        prim = stage.DefinePrim(self.prim_path, "Xform")
        prim.GetReferences().AddReference(self.usd_path)
        transform = UsdGeom.Xformable(prim)
        transform.ClearXformOpOrder()
        transform.AddTranslateOp().Set(Gf.Vec3d(*self.spawn_xyz))
        transform.AddRotateXYZOp().Set(
            Gf.Vec3f(0.0, 0.0, self.spawn_yaw)
        )
        stage.Load(self.prim_path)

        graph_path = f"{self.sensor_prim_path}/ActionGraph"
        _configure_embedded_graph(stage, graph_path, self.name)
        carb.log_info(
            f"[IW HUB] spawned {self.name} at {self.spawn_xyz}"
        )

    def post_reset(self):
        stage = omni.usd.get_context().get_stage()
        graph_path = f"{self.sensor_prim_path}/ActionGraph"
        if og.get_graph_by_path(graph_path) is not None:
            carb.log_info(f"[IW HUB] embedded graph active: {graph_path}")
            return

        graph_prim = stage.GetPrimAtPath(graph_path)
        if graph_prim.IsValid():
            graph_prim.SetActive(False)
        _build_fallback_graph(self.prim_path, self.name)
        carb.log_warn("[IW HUB] using fallback drive/odom graph")

    def on_physics_step(self, dt):
        del dt

