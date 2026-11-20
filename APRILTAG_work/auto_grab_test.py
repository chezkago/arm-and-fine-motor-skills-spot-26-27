import json
import math
import time
from pathlib import Path

import bosdyn.client
import bosdyn.client.util
from bosdyn.api import world_object_pb2
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, get_a_tform_b
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, block_until_arm_arrives
from bosdyn.client.world_object import WorldObjectClient

REGISTRY_PATH = Path(__file__).resolve().parent / "APRILTAG_Registry.json"

# --- Calibration & Reach Settings ---
# Measure ONLY the black outer square of Tag 2 on your iPad in mm (4.5 in = 114.3 mm).
# If Spot's Admin Console is still set to default 146 mm, this corrects the 3D distance in Python.
ACTUAL_TAG_MM = 114.3
ADMIN_CONSOLE_TAG_MM = 146.0
TAG_SCALE = ACTUAL_TAG_MM / ADMIN_CONSOLE_TAG_MM

MIN_REACH_X = 0.35
MAX_REACH_X = 0.75
MIN_BODY_Z = -0.48       # Floor guard (~4.5 in off ground is around -0.41 to -0.45 in body frame)
PRE_GRASP_HEIGHT = 0.12  # Hover 12 cm straight above handle before descending


def load_registry():
    with open(REGISTRY_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def find_handle_in_body_frame(world_object_client, registry):
    """Reads AprilTags, applies physical size scale correction, and returns handle pose in body frame."""
    response = world_object_client.list_world_objects(
        object_type=[world_object_pb2.WORLD_OBJECT_APRILTAG]
    )
    for obj in response.world_objects:
        tag_id = obj.apriltag_properties.tag_id
        entry = registry.get(str(tag_id))
        if not entry or entry.get("type") != "grasp_handle":
            continue

        raw_body_tform_tag = get_a_tform_b(
            obj.transforms_snapshot,
            BODY_FRAME_NAME,
            obj.apriltag_properties.frame_name_fiducial,
        )
        if raw_body_tform_tag is None:
            continue

        # Scale translation from camera/body origin to match actual physical tag size on iPad
        corrected_tag_pose = SE3Pose(
            x=raw_body_tform_tag.x * TAG_SCALE,
            y=raw_body_tform_tag.y * TAG_SCALE,
            z=raw_body_tform_tag.z * TAG_SCALE,
            rot=raw_body_tform_tag.rot,
        )

        dx, dy, dz = entry.get("grasp_offset_xyz")
        body_tform_handle = SE3Pose(
            x=corrected_tag_pose.x + dx,
            y=corrected_tag_pose.y + dy,
            z=corrected_tag_pose.z + dz,
            rot=Quat(),
        )
        return tag_id, entry, corrected_tag_pose, body_tform_handle

    return None, None, None, None


def main():
    registry = load_registry()

    sdk = bosdyn.client.create_standard_sdk("ArmOnlyTopDownGrasp")
    robot = sdk.create_robot("192.168.80.3")
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    assert robot.has_arm(), "Robot does not have an arm attached."
    assert not robot.is_estopped(), "Robot is E-Stopped! Release E-Stop first."

    lease_client = robot.ensure_client(LeaseClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)

    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        print("Powering on motors and standing Spot...")
        robot.power_on(timeout_sec=20)
        command_client.robot_command(RobotCommandBuilder.synchro_stand_command())
        time.sleep(2.0)

        # 1. Aim hand camera 45 degrees downward so it sees low floor-level tags even up close
        print("Aiming hand camera 45 deg down to scan for Tag 2...")
        look_quat = Quat.from_pitch(math.radians(60))
        look_cmd = RobotCommandBuilder.arm_pose_command(
            x=0.45, y=0.0, z=0.0,
            qw=look_quat.w, qx=look_quat.x, qy=look_quat.y, qz=look_quat.z,
            frame_name=BODY_FRAME_NAME,
            seconds=2.0,
        )
        cmd_id = command_client.robot_command(look_cmd)
        block_until_arm_arrives(command_client, cmd_id, timeout_sec=4.0)

        # 2. Scan for Tag 2
        tag_id, entry, tag_pose, handle_pose = None, None, None, None
        for _ in range(20):
            tag_id, entry, tag_pose, handle_pose = find_handle_in_body_frame(world_object_client, registry)
            if handle_pose is not None:
                break
            time.sleep(0.5)

        if handle_pose is None:
            print("Tag 2 not seen. Stowing arm...")
            command_client.robot_command(RobotCommandBuilder.arm_stow_command())
            return

        print(f"Tag {tag_id} center (scaled): x={tag_pose.x:.3f}m, y={tag_pose.y:.3f}m, z={tag_pose.z:.3f}m")
        print(f"Handle target (scaled): x={handle_pose.x:.3f}m, y={handle_pose.y:.3f}m, z={handle_pose.z:.3f}m")

        if not (MIN_REACH_X <= handle_pose.x <= MAX_REACH_X):
            print(f"X={handle_pose.x:.3f}m out of range [{MIN_REACH_X}, {MAX_REACH_X}]. Move Spot closer/back.")
            command_client.robot_command(RobotCommandBuilder.arm_stow_command())
            return

        if abs(handle_pose.y) > 0.35:
            print(f"Y={handle_pose.y:.3f}m too far off-center. Center Spot in front of handle.")
            command_client.robot_command(RobotCommandBuilder.arm_stow_command())
            return

        if handle_pose.z < MIN_BODY_Z:
            print(f"Z={handle_pose.z:.3f}m below floor guard ({MIN_BODY_Z}m). Check grasp_offset_xyz.")
            command_client.robot_command(RobotCommandBuilder.arm_stow_command())
            return

        # 3. Hover 12 cm straight above handle pitched 90 degrees downward
        top_down_quat = Quat.from_pitch(math.pi / 2.0)
        hover_cmd = RobotCommandBuilder.arm_pose_command(
            x=handle_pose.x, y=handle_pose.y, z=handle_pose.z + PRE_GRASP_HEIGHT,
            qw=top_down_quat.w, qx=top_down_quat.x, qy=top_down_quat.y, qz=top_down_quat.z,
            frame_name=BODY_FRAME_NAME,
            seconds=2.5,
        )
        open_cmd = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
        cmd_id = command_client.robot_command(RobotCommandBuilder.build_synchro_command(hover_cmd, open_cmd))
        block_until_arm_arrives(command_client, cmd_id, timeout_sec=5.0)
        print("Hovering 90 deg above handle...")

        # 4. Lower straight down onto handle and close gripper
        grasp_cmd = RobotCommandBuilder.arm_pose_command(
            x=handle_pose.x, y=handle_pose.y, z=handle_pose.z,
            qw=top_down_quat.w, qx=top_down_quat.x, qy=top_down_quat.y, qz=top_down_quat.z,
            frame_name=BODY_FRAME_NAME,
            seconds=2.0,
        )
        cmd_id = command_client.robot_command(grasp_cmd)
        block_until_arm_arrives(command_client, cmd_id, timeout_sec=4.0)

        print("Closing gripper...")
        command_client.robot_command(RobotCommandBuilder.claw_gripper_open_fraction_command(0.0))
        time.sleep(1.5)

        # 5. Lift 10 cm straight up, hold, release, and stow
        lift_cmd = RobotCommandBuilder.arm_pose_command(
            x=handle_pose.x, y=handle_pose.y, z=handle_pose.z + 0.10,
            qw=top_down_quat.w, qx=top_down_quat.x, qy=top_down_quat.y, qz=top_down_quat.z,
            frame_name=BODY_FRAME_NAME,
            seconds=2.0,
        )
        cmd_id = command_client.robot_command(lift_cmd)
        block_until_arm_arrives(command_client, cmd_id, timeout_sec=4.0)
        print("Lifted! Holding 3 seconds...")
        time.sleep(3.0)

        cmd_id = command_client.robot_command(grasp_cmd)
        block_until_arm_arrives(command_client, cmd_id, timeout_sec=4.0)
        command_client.robot_command(open_cmd)
        time.sleep(1.0)

        cmd_id = command_client.robot_command(RobotCommandBuilder.arm_stow_command())
        block_until_arm_arrives(command_client, cmd_id, timeout_sec=5.0)
        print("Done and stowed.")


if __name__ == "__main__":
    main()