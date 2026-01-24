# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
IsaacStreamer - Teleoperation streamer using TeleopCore DeviceIO.

This streamer uses TeleopCore's deviceio module to receive controller, head,
and hand tracking data from OpenXR-compatible devices (e.g., CloudXR, Quest, Pico via OpenXR).

Unlike PicoStreamer which uses the XRoboToolkit SDK directly, this streamer uses
the standardized TeleopCore/OpenXR pipeline for device communication.
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as R

from gr00t_wbc.control.teleop.streamers.base_streamer import BaseStreamer, StreamerOutput

# OpenXR coordinate system: Y-up, we need Z-up for robot
# This is the same transformation as PicoStreamer uses
R_HEADSET_TO_WORLD = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ]
)


class IsaacStreamer(BaseStreamer):
    """
    Streamer using TeleopCore DeviceIO for OpenXR-based teleoperation.

    This streamer creates its own OpenXR and DeviceIO sessions to poll
    controller, head, and hand tracking data. It produces the same
    StreamerOutput format as PicoStreamer for compatibility with the
    existing teleop pipeline.

    Usage:
        streamer = IsaacStreamer(app_name="MyTeleopApp")
        streamer.start_streaming()
        while True:
            data = streamer.get()
            # Process data...
        streamer.stop_streaming()
    """

    def __init__(self, app_name: str = "GR00T_WBC_Teleop"):
        """Initialize the IsaacStreamer.

        Args:
            app_name: Name for the OpenXR application session
        """
        self.app_name = app_name

        # Session objects (initialized in start_streaming)
        self._oxr_session = None
        self._deviceio_session = None
        self._controller_tracker = None
        self._head_tracker = None

        # Track if streaming is active
        self._is_streaming = False

        self.reset_status()

    def reset_status(self):
        """Reset internal state variables."""
        self.current_base_height = 0.74  # Initial base height, 0.74m (standing height)
        self.toggle_policy_action_last = False
        self.toggle_activation_last = False
        self.toggle_data_collection_last = False
        self.toggle_data_abort_last = False
        self._frame_count = 0

    def start_streaming(self):
        """Initialize OpenXR and DeviceIO sessions and start tracking."""
        if self._is_streaming:
            print("IsaacStreamer: Already streaming")
            return

        try:
            import teleopcore.deviceio as deviceio
            import teleopcore.oxr as oxr
        except ImportError as e:
            raise ImportError(
                "TeleopCore is required for IsaacStreamer. "
                "Please install teleopcore package."
            ) from e

        # Create trackers (head + controllers, no hand tracking needed)
        self._controller_tracker = deviceio.ControllerTracker()
        self._head_tracker = deviceio.HeadTracker()

        trackers = [self._controller_tracker, self._head_tracker]

        # Get required OpenXR extensions from trackers
        required_extensions = deviceio.DeviceIOSession.get_required_extensions(trackers)

        # Create OpenXR session
        print(f"IsaacStreamer: Creating OpenXR session with app name: {self.app_name}")
        self._oxr_session = oxr.OpenXRSession.create(self.app_name, required_extensions)
        self._oxr_session.__enter__()

        # Get OpenXR handles for DeviceIO
        handles = self._oxr_session.get_handles()

        # Create DeviceIO session with trackers
        print("IsaacStreamer: Creating DeviceIO session with trackers")
        self._deviceio_session = deviceio.DeviceIOSession.run(trackers, handles)
        self._deviceio_session.__enter__()

        self._is_streaming = True
        print("IsaacStreamer: Streaming started successfully")

    def stop_streaming(self):
        """Stop tracking and cleanup sessions."""
        if not self._is_streaming:
            return

        print("IsaacStreamer: Stopping streaming...")

        # Cleanup in reverse order
        if self._deviceio_session is not None:
            try:
                self._deviceio_session.__exit__(None, None, None)
            except Exception as e:
                print(f"IsaacStreamer: Error closing DeviceIO session: {e}")
            self._deviceio_session = None

        if self._oxr_session is not None:
            try:
                self._oxr_session.__exit__(None, None, None)
            except Exception as e:
                print(f"IsaacStreamer: Error closing OpenXR session: {e}")
            self._oxr_session = None

        self._controller_tracker = None
        self._head_tracker = None
        self._is_streaming = False

        print("IsaacStreamer: Streaming stopped")

    def get(self) -> StreamerOutput:
        """Get current tracking data and return as StreamerOutput.

        Returns:
            StreamerOutput with IK data, control commands, teleop commands, and data collection commands.
        """
        if not self._is_streaming:
            print("IsaacStreamer: Warning - get() called but not streaming")
            return StreamerOutput(source="isaac")

        # Update DeviceIO session (polls all trackers)
        self._deviceio_session.update()

        # Get raw data from trackers
        isaac_data = self._get_isaac_data()

        # Convert to unified format
        raw_data = self._generate_unified_raw_data(isaac_data)
        return raw_data

    def _get_isaac_data(self) -> dict:
        """Poll all trackers and collect raw data.

        Returns:
            Dictionary with controller and head tracking data.
        """
        isaac_data = {}

        # Get controller data (both left and right)
        controller_data = self._controller_tracker.get_controller_data(self._deviceio_session)
        isaac_data["left_controller"] = controller_data.left_controller
        isaac_data["right_controller"] = controller_data.right_controller

        # Get head pose
        head_data = self._head_tracker.get_head(self._deviceio_session)
        isaac_data["head_pose"] = head_data

        return isaac_data

    def _generate_unified_raw_data(self, isaac_data: dict) -> StreamerOutput:
        """Convert TeleopCore data to StreamerOutput format.

        This method maps the TeleopCore schema types to the same output format
        as PicoStreamer for compatibility with the existing teleop pipeline.

        Args:
            isaac_data: Raw data from TeleopCore trackers

        Returns:
            StreamerOutput with all teleoperation data
        """
        left_controller = isaac_data["left_controller"]
        right_controller = isaac_data["right_controller"]
        head_pose = isaac_data["head_pose"]

        # Get controller poses (grip pose is used for wrist tracking)
        # Transform from OpenXR coordinates to robot coordinates
        left_controller_T = self._process_controller_pose(left_controller, head_pose)
        right_controller_T = self._process_controller_pose(right_controller, head_pose)

        # Get navigation commands from controller joysticks
        DEAD_ZONE = 0.1
        MAX_LINEAR_VEL = 0.5  # m/s
        MAX_ANGULAR_VEL = 1.0  # rad/s

        # Left joystick for X/Y translation, right joystick for yaw
        left_inputs = left_controller.inputs if left_controller else None
        right_inputs = right_controller.inputs if right_controller else None

        fwd_bwd_input = -left_inputs.thumbstick_y if left_inputs else 0.0
        strafe_input = -left_inputs.thumbstick_x if left_inputs else 0.0
        yaw_input = -right_inputs.thumbstick_x if right_inputs else 0.0

        lin_vel_x = self._apply_dead_zone(fwd_bwd_input, DEAD_ZONE) * MAX_LINEAR_VEL
        lin_vel_y = self._apply_dead_zone(strafe_input, DEAD_ZONE) * MAX_LINEAR_VEL
        ang_vel_z = self._apply_dead_zone(yaw_input, DEAD_ZONE) * MAX_ANGULAR_VEL

        # Get base height command from secondary button (B/Y buttons)
        # Map: secondary_click on right (B) = up, secondary_click on left (Y) = down
        height_increment = 0.01
        if right_inputs and right_inputs.secondary_click:
            self.current_base_height += height_increment
        elif left_inputs and left_inputs.secondary_click:
            self.current_base_height -= height_increment
        self.current_base_height = np.clip(self.current_base_height, 0.2, 0.74)

        # Get gripper commands from triggers and grips
        left_fingers = self._generate_finger_data(left_inputs)
        right_fingers = self._generate_finger_data(right_inputs)

        # Activation commands:
        # - thumbstick_click on left + trigger on left = toggle policy action
        # - thumbstick_click on left + trigger on right = toggle activation
        left_thumbstick_click = left_inputs.thumbstick_click if left_inputs else False
        left_trigger = left_inputs.trigger_value if left_inputs else 0.0
        right_trigger = right_inputs.trigger_value if right_inputs else 0.0

        toggle_policy_action_tmp = left_thumbstick_click and (left_trigger > 0.5)
        toggle_activation_tmp = left_thumbstick_click and (right_trigger > 0.5)

        if self.toggle_policy_action_last != toggle_policy_action_tmp:
            toggle_policy_action = toggle_policy_action_tmp
        else:
            toggle_policy_action = False
        self.toggle_policy_action_last = toggle_policy_action_tmp

        if self.toggle_activation_last != toggle_activation_tmp:
            toggle_activation = toggle_activation_tmp
        else:
            toggle_activation = False
        self.toggle_activation_last = toggle_activation_tmp

        # Data collection commands:
        # - primary_click on right (A) = toggle data collection
        # - primary_click on left (X) = abort data collection
        toggle_data_collection_tmp = right_inputs.primary_click if right_inputs else False
        toggle_data_abort_tmp = left_inputs.primary_click if left_inputs else False

        if self.toggle_data_collection_last != toggle_data_collection_tmp:
            toggle_data_collection = toggle_data_collection_tmp
        else:
            toggle_data_collection = False
        self.toggle_data_collection_last = toggle_data_collection_tmp

        if self.toggle_data_abort_last != toggle_data_abort_tmp:
            toggle_data_abort = toggle_data_abort_tmp
        else:
            toggle_data_abort = False
        self.toggle_data_abort_last = toggle_data_abort_tmp

        # Print detailed status periodically for debugging
        self._print_controller_status(
            left_controller, right_controller, head_pose,
            toggle_policy_action, toggle_activation
        )

        return StreamerOutput(
            ik_data={
                "left_wrist": left_controller_T,
                "right_wrist": right_controller_T,
                "left_fingers": {"position": left_fingers},
                "right_fingers": {"position": right_fingers},
            },
            control_data={
                "base_height_command": self.current_base_height,
                "navigate_cmd": [lin_vel_x, lin_vel_y, ang_vel_z],
                "toggle_policy_action": toggle_policy_action,
            },
            teleop_data={
                "toggle_activation": toggle_activation,
            },
            data_collection_data={
                "toggle_data_collection": toggle_data_collection,
                "toggle_data_abort": toggle_data_abort,
            },
            source="isaac",
        )

    def _print_controller_status(
        self, left_controller, right_controller, head_pose,
        toggle_policy_action: bool = False, toggle_activation: bool = False
    ):
        """Print controller status inline (updates same line)."""
        self._frame_count += 1

        def ctrl_status(ctrl, name):
            if ctrl is None:
                return f"{name}:None"
            a = "A" if ctrl.is_active else "-"
            g = "G" if (ctrl.grip_pose and ctrl.grip_pose.is_valid) else "-"
            m = "M" if (ctrl.aim_pose and ctrl.aim_pose.is_valid) else "-"
            if ctrl.grip_pose and ctrl.grip_pose.is_valid:
                p = ctrl.grip_pose.pose.position
                pos = f"[{p.x:+.2f},{p.y:+.2f},{p.z:+.2f}]"
            else:
                pos = "[---.--,---.--,---.--]"
            return f"{name}:{a}{g}{m}{pos}"

        head_str = "H:OK" if (head_pose and head_pose.is_valid) else "H:--"
        left_str = ctrl_status(left_controller, "L")
        right_str = ctrl_status(right_controller, "R")

        # Toggle status: P=policy_action, A=activation (uppercase when triggered)
        policy_str = "P" if toggle_policy_action else "p"
        activ_str = "A" if toggle_activation else "a"
        toggle_str = f"[{policy_str}{activ_str}]"

        print(f"\r[{self._frame_count:5d}] {head_str} | {left_str} | {right_str} | {toggle_str}    ", end="", flush=True)

    def _process_controller_pose(self, controller, head_pose) -> np.ndarray:
        """Process controller pose and transform to robot coordinate frame.

        Similar to PicoStreamer._process_xr_pose but using TeleopCore schema types.

        Args:
            controller: ControllerSnapshot from TeleopCore
            head_pose: HeadPoseT from TeleopCore

        Returns:
            4x4 transformation matrix for the controller pose in robot frame
        """
        # Return identity if controller is not valid
        if controller is None:
            return np.eye(4)

        if not controller.is_active:
            return np.eye(4)

        grip_pose = controller.grip_pose
        if not grip_pose.is_valid:
            # Try aim_pose as fallback since is_active can be True if aim_pose is valid
            aim_pose = controller.aim_pose
            if aim_pose.is_valid:
                grip_pose = aim_pose
            else:
                return np.eye(4)

        # Extract controller position and orientation
        pose = grip_pose.pose
        xr_pose_xyz = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z
        ])
        xr_pose_quat = np.array([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])

        # Handle all-zero quaternion case by using identity quaternion
        if np.allclose(xr_pose_quat, 0):
            xr_pose_quat = np.array([0, 0, 0, 1])  # identity quaternion: x, y, z, w

        # Convert from Y-up (OpenXR) to Z-up (robot)
        xr_pose_xyz = R_HEADSET_TO_WORLD @ xr_pose_xyz
        xr_pose_rotation = R.from_quat(xr_pose_quat).as_matrix()
        xr_pose_rotation = R_HEADSET_TO_WORLD @ xr_pose_rotation @ R_HEADSET_TO_WORLD.T

        # Process head pose for relative positioning
        if head_pose is not None and head_pose.is_valid and head_pose.pose is not None:
            headset_pose_xyz = np.array([
                head_pose.pose.position.x,
                head_pose.pose.position.y,
                head_pose.pose.position.z
            ])
            headset_pose_quat = np.array([
                head_pose.pose.orientation.x,
                head_pose.pose.orientation.y,
                head_pose.pose.orientation.z,
                head_pose.pose.orientation.w
            ])

            if np.allclose(headset_pose_quat, 0):
                headset_pose_quat = np.array([0, 0, 0, 1])

            # Convert from Y-up to Z-up
            headset_pose_xyz = R_HEADSET_TO_WORLD @ headset_pose_xyz
            headset_pose_rotation = R.from_quat(headset_pose_quat).as_matrix()
            headset_pose_rotation = R_HEADSET_TO_WORLD @ headset_pose_rotation @ R_HEADSET_TO_WORLD.T

            # Calculate delta between controller and headset positions
            xr_pose_xyz_delta = xr_pose_xyz - headset_pose_xyz

            # Calculate yaw of headset and compensate
            R_headset_to_world = R.from_matrix(headset_pose_rotation)
            headset_pose_yaw = R_headset_to_world.as_euler("xyz")[2]
            inverse_yaw_rotation = R.from_euler("z", -headset_pose_yaw).as_matrix()

            # Align with headset yaw
            xr_pose_xyz_delta = inverse_yaw_rotation @ xr_pose_xyz_delta
            xr_pose_rotation = inverse_yaw_rotation @ xr_pose_rotation

            xr_pose_xyz = xr_pose_xyz_delta

        # Build 4x4 transformation matrix
        xr_pose_T = np.eye(4)
        xr_pose_T[:3, :3] = xr_pose_rotation
        xr_pose_T[:3, 3] = xr_pose_xyz
        return xr_pose_T

    def _apply_dead_zone(self, value: float, dead_zone: float) -> float:
        """Apply dead zone and normalize joystick input.

        Args:
            value: Raw joystick value (-1 to 1)
            dead_zone: Dead zone threshold

        Returns:
            Normalized value with dead zone applied
        """
        if abs(value) < dead_zone:
            return 0.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - dead_zone) / (1.0 - dead_zone)

    def _generate_finger_data(self, inputs) -> np.ndarray:
        """Generate finger position data from controller inputs.

        Maps trigger and grip buttons to finger positions, same as PicoStreamer.

        Args:
            inputs: ControllerInputState from TeleopCore (or None)

        Returns:
            Finger position array (25, 4, 4) with finger states
        """
        fingertips = np.zeros([25, 4, 4])

        thumb = 0
        index = 5
        middle = 10
        ring = 15

        # Default: open thumb
        fingertips[4 + thumb, 0, 3] = 1.0

        if inputs is None:
            return fingertips

        trigger = inputs.trigger_value
        grip = inputs.squeeze_value
        thumbstick_click = inputs.thumbstick_click

        # Control fingers based on trigger/grip state
        # (Similar logic to PicoStreamer but using thumbstick_click instead of menu button)
        if not thumbstick_click:
            if trigger > 0.5 and not grip > 0.5:
                fingertips[4 + index, 0, 3] = 1.0  # close index
            elif trigger > 0.5 and grip > 0.5:
                fingertips[4 + middle, 0, 3] = 1.0  # close middle
            elif not trigger > 0.5 and grip > 0.5:
                fingertips[4 + ring, 0, 3] = 1.0  # close ring

        return fingertips

    def __del__(self):
        """Cleanup on destruction."""
        self.stop_streaming()


if __name__ == "__main__":
    """Unit test for IsaacStreamer."""

    print("Testing IsaacStreamer...")
    print("Note: This requires an active OpenXR runtime (e.g., CloudXR, SteamVR, etc.)")

    streamer = IsaacStreamer(app_name="IsaacStreamer_Test")

    try:
        streamer.start_streaming()

        print("\nStreaming started. Reading data for 60 seconds...")
        start_time = time.time()
        frame_count = 0

        while time.time() - start_time < 60.0:
            raw_data = streamer.get()
            frame_count += 1

            # Print status every second
            if frame_count % 10 == 0:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                print(f"Frame {frame_count}, FPS: {fps:.1f}")
                print(f"  Left wrist:\n{raw_data.ik_data['left_wrist']}")
                print(f"  Right wrist:\n{raw_data.ik_data['right_wrist']}")
                print(f"  Navigate cmd: {raw_data.control_data['navigate_cmd']}")

            time.sleep(0.1)  # ~10 Hz polling

        print(f"\nTest complete. Total frames: {frame_count}")

    except Exception as e:
        print(f"Error during test: {e}")
        import traceback
        traceback.print_exc()

    finally:
        streamer.stop_streaming()
        print("Streamer stopped.")
