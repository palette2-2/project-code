#!/usr/bin/env python3
"""
Low-latency PICO/XRobot teleop bridge for sim2real.

Architecture:
1. XR callback thread stores the latest VR snapshot with a monotonic timestamp.
2. A retarget thread waits for new VR data and only retargets the latest snapshot.
3. A request thread serves the newest ZMQ request using time-based interpolation over a
   short retarget history buffer.
4. A control thread publishes controller buttons and, when enabled, the latest
   Unitree G1 upper-body reference, PICO RC command, and Dex3 target over LCM.
"""

import argparse
import json
import multiprocessing as mp
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Union

import numpy as np
from scipy.spatial.transform import Rotation as R

from default_mimic_obs import DEFAULT_MIMIC_OBS
from lcm_types.hand_action_lcmt import hand_action_lcmt
from lcm_types.rc_command_lcmt import rc_command_lcmt
from lcm_types.ref_upper_dof_pos_lcmt import ref_upper_dof_pos_lcmt

GMR = None
RobotMotionViewer = None
quat_mul_np = None
xrt = None

DEFAULT_LCM_URL = "udpm://239.255.76.67:7667?ttl=255"
DEFAULT_LCM_CHANNEL = "ref_upper_dof_pos_channel"
DEFAULT_RC_LCM_CHANNEL = "pico_rc_command"
DEFAULT_HAND_LCM_CHANNEL = "hand_action"
DEFAULT_DEX3_POSE_CONFIG = Path(__file__).resolve().parent / "config" / "dex3_hand_poses.json"
G1_DOF_COUNT = 29
G1_UPPER_DOF_START = 15
G1_UPPER_DOF_END = 29

DEX3_LEFT_MIN = np.array([-1.05, -0.724, 0.0, -1.57, -1.75, -1.57, -1.75])
DEX3_LEFT_MAX = np.array([1.05, 1.05, 1.75, 0.0, 0.0, 0.0, 0.0])
DEX3_RIGHT_MIN = np.array([-1.05, -1.05, -1.75, 0.0, 0.0, 0.0, 0.0])
DEX3_RIGHT_MAX = np.array([1.05, 0.742, 0.0, 1.57, 1.75, 1.57, 1.75])

XR_BODY_JOINT_NAMES = [
    "Pelvis",
    "Left_Hip",
    "Right_Hip",
    "Spine1",
    "Left_Knee",
    "Right_Knee",
    "Spine2",
    "Left_Ankle",
    "Right_Ankle",
    "Spine3",
    "Left_Foot",
    "Right_Foot",
    "Neck",
    "Left_Collar",
    "Right_Collar",
    "Head",
    "Left_Shoulder",
    "Right_Shoulder",
    "Left_Elbow",
    "Right_Elbow",
    "Left_Wrist",
    "Right_Wrist",
    "Left_Hand",
    "Right_Hand",
]


def _load_runtime_dependencies() -> None:
    global GMR, RobotMotionViewer, quat_mul_np, xrt

    try:
        from general_motion_retargeting import GeneralMotionRetargeting as _GMR
        from general_motion_retargeting import RobotMotionViewer as _RobotMotionViewer
        from general_motion_retargeting.rot_utils import quat_mul_np as _quat_mul_np
    except ImportError as exc:
        raise ImportError(
            "Failed to import 'general_motion_retargeting'. Install GMR in the active Python environment."
        ) from exc

    try:
        import xrobotoolkit_sdk as _xrt
    except ImportError as exc:
        raise ImportError(
            "Failed to import 'xrobotoolkit_sdk'. Install the patched SDK in the active Python environment."
        ) from exc

    for name in (
        "register_frame_callback",
        "clear_frame_callback",
        "has_frame_callback",
    ):
        if not hasattr(_xrt, name):
            raise ImportError(
                "Installed xrobotoolkit_sdk does not expose callback APIs. "
                "Reinstall the patched XRoboToolkit-PC-Service-Pybind build."
            )

    GMR = _GMR
    RobotMotionViewer = _RobotMotionViewer
    quat_mul_np = _quat_mul_np
    xrt = _xrt


@dataclass
class RetargetedFrame:
    recv_ns: int
    qpos: np.ndarray


class PicoControllerMapper:
    """Condition PICO controls and enforce release-to-arm after startup/stale input."""

    def __init__(
        self,
        stale_timeout_ms: float = 250.0,
        stick_deadzone: float = 0.1,
        press_threshold: float = 0.7,
        release_threshold: float = 0.3,
        toggle_button: str = "right_key_two",
    ) -> None:
        if stale_timeout_ms <= 0:
            raise ValueError("controller_stale_timeout_ms must be > 0")
        if not 0.0 <= stick_deadzone < 1.0:
            raise ValueError("stick_deadzone must be in [0, 1)")
        if not 0.0 <= release_threshold < press_threshold <= 1.0:
            raise ValueError("button thresholds must satisfy 0 <= release < press <= 1")
        self.stale_timeout_ns = int(stale_timeout_ms * 1e6)
        self.stick_deadzone = float(stick_deadzone)
        self.press_threshold = float(press_threshold)
        self.release_threshold = float(release_threshold)
        self.toggle_button = str(toggle_button)
        self.armed = False
        self.r1_pressed = False
        self.r2_pressed = False
        self.toggle_was_pressed = False

    @staticmethod
    def _finite_float(value: Any) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return 0.0
        return result if np.isfinite(result) else 0.0

    @staticmethod
    def _is_finite_number(value: Any) -> bool:
        try:
            return bool(np.isfinite(float(value)))
        except (TypeError, ValueError):
            return False

    def _axis(self, value: Any) -> list[float]:
        if not isinstance(value, (list, tuple, np.ndarray)) or len(value) < 2:
            return [0.0, 0.0]
        if not self._is_finite_number(value[0]) or not self._is_finite_number(value[1]):
            return [0.0, 0.0]
        vector = np.array(
            [self._finite_float(value[0]), self._finite_float(value[1])],
            dtype=np.float64,
        )
        magnitude = float(np.linalg.norm(vector))
        if magnitude <= self.stick_deadzone or magnitude <= 1e-12:
            return [0.0, 0.0]
        direction = vector / magnitude
        mapped_magnitude = min(1.0, (magnitude - self.stick_deadzone) / (1.0 - self.stick_deadzone))
        return (direction * mapped_magnitude).astype(np.float32).tolist()

    def _hysteresis(self, value: float, was_pressed: bool) -> bool:
        if value >= self.press_threshold:
            return True
        if value <= self.release_threshold:
            return False
        return was_pressed

    def _neutral(self, fresh: bool = False) -> Dict[str, Any]:
        return {
            "left_stick": [0.0, 0.0],
            "right_stick": [0.0, 0.0],
            "r1": False,
            "r2": False,
            "toggle_rising": False,
            "fresh": bool(fresh),
            "armed": bool(self.armed),
        }

    def update(self, buttons: Dict[str, Any], recv_ns: int, now_ns: int) -> Dict[str, Any]:
        fresh = recv_ns > 0 and 0 <= now_ns - recv_ns <= self.stale_timeout_ns
        if not fresh:
            self.armed = False
            self.r1_pressed = False
            self.r2_pressed = False
            self.toggle_was_pressed = False
            return self._neutral(fresh=False)

        grip = self._finite_float(buttons.get("right_grip_value", buttons.get("right_grip", 0.0)))
        trigger = self._finite_float(
            buttons.get("right_trigger_value", buttons.get("right_index_trig", 0.0))
        )
        toggle_pressed = bool(buttons.get(self.toggle_button, False))

        if not self.armed:
            if grip <= self.release_threshold and trigger <= self.release_threshold and not toggle_pressed:
                self.armed = True
            self.toggle_was_pressed = toggle_pressed
            return self._neutral(fresh=True)

        self.r1_pressed = self._hysteresis(grip, self.r1_pressed)
        self.r2_pressed = self._hysteresis(trigger, self.r2_pressed)
        toggle_rising = toggle_pressed and not self.toggle_was_pressed
        self.toggle_was_pressed = toggle_pressed
        return {
            "left_stick": self._axis(buttons.get("left_axis", [0.0, 0.0])),
            "right_stick": self._axis(buttons.get("right_axis", [0.0, 0.0])),
            "r1": self.r1_pressed,
            "r2": self.r2_pressed,
            "toggle_rising": toggle_rising,
            "fresh": True,
            "armed": True,
        }


def load_dex3_pose_config(path: Union[str, Path]) -> Dict[str, np.ndarray]:
    config_path = Path(path).expanduser()
    try:
        with config_path.open("r", encoding="utf-8") as config_file:
            raw = json.load(config_file)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"failed to load Dex3 pose config {config_path}: {exc}") from exc

    if not isinstance(raw, dict):
        raise ValueError("Dex3 pose config must be a JSON object")

    limits = {
        "left_open": (DEX3_LEFT_MIN, DEX3_LEFT_MAX),
        "left_closed": (DEX3_LEFT_MIN, DEX3_LEFT_MAX),
        "right_open": (DEX3_RIGHT_MIN, DEX3_RIGHT_MAX),
        "right_closed": (DEX3_RIGHT_MIN, DEX3_RIGHT_MAX),
    }
    unexpected = sorted(set(raw) - set(limits))
    if unexpected:
        raise ValueError(f"Dex3 pose config contains unexpected fields: {unexpected}")
    poses: Dict[str, np.ndarray] = {}
    for name, (minimum, maximum) in limits.items():
        if name not in raw:
            raise ValueError(f"Dex3 pose config is missing {name!r}")
        try:
            pose = np.asarray(raw[name], dtype=np.float64)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Dex3 pose {name!r} must contain numeric values") from exc
        if pose.shape != (7,):
            raise ValueError(f"Dex3 pose {name!r} must have shape (7,), got {pose.shape}")
        if not np.all(np.isfinite(pose)):
            raise ValueError(f"Dex3 pose {name!r} contains NaN or Inf")
        invalid = np.flatnonzero((pose < minimum) | (pose > maximum))
        if invalid.size:
            joint = int(invalid[0])
            raise ValueError(
                f"Dex3 pose {name!r}[{joint}]={pose[joint]} is outside "
                f"[{minimum[joint]}, {maximum[joint]}]"
            )
        poses[name] = pose.copy()
    return poses


class Dex3HandStateMachine:
    """Toggle two Dex3 hands and generate a smooth, stale-safe 14D target."""

    def __init__(
        self,
        poses: Dict[str, np.ndarray],
        transition_s: float = 0.5,
        stale_timeout_ms: float = 250.0,
    ) -> None:
        if transition_s <= 0:
            raise ValueError("hand_transition_s must be > 0")
        if stale_timeout_ms <= 0:
            raise ValueError("controller_stale_timeout_ms must be > 0")
        self.poses = {name: np.asarray(value, dtype=np.float64).copy() for name, value in poses.items()}
        self.transition_ns = int(float(transition_s) * 1e9)
        self.stale_timeout_ns = int(float(stale_timeout_ms) * 1e6)

        self.armed = False
        self.left_closed = False
        self.right_closed = False
        self.left_was_pressed = False
        self.right_was_pressed = False

        self.transition_start_ns = 0
        self.transition_start = self._desired_pose()
        self.transition_target = self.transition_start.copy()

    def _desired_pose(self) -> np.ndarray:
        left = self.poses["left_closed" if self.left_closed else "left_open"]
        right = self.poses["right_closed" if self.right_closed else "right_open"]
        return np.concatenate((left, right)).astype(np.float64, copy=False)

    def _sample(self, now_ns: int) -> np.ndarray:
        if self.transition_start_ns <= 0:
            return self.transition_target.copy()
        elapsed_ns = max(0, int(now_ns) - self.transition_start_ns)
        alpha = min(1.0, float(elapsed_ns) / float(self.transition_ns))
        return self.transition_start + alpha * (self.transition_target - self.transition_start)

    def _start_transition(self, now_ns: int, current: np.ndarray) -> None:
        self.transition_start_ns = int(now_ns)
        self.transition_start = current.copy()
        self.transition_target = self._desired_pose()

    def _freeze(self, now_ns: int, current: np.ndarray) -> None:
        self.transition_start_ns = int(now_ns)
        self.transition_start = current.copy()
        self.transition_target = current.copy()

    def update(
        self, buttons: Dict[str, Any], recv_ns: int, now_ns: int
    ) -> tuple[np.ndarray, Dict[str, Any]]:
        current = self._sample(now_ns)
        fresh = recv_ns > 0 and 0 <= now_ns - recv_ns <= self.stale_timeout_ns
        left_pressed = bool(buttons.get("left_key_one", False))
        right_pressed = bool(buttons.get("right_key_one", False))

        if not fresh:
            self._freeze(now_ns, current)
            self.armed = False
            self.left_was_pressed = False
            self.right_was_pressed = False
        elif not self.armed:
            if not left_pressed and not right_pressed:
                self.armed = True
            self.left_was_pressed = left_pressed
            self.right_was_pressed = right_pressed
        else:
            left_rising = left_pressed and not self.left_was_pressed
            right_rising = right_pressed and not self.right_was_pressed
            self.left_was_pressed = left_pressed
            self.right_was_pressed = right_pressed
            if left_rising or right_rising:
                if left_rising:
                    self.left_closed = not self.left_closed
                if right_rising:
                    self.right_closed = not self.right_closed
                self._start_transition(now_ns, current)
                current = self._sample(now_ns)

        moving = not np.array_equal(self.transition_start, self.transition_target) and (
            int(now_ns) - self.transition_start_ns < self.transition_ns
        )
        return current.copy(), {
            "armed": bool(self.armed),
            "fresh": bool(fresh),
            "left": "closed" if self.left_closed else "open",
            "right": "closed" if self.right_closed else "open",
            "moving": bool(moving),
        }


class UpperTeleopStateMachine:
    PAUSED_INITIAL = "paused_initial"
    BLENDING = "blending"
    LIVE = "live"
    PAUSED_HOLD = "paused_hold"

    def __init__(
        self,
        blend_s: float = 0.5,
        pose_stale_timeout_ms: float = 250.0,
        start_active: bool = False,
    ) -> None:
        if blend_s < 0:
            raise ValueError("upper_teleop_blend_s must be >= 0")
        if pose_stale_timeout_ms <= 0:
            raise ValueError("upper_pose_stale_timeout_ms must be > 0")
        self.blend_ns = int(blend_s * 1e9)
        self.pose_stale_timeout_ns = int(pose_stale_timeout_ms * 1e6)
        self.start_active = bool(start_active)
        self.state = self.PAUSED_INITIAL
        self.output = np.zeros(G1_UPPER_DOF_END - G1_UPPER_DOF_START, dtype=np.float64)
        self.blend_start = self.output.copy()
        self.blend_start_ns = 0

    def _valid_pose(self, pose: Optional[np.ndarray], recv_ns: int, now_ns: int) -> tuple[Optional[np.ndarray], bool]:
        if pose is None or recv_ns <= 0 or not 0 <= now_ns - recv_ns <= self.pose_stale_timeout_ns:
            return None, False
        try:
            value = np.asarray(pose, dtype=np.float64).reshape(-1)
        except (TypeError, ValueError):
            return None, False
        if value.shape != self.output.shape or not np.all(np.isfinite(value)):
            return None, False
        return value, True

    def _begin_blend(self, now_ns: int) -> None:
        self.blend_start = self.output.copy()
        self.blend_start_ns = int(now_ns)
        self.state = self.BLENDING

    def _pause(self) -> None:
        self.output = self.output.copy()
        self.state = self.PAUSED_HOLD

    def step(
        self,
        now_ns: int,
        live_pose: Optional[np.ndarray],
        live_pose_recv_ns: int,
        controller_fresh: bool,
        controller_armed: bool,
        toggle_rising: bool,
    ) -> tuple[np.ndarray, Dict[str, Any], Optional[str]]:
        pose, pose_fresh = self._valid_pose(live_pose, live_pose_recv_ns, now_ns)
        active = self.state in (self.BLENDING, self.LIVE)
        notice = None

        if active and (not controller_fresh or not pose_fresh):
            self._pause()
            notice = "upper teleop paused because PICO controller or pose data became stale"
            active = False

        if toggle_rising:
            if active:
                self._pause()
                notice = "upper teleop paused and holding the last published reference"
            elif controller_fresh and controller_armed and pose_fresh:
                self._begin_blend(now_ns)
                notice = "upper teleop blending to the current PICO pose"
            else:
                notice = "upper teleop remains paused: no fresh PICO/GMR pose is available"
        elif (
            self.start_active
            and self.state == self.PAUSED_INITIAL
            and controller_fresh
            and controller_armed
            and pose_fresh
        ):
            self.start_active = False
            self._begin_blend(now_ns)
            notice = "upper teleop auto-start blending enabled"

        if self.state == self.BLENDING and pose is not None:
            if self.blend_ns == 0:
                alpha = 1.0
            else:
                alpha = float(np.clip((now_ns - self.blend_start_ns) / self.blend_ns, 0.0, 1.0))
            self.output = self.blend_start * (1.0 - alpha) + pose * alpha
            if alpha >= 1.0:
                self.state = self.LIVE
                notice = notice or "upper teleop is live"
        elif self.state == self.LIVE and pose is not None:
            self.output = pose.copy()

        status = {"state": self.state, "pose_fresh": bool(pose_fresh)}
        return self.output.copy(), status, notice


class _RetargetWorkerRuntime:
    ROBOT_GROUND_REFERENCE_BODY_NAMES = ("left_toe_link", "right_toe_link")

    def __init__(self, worker_config: Dict[str, Any]):
        from general_motion_retargeting import GeneralMotionRetargeting
        from general_motion_retargeting.rot_utils import quat_mul_np as worker_quat_mul_np

        self._quat_mul_np = worker_quat_mul_np
        self.retarget = GeneralMotionRetargeting(
            src_human="xrobot",
            tgt_robot="unitree_g1",
            actual_human_height=float(worker_config["actual_human_height"]),
        )
        self.retarget.max_iter = int(worker_config["gmr_max_iter"])
        self.send_human_motion = bool(worker_config["send_human_motion"])
        self.min_link_height = float(worker_config["min_link_height"])
        self.min_link_height_align_strategy = str(worker_config["min_link_height_align_strategy"])
        self.min_link_height_bootstrap_frames = max(1, int(worker_config["min_link_height_bootstrap_frames"]))
        self.fixed_min_link_height_offset: Optional[float] = None
        self.min_link_height_offset_samples: list[float] = []
        self.rotation_matrix = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
        self.rotation_quat = R.from_matrix(self.rotation_matrix).as_quat(scalar_first=True)

    def _body_poses_to_pose_dict(self, poses: Any) -> Optional[Dict[str, Any]]:
        if not isinstance(poses, (list, tuple)) or len(poses) < len(XR_BODY_JOINT_NAMES):
            return None

        body_pose_dict: Dict[str, Any] = {}
        for i, joint_name in enumerate(XR_BODY_JOINT_NAMES):
            pose = poses[i]
            if not isinstance(pose, (list, tuple)) or len(pose) < 7:
                return None
            x, y, z, qx, qy, qz, qw = [float(v) for v in pose[:7]]
            pos = np.array([x, y, z], dtype=np.float64) @ self.rotation_matrix.T
            rot = self._quat_mul_np(
                self.rotation_quat.reshape(1, 4),
                np.array([[qw, qx, qy, qz]], dtype=np.float64),
                scalar_first=True,
            )[0]
            body_pose_dict[joint_name] = [pos.tolist(), rot.tolist()]
        return body_pose_dict

    def _get_current_min_body_z(self) -> Optional[float]:
        body_z = self.retarget.configuration.data.xpos[1:, 2]
        if body_z.size == 0:
            return None
        min_body_z = float(np.min(body_z))
        if not np.isfinite(min_body_z):
            return None
        return min_body_z

    def _get_current_ground_reference_z(self) -> Optional[float]:
        toe_z_values: list[float] = []
        body_name_map = getattr(self.retarget, "robot_body_names", {})
        data = self.retarget.configuration.data

        for body_name in self.ROBOT_GROUND_REFERENCE_BODY_NAMES:
            body_id = body_name_map.get(body_name)
            if body_id is None:
                continue
            if body_id < 0 or body_id >= data.xpos.shape[0]:
                continue
            z = float(data.xpos[body_id, 2])
            if np.isfinite(z):
                toe_z_values.append(z)

        if toe_z_values:
            return float(min(toe_z_values))
        return self._get_current_min_body_z()

    def _apply_min_link_height_offset(self, qpos: np.ndarray) -> np.ndarray:
        qpos_adj = np.asarray(qpos, dtype=np.float32).copy()
        ground_ref_z = self._get_current_ground_reference_z()
        if ground_ref_z is None:
            return qpos_adj

        if self.min_link_height_align_strategy == "per_frame":
            qpos_adj[2] += self.min_link_height - ground_ref_z
            return qpos_adj

        if self.fixed_min_link_height_offset is None:
            offset = self.min_link_height - ground_ref_z
            self.min_link_height_offset_samples.append(offset)
            if len(self.min_link_height_offset_samples) >= self.min_link_height_bootstrap_frames:
                self.fixed_min_link_height_offset = float(np.mean(self.min_link_height_offset_samples))
                print(
                    "[Info] worker startup_fixed ground calibration: "
                    f"{self.fixed_min_link_height_offset:.6f} m from "
                    f"{len(self.min_link_height_offset_samples)} frames"
                )
                self.min_link_height_offset_samples.clear()

        applied_offset = (
            self.fixed_min_link_height_offset
            if self.fixed_min_link_height_offset is not None
            else float(np.mean(self.min_link_height_offset_samples))
            if self.min_link_height_offset_samples
            else 0.0
        )
        qpos_adj[2] += applied_offset
        return qpos_adj

    @staticmethod
    def _copy_human_motion_data(human_motion_data: Any) -> Optional[Dict[str, Any]]:
        if not isinstance(human_motion_data, dict):
            return None
        copied: Dict[str, Any] = {}
        for key, value in human_motion_data.items():
            if not isinstance(value, (list, tuple)) or len(value) < 2:
                continue
            pos = np.asarray(value[0], dtype=np.float32).copy()
            rot = np.asarray(value[1], dtype=np.float32).copy()
            copied[key] = (pos, rot)
        return copied

    def process_packet(self, packet: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        body_pose_dict = self._body_poses_to_pose_dict(packet.get("poses"))
        if body_pose_dict is None:
            return None

        qpos_curr = self.retarget.retarget(body_pose_dict, offset_to_ground=False)
        if qpos_curr is None:
            return None

        qpos_curr = np.asarray(qpos_curr, dtype=np.float32).reshape(-1)
        if qpos_curr.shape[0] < 36:
            raise ValueError(f"retarget qpos too short: {qpos_curr.shape[0]}")
        qpos_curr = self._apply_min_link_height_offset(qpos_curr[:36])

        return {
            "type": "retarget_result",
            "seq": int(packet["seq"]),
            "recv_ns": int(packet["recv_ns"]),
            "qpos": qpos_curr.astype(np.float32, copy=True),
            "human_motion_data": self._copy_human_motion_data(self.retarget.scaled_human_data)
            if self.send_human_motion
            else None,
        }


def _retarget_worker_main(
    raw_recv_conn: Any,
    result_send_conn: Any,
    worker_config: Dict[str, Any],
) -> None:
    try:
        runtime = _RetargetWorkerRuntime(worker_config)
    except Exception as exc:
        try:
            result_send_conn.send({"type": "worker_init_error", "error": str(exc)})
        except Exception:
            pass
        return

    try:
        result_send_conn.send({"type": "worker_ready"})
    except Exception:
        return

    last_processed_seq = 0
    while True:
        try:
            if not raw_recv_conn.poll(0.1):
                continue
            packet = raw_recv_conn.recv()
        except EOFError:
            break
        except Exception as exc:
            try:
                result_send_conn.send({"type": "worker_runtime_error", "error": str(exc)})
            except Exception:
                pass
            continue

        if isinstance(packet, dict) and packet.get("type") == "shutdown":
            break

        dropped_before_process = 0
        while raw_recv_conn.poll():
            try:
                newer_packet = raw_recv_conn.recv()
            except EOFError:
                newer_packet = None
            if newer_packet is None:
                break
            if isinstance(newer_packet, dict) and newer_packet.get("type") == "shutdown":
                return
            dropped_before_process += 1
            packet = newer_packet

        prev_processed_seq = last_processed_seq
        try:
            result = runtime.process_packet(packet)
        except Exception as exc:
            try:
                result_send_conn.send({"type": "worker_runtime_error", "error": str(exc)})
            except Exception:
                pass
            continue

        if result is None:
            continue

        result["dropped_before_process"] = int(dropped_before_process)
        result["prev_processed_seq"] = int(prev_processed_seq)
        last_processed_seq = int(result["seq"])

        try:
            result_send_conn.send(result)
        except (BrokenPipeError, EOFError, OSError):
            break


class LowLatencyTeleopPoseZMQServer:
    BODY_JOINT_NAMES = XR_BODY_JOINT_NAMES

    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.robot = args.robot
        self.vis_fps = int(args.vis_fps)
        self.ctrl_fps = int(args.ctrl_fps)
        self.lookback_ns = int(float(args.lookback_ms) * 1e6)
        self.retarget_buffer_window_ns = int(float(args.retarget_buffer_window_s) * 1e9)
        self.log_interval_s = float(args.log_interval_s)
        self.publish_lcm_ref = not bool(getattr(args, "disable_lcm_ref", False))
        self.publish_lcm_rc = bool(getattr(args, "enable_lcm_rc", False))
        self.publish_lcm_hand = bool(getattr(args, "enable_lcm_hand", False))
        self.lcm_url = str(getattr(args, "lcm_url", DEFAULT_LCM_URL))
        self.lcm_channel = str(getattr(args, "lcm_channel", DEFAULT_LCM_CHANNEL))
        self.rc_lcm_channel = str(getattr(args, "rc_lcm_channel", DEFAULT_RC_LCM_CHANNEL))
        self.hand_lcm_channel = str(
            getattr(args, "hand_lcm_channel", DEFAULT_HAND_LCM_CHANNEL)
        )
        self.dex3_pose_config = str(
            getattr(args, "dex3_pose_config", DEFAULT_DEX3_POSE_CONFIG)
        )
        dex3_poses = load_dex3_pose_config(self.dex3_pose_config)
        self.controller_mapper = PicoControllerMapper(
            stale_timeout_ms=float(getattr(args, "controller_stale_timeout_ms", 250.0)),
            stick_deadzone=float(getattr(args, "stick_deadzone", 0.1)),
            press_threshold=float(getattr(args, "button_press_threshold", 0.7)),
            release_threshold=float(getattr(args, "button_release_threshold", 0.3)),
            toggle_button=str(getattr(args, "upper_teleop_toggle_button", "right_key_two")),
        )
        self.upper_teleop = UpperTeleopStateMachine(
            blend_s=float(getattr(args, "upper_teleop_blend_s", 0.5)),
            pose_stale_timeout_ms=float(getattr(args, "upper_pose_stale_timeout_ms", 250.0)),
            start_active=bool(getattr(args, "start_upper_teleop_active", False)),
        )
        self.dex3_hand = Dex3HandStateMachine(
            poses=dex3_poses,
            transition_s=float(getattr(args, "hand_transition_s", 0.5)),
            stale_timeout_ms=float(getattr(args, "controller_stale_timeout_ms", 250.0)),
        )

        if self.vis_fps <= 0:
            raise ValueError("vis_fps must be > 0")
        if self.ctrl_fps <= 0:
            raise ValueError("ctrl_fps must be > 0")
        if self.lookback_ns < 0:
            raise ValueError("lookback_ms must be >= 0")
        if self.retarget_buffer_window_ns <= 0:
            raise ValueError("retarget_buffer_window_s must be > 0")
        if self.log_interval_s < 0:
            raise ValueError("log_interval_s must be >= 0")

        self.retarget = None
        self.viewer = None
        self.gmr_max_iter = 5

        self.zmq_context = None
        self.req_sock = None
        self.rep_sock = None
        self.ctrl_sock = None
        self.lcm = None

        self.default_qpos = self._build_default_qpos()
        self.last_controller_buttons: Dict[str, Any] = self._default_controller_buttons()

        self.min_link_height = float(args.min_link_height)
        self.min_link_height_align_strategy = str(args.min_link_height_align_strategy)
        self.min_link_height_bootstrap_frames = max(1, int(args.min_link_height_bootstrap_frames))
        self.fixed_min_link_height_offset: Optional[float] = None
        self.min_link_height_offset_samples: list[float] = []

        self.rotation_matrix = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
        self.rotation_quat = R.from_matrix(self.rotation_matrix).as_quat(scalar_first=True)

        self.latest_vr_lock = threading.Lock()
        self.latest_vr_poses: Optional[Any] = None
        self.latest_vr_recv_ns: int = 0
        self.latest_vr_seq: int = 0
        self.latest_vr_motion_timestamp_ns: Optional[int] = None
        self.latest_controller_recv_ns: int = 0
        self.latest_controller_frame_token: Optional[tuple[Any, ...]] = None

        self.retarget_buffer_lock = threading.Lock()
        self.retarget_buffer: deque[RetargetedFrame] = deque()
        self.vis_lock = threading.Lock()
        self.latest_vis_qpos: Optional[np.ndarray] = None
        self.latest_vis_human_motion: Optional[Dict[str, Any]] = None
        self.latest_upper_dof_pos_lock = threading.Lock()
        self.latest_upper_dof_pos: Optional[np.ndarray] = None
        self.latest_upper_dof_pos_recv_ns: int = 0
        self.lcm_publish_count = 0
        self.lcm_publish_error_reported = False
        self.rc_lcm_publish_count = 0
        self.rc_lcm_publish_error_reported = False
        self.hand_lcm_publish_count = 0
        self.hand_lcm_publish_error_reported = False

        self.vr_frame_event = threading.Event()
        self.stop_event = threading.Event()
        self.stats_lock = threading.Lock()

        self.frame_seq = 0
        self.last_vis_monotonic = 0.0
        self.last_req_monotonic: Optional[float] = None
        self.req_count = 0
        self.reply_count = 0
        self.reply_drop_count = 0
        self.req_merged_total = 0
        self.fallback_count = 0
        self.raw_motion_drop_count = 0
        self.latest_req_dt_ms: Optional[float] = None
        self.latest_merged_reqs = 0

        self.callback_count = 0
        self.retarget_count = 0
        self.latest_debug_info: Dict[str, Any] = {
            "mode": "no_data",
            "target_age_ms": None,
            "older_age_ms": None,
            "newer_age_ms": None,
            "span_ms": None,
            "buffer_len": 0,
            "retarget_age_ms": None,
            "raw_motion_age_ms": None,
        }

        self.retarget_thread = None
        self.raw_sender_thread = None
        self.worker_result_thread = None
        self.request_thread = None
        self.control_thread = None
        self.stats_thread = None
        self.visualization_thread = None

        self.mp_ctx = mp.get_context("spawn")
        self.raw_send_conn = None
        self.raw_recv_conn = None
        self.result_send_conn = None
        self.result_recv_conn = None
        self.retarget_process = None

    def _build_default_qpos(self) -> np.ndarray:
        mimic = np.asarray(DEFAULT_MIMIC_OBS[self.robot], dtype=np.float32).reshape(-1)
        if mimic.shape[0] < 35:
            raise ValueError(f"DEFAULT_MIMIC_OBS[{self.robot}] must be at least 35 dims")
        dof_pos = mimic[6:35]
        root_z = float(mimic[2])
        root_pos = np.array([0.0, 0.0, root_z], dtype=np.float32)
        root_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        return np.concatenate([root_pos, root_quat, dof_pos], axis=0).astype(np.float32)

    @staticmethod
    def _default_controller_buttons() -> Dict[str, Any]:
        return {
            "left_key_one": False,
            "left_key_two": False,
            "left_axis_click": False,
            "left_index_trig": False,
            "left_trigger_value": 0.0,
            "left_grip": False,
            "left_grip_value": 0.0,
            "left_axis": [0.0, 0.0],
            "right_key_one": False,
            "right_key_two": False,
            "right_axis_click": False,
            "right_index_trig": False,
            "right_trigger_value": 0.0,
            "right_grip": False,
            "right_grip_value": 0.0,
            "right_axis": [0.0, 0.0],
        }

    @staticmethod
    def _normalize_quat_wxyz(quat: np.ndarray) -> np.ndarray:
        q = np.asarray(quat, dtype=np.float32).reshape(4)
        norm = float(np.linalg.norm(q))
        if not np.isfinite(norm) or norm < 1e-8:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        return (q / norm).astype(np.float32)

    def _slerp_quat_wxyz(self, quat0: np.ndarray, quat1: np.ndarray, alpha: float) -> np.ndarray:
        q0 = self._normalize_quat_wxyz(quat0).astype(np.float64)
        q1 = self._normalize_quat_wxyz(quat1).astype(np.float64)
        t = float(np.clip(alpha, 0.0, 1.0))

        dot = float(np.dot(q0, q1))
        if dot < 0.0:
            q1 = -q1
            dot = -dot

        if dot > 0.9995:
            out = q0 + t * (q1 - q0)
            return self._normalize_quat_wxyz(out)

        theta_0 = float(np.arccos(np.clip(dot, -1.0, 1.0)))
        sin_theta_0 = float(np.sin(theta_0))
        if abs(sin_theta_0) < 1e-8:
            return self._normalize_quat_wxyz(q0)

        theta = theta_0 * t
        s0 = np.sin(theta_0 - theta) / sin_theta_0
        s1 = np.sin(theta) / sin_theta_0
        out = s0 * q0 + s1 * q1
        return self._normalize_quat_wxyz(out)

    def _interpolate_qpos(self, prev_qpos: np.ndarray, next_qpos: np.ndarray, alpha: float) -> np.ndarray:
        t = float(np.clip(alpha, 0.0, 1.0))
        frame = prev_qpos * (1.0 - t) + next_qpos * t
        frame[3:7] = self._slerp_quat_wxyz(prev_qpos[3:7], next_qpos[3:7], t)
        return frame.astype(np.float32)

    def _extract_controller_buttons_from_snapshot(self, snapshot: Optional[dict]) -> Dict[str, Any]:
        if snapshot is None:
            return self.last_controller_buttons

        controllers = snapshot.get("controllers", {}) if isinstance(snapshot, dict) else {}
        left = controllers.get("left", {}) if isinstance(controllers, dict) else {}
        right = controllers.get("right", {}) if isinstance(controllers, dict) else {}

        def _safe_float(value: Any) -> float:
            try:
                result = float(value)
            except (TypeError, ValueError):
                return 0.0
            return result if np.isfinite(result) else 0.0

        def _safe_bool(value: Any) -> bool:
            if isinstance(value, (bool, np.bool_)):
                return bool(value)
            return _safe_float(value) > 0.5

        def _axis(values: Any) -> list[float]:
            if isinstance(values, (list, tuple, np.ndarray)) and len(values) >= 2:
                try:
                    axis = [float(values[0]), float(values[1])]
                except (TypeError, ValueError):
                    return [0.0, 0.0]
                if np.all(np.isfinite(axis)):
                    return axis
            return [0.0, 0.0]

        left_trigger = _safe_float(left.get("trigger", 0.0))
        left_grip = _safe_float(left.get("grip", 0.0))
        right_trigger = _safe_float(right.get("trigger", 0.0))
        right_grip = _safe_float(right.get("grip", 0.0))

        return {
            "left_key_one": _safe_bool(left.get("primary_button", False)),
            "left_key_two": _safe_bool(left.get("secondary_button", False)),
            "left_axis_click": _safe_bool(left.get("axis_click", False)),
            "left_index_trig": left_trigger > 1e-4,
            "left_trigger_value": left_trigger,
            "left_grip": left_grip > 1e-4,
            "left_grip_value": left_grip,
            "left_axis": _axis(left.get("axis", [0.0, 0.0])),
            "right_key_one": _safe_bool(right.get("primary_button", False)),
            "right_key_two": _safe_bool(right.get("secondary_button", False)),
            "right_axis_click": _safe_bool(right.get("axis_click", False)),
            "right_index_trig": right_trigger > 1e-4,
            "right_trigger_value": right_trigger,
            "right_grip": right_grip > 1e-4,
            "right_grip_value": right_grip,
            "right_axis": _axis(right.get("axis", [0.0, 0.0])),
        }

    @staticmethod
    def _snapshot_has_controller_data(snapshot: Optional[dict]) -> bool:
        if not isinstance(snapshot, dict):
            return False
        controllers = snapshot.get("controllers")
        return (
            isinstance(controllers, dict)
            and isinstance(controllers.get("left"), dict)
            and isinstance(controllers.get("right"), dict)
        )

    @staticmethod
    def _controller_frame_token(snapshot: Optional[dict]) -> Optional[tuple[Any, ...]]:
        """Prefer controller-local timestamps so repeated snapshots eventually go stale."""
        if not isinstance(snapshot, dict):
            return None
        controllers = snapshot.get("controllers")
        if not isinstance(controllers, dict):
            return None

        def _timestamp(value: Any) -> Optional[int]:
            try:
                result = int(value)
            except (TypeError, ValueError, OverflowError):
                return None
            return result if result != 0 else None

        local_tokens = []
        for name in ("left", "right"):
            controller = controllers.get(name)
            timestamp = _timestamp(controller.get("timestamp_ns")) if isinstance(controller, dict) else None
            if timestamp is not None:
                local_tokens.append((name, timestamp))
        if local_tokens:
            return tuple(local_tokens)
        controllers_timestamp = _timestamp(controllers.get("timestamp_ns"))
        if controllers_timestamp is not None:
            return ("controllers", controllers_timestamp)
        snapshot_timestamp = _timestamp(snapshot.get("timestamp_ns"))
        if snapshot_timestamp is not None:
            return ("snapshot", snapshot_timestamp)
        return None

    @staticmethod
    def _serialize_qpos_frame(qpos: np.ndarray) -> Dict[str, Any]:
        q = np.asarray(qpos, dtype=np.float32).reshape(-1)
        return {
            "root_pos": q[0:3].tolist(),
            "root_quat": q[3:7].tolist(),
            "dof_pos": q[7:36].tolist(),
        }

    @staticmethod
    def _extract_upper_dof_pos(qpos: np.ndarray) -> np.ndarray:
        """Extract the 14 absolute arm joint positions from a 36D G1 qpos."""
        q = np.asarray(qpos, dtype=np.float32).reshape(-1)
        expected_qpos_size = 7 + G1_DOF_COUNT
        if q.shape[0] < expected_qpos_size:
            raise ValueError(
                f"G1 qpos must contain at least {expected_qpos_size} values, got {q.shape[0]}"
            )
        return q[
            7 + G1_UPPER_DOF_START : 7 + G1_UPPER_DOF_END
        ].astype(np.float64, copy=True)

    @staticmethod
    def _build_upper_reference_message(upper_dof_pos: np.ndarray) -> ref_upper_dof_pos_lcmt:
        upper = np.asarray(upper_dof_pos, dtype=np.float64).reshape(-1)
        upper_dof_count = G1_UPPER_DOF_END - G1_UPPER_DOF_START
        if upper.shape[0] != upper_dof_count:
            raise ValueError(
                f"upper reference must contain {upper_dof_count} values, got {upper.shape[0]}"
            )
        msg = ref_upper_dof_pos_lcmt()
        msg.ref_upper_dof_pos = upper.tolist()
        return msg

    @staticmethod
    def _build_rc_command_message(rc_state: Dict[str, Any]) -> rc_command_lcmt:
        msg = rc_command_lcmt()
        msg.left_stick = [float(x) for x in rc_state.get("left_stick", [0.0, 0.0])[:2]]
        msg.right_stick = [float(x) for x in rc_state.get("right_stick", [0.0, 0.0])[:2]]
        msg.right_lower_left_switch = int(bool(rc_state.get("r1", False)))
        msg.right_lower_right_switch = int(bool(rc_state.get("r2", False)))
        return msg

    @staticmethod
    def _build_control_payload(
        buttons: Dict[str, Any],
        upper_status: Dict[str, Any],
        wall_time_ms: int,
        hand_status: Optional[Dict[str, Any]] = None,
        hand_enabled: bool = False,
    ) -> Dict[str, Any]:
        payload = {
            "t_ms": int(wall_time_ms),
            "controller_buttons": buttons,
            "upper_teleop": {
                "state": str(upper_status["state"]),
                "pose_fresh": bool(upper_status["pose_fresh"]),
            },
        }
        if hand_status is not None:
            payload["dex3_hand"] = {
                "enabled": bool(hand_enabled),
                "armed": bool(hand_status["armed"]),
                "fresh": bool(hand_status["fresh"]),
                "left": str(hand_status["left"]),
                "right": str(hand_status["right"]),
            }
        return payload

    def _publish_upper_dof_pos(self, upper_dof_pos: np.ndarray) -> None:
        if not self.publish_lcm_ref or self.lcm is None:
            return

        try:
            msg = self._build_upper_reference_message(upper_dof_pos)
            self.lcm.publish(self.lcm_channel, msg.encode())
            self.lcm_publish_count += 1
        except Exception as exc:
            if not self.lcm_publish_error_reported:
                print(f"[Warning] LCM upper reference publish failed: {exc}")
                self.lcm_publish_error_reported = True

    def _publish_rc_command(self, rc_state: Dict[str, Any]) -> None:
        if not self.publish_lcm_rc or self.lcm is None:
            return
        try:
            msg = self._build_rc_command_message(rc_state)
            self.lcm.publish(self.rc_lcm_channel, msg.encode())
            self.rc_lcm_publish_count += 1
        except Exception as exc:
            if not self.rc_lcm_publish_error_reported:
                print(f"[Warning] LCM PICO RC publish failed: {exc}")
                self.rc_lcm_publish_error_reported = True

    @staticmethod
    def _build_hand_action_message(hand_action: np.ndarray) -> hand_action_lcmt:
        action = np.asarray(hand_action, dtype=np.float64)
        if action.shape != (14,):
            raise ValueError(f"Dex3 hand action must have shape (14,), got {action.shape}")
        if not np.all(np.isfinite(action)):
            raise ValueError("Dex3 hand action contains NaN or Inf")
        msg = hand_action_lcmt()
        msg.act = action.tolist()
        return msg

    def _publish_hand_action(self, hand_action: np.ndarray) -> None:
        if not self.publish_lcm_hand or self.lcm is None:
            return
        try:
            msg = self._build_hand_action_message(hand_action)
            self.lcm.publish(self.hand_lcm_channel, msg.encode())
            self.hand_lcm_publish_count += 1
        except Exception as exc:
            if not self.hand_lcm_publish_error_reported:
                print(f"[Warning] LCM Dex3 hand publish failed: {exc}")
                self.hand_lcm_publish_error_reported = True

    def _on_vr_frame(self, snapshot: dict) -> None:
        recv_ns = time.monotonic_ns()
        controller_buttons = self._extract_controller_buttons_from_snapshot(snapshot)
        top_timestamp_ns = None
        try:
            top_timestamp_ns = int(snapshot.get("timestamp_ns", 0)) if isinstance(snapshot, dict) else None
        except Exception:
            top_timestamp_ns = None
        body = snapshot.get("body", {}) if isinstance(snapshot, dict) else {}
        body_available = bool(body.get("available", False)) if isinstance(body, dict) else False
        body_timestamp_ns = None
        if body_available:
            try:
                body_timestamp_ns = int(body.get("timestamp_ns", 0))
            except Exception:
                body_timestamp_ns = None
        motion_timestamp_ns = body_timestamp_ns if body_timestamp_ns not in (None, 0) else top_timestamp_ns

        should_wake_retarget = False
        with self.latest_vr_lock:
            if self._snapshot_has_controller_data(snapshot):
                controller_frame_token = self._controller_frame_token(snapshot)
                if (
                    controller_frame_token is None
                    or controller_frame_token != self.latest_controller_frame_token
                ):
                    self.last_controller_buttons = controller_buttons
                    self.latest_controller_recv_ns = recv_ns
                    self.latest_controller_frame_token = controller_frame_token
            self.callback_count += 1
            if body_available and motion_timestamp_ns is not None:
                if self.latest_vr_motion_timestamp_ns != motion_timestamp_ns:
                    self.latest_vr_poses = body.get("poses", None)
                    self.latest_vr_recv_ns = recv_ns
                    self.latest_vr_seq += 1
                    self.latest_vr_motion_timestamp_ns = motion_timestamp_ns
                    should_wake_retarget = True
        if should_wake_retarget:
            self.vr_frame_event.set()

    def _append_retarget_frame(self, recv_ns: int, qpos: np.ndarray) -> None:
        cutoff_ns = recv_ns - self.retarget_buffer_window_ns
        with self.retarget_buffer_lock:
            self.retarget_buffer.append(RetargetedFrame(recv_ns=recv_ns, qpos=qpos.astype(np.float32, copy=True)))
            while self.retarget_buffer and self.retarget_buffer[0].recv_ns < cutoff_ns:
                self.retarget_buffer.popleft()

    @staticmethod
    def _copy_human_motion_data(human_motion_data: Any) -> Optional[Dict[str, Any]]:
        if not isinstance(human_motion_data, dict):
            return None

        copied: Dict[str, Any] = {}
        for key, value in human_motion_data.items():
            if not isinstance(value, (list, tuple)) or len(value) < 2:
                continue
            pos = np.asarray(value[0], dtype=np.float32).copy()
            rot = np.asarray(value[1], dtype=np.float32).copy()
            copied[key] = (pos, rot)
        return copied

    def _get_retarget_frames_snapshot(self) -> list[RetargetedFrame]:
        with self.retarget_buffer_lock:
            return list(self.retarget_buffer)

    def _sample_target_qpos(self, frames: list[RetargetedFrame], target_ns: int) -> tuple[np.ndarray, bool, Dict[str, Any]]:
        if not frames:
            return self.default_qpos.copy(), True, {
                "mode": "default",
                "target_ns": target_ns,
                "older_ns": None,
                "newer_ns": None,
                "alpha": None,
                "buffer_len": 0,
        }
        if len(frames) == 1:
            only_ns = frames[0].recv_ns
            return frames[0].qpos.astype(np.float32, copy=True), True, {
                "mode": "single_frame",
                "target_ns": target_ns,
                "older_ns": only_ns,
                "newer_ns": only_ns,
                "alpha": None,
                "buffer_len": 1,
            }
        if target_ns <= frames[0].recv_ns:
            oldest_ns = frames[0].recv_ns
            return frames[0].qpos.astype(np.float32, copy=True), True, {
                "mode": "fallback_oldest",
                "target_ns": target_ns,
                "older_ns": oldest_ns,
                "newer_ns": oldest_ns,
                "alpha": None,
                "buffer_len": len(frames),
            }
        if target_ns >= frames[-1].recv_ns:
            latest_ns = frames[-1].recv_ns
            return frames[-1].qpos.astype(np.float32, copy=True), True, {
                "mode": "fallback_latest",
                "target_ns": target_ns,
                "older_ns": latest_ns,
                "newer_ns": latest_ns,
                "alpha": None,
                "buffer_len": len(frames),
            }

        for idx in range(1, len(frames)):
            prev_frame = frames[idx - 1]
            next_frame = frames[idx]
            if target_ns <= next_frame.recv_ns:
                dt = next_frame.recv_ns - prev_frame.recv_ns
                if dt <= 0:
                    same_ns = next_frame.recv_ns
                    return next_frame.qpos.astype(np.float32, copy=True), True, {
                        "mode": "degenerate_dt",
                        "target_ns": target_ns,
                        "older_ns": same_ns,
                        "newer_ns": same_ns,
                        "alpha": None,
                        "buffer_len": len(frames),
                    }
                alpha = float(target_ns - prev_frame.recv_ns) / float(dt)
                return self._interpolate_qpos(prev_frame.qpos, next_frame.qpos, alpha), False, {
                    "mode": "interpolate",
                    "target_ns": target_ns,
                    "older_ns": prev_frame.recv_ns,
                    "newer_ns": next_frame.recv_ns,
                    "alpha": alpha,
                    "buffer_len": len(frames),
                }

        latest_ns = frames[-1].recv_ns
        return frames[-1].qpos.astype(np.float32, copy=True), True, {
            "mode": "fallback_latest",
            "target_ns": target_ns,
            "older_ns": latest_ns,
            "newer_ns": latest_ns,
            "alpha": None,
            "buffer_len": len(frames),
        }

    def _build_reply_frames(self, req_recv_ns: int) -> tuple[list[np.ndarray], bool, Dict[str, Any]]:
        frames = self._get_retarget_frames_snapshot()
        target_base_ns = req_recv_ns - self.lookback_ns
        qpos, used_fallback, sample_info = self._sample_target_qpos(frames, target_base_ns)
        return [qpos], used_fallback, sample_info

    def _get_latest_frame_ages_ms(self, now_ns: Optional[int] = None) -> tuple[Optional[float], Optional[float]]:
        if now_ns is None:
            now_ns = time.monotonic_ns()

        with self.latest_vr_lock:
            latest_raw_recv_ns = int(self.latest_vr_recv_ns) if self.latest_vr_recv_ns > 0 else None

        with self.retarget_buffer_lock:
            latest_retarget_recv_ns = self.retarget_buffer[-1].recv_ns if self.retarget_buffer else None

        raw_motion_age_ms = None
        if latest_raw_recv_ns is not None:
            raw_motion_age_ms = round((now_ns - latest_raw_recv_ns) / 1e6, 3)

        retarget_age_ms = None
        if latest_retarget_recv_ns is not None:
            retarget_age_ms = round((now_ns - latest_retarget_recv_ns) / 1e6, 3)

        return retarget_age_ms, raw_motion_age_ms

    def _update_debug_info(self, sample_info: Dict[str, Any], req_recv_ns: int) -> None:
        older_ns = sample_info.get("older_ns")
        newer_ns = sample_info.get("newer_ns")
        retarget_age_ms, raw_motion_age_ms = self._get_latest_frame_ages_ms()

        info = {
            "mode": sample_info.get("mode"),
            "target_age_ms": round((req_recv_ns - int(sample_info["target_ns"])) / 1e6, 3),
            "older_age_ms": None if older_ns is None else round((req_recv_ns - int(older_ns)) / 1e6, 3),
            "newer_age_ms": None if newer_ns is None else round((req_recv_ns - int(newer_ns)) / 1e6, 3),
            "span_ms": None
            if older_ns is None or newer_ns is None
            else round((int(newer_ns) - int(older_ns)) / 1e6, 3),
            "alpha": sample_info.get("alpha"),
            "buffer_len": int(sample_info.get("buffer_len", 0)),
            "retarget_age_ms": retarget_age_ms,
            "raw_motion_age_ms": raw_motion_age_ms,
        }
        with self.stats_lock:
            self.latest_debug_info = info

    def _warn_on_fallback(self, sample_info: Dict[str, Any]) -> None:
        now_ns = time.monotonic_ns()
        retarget_age_ms, raw_motion_age_ms = self._get_latest_frame_ages_ms(now_ns=now_ns)
        target_age_ms = round((now_ns - int(sample_info["target_ns"])) / 1e6, 3)
        older_ns = sample_info.get("older_ns")
        newer_ns = sample_info.get("newer_ns")
        older_age_ms = None if older_ns is None else round((now_ns - int(older_ns)) / 1e6, 3)
        newer_age_ms = None if newer_ns is None else round((now_ns - int(newer_ns)) / 1e6, 3)
        print(
            "[Warning] interpolation fallback "
            f"mode={sample_info.get('mode')}, "
            f"target_age_ms={target_age_ms}, "
            f"older_age_ms={older_age_ms}, "
            f"newer_age_ms={newer_age_ms}, "
            f"buffer={int(sample_info.get('buffer_len', 0))}, "
            f"latest_retarget_age_ms={retarget_age_ms}, "
            f"latest_raw_motion_age_ms={raw_motion_age_ms}"
        )

    def _warn_on_raw_motion_drop(self, dropped_count: int, latest_seq: int, last_processed_seq: int) -> None:
        now_ns = time.monotonic_ns()
        retarget_age_ms, raw_motion_age_ms = self._get_latest_frame_ages_ms(now_ns=now_ns)
        print(
            "[Warning] retarget lag dropped raw motion frames "
            f"dropped={int(dropped_count)}, "
            f"last_processed_seq={int(last_processed_seq)}, "
            f"latest_seq={int(latest_seq)}, "
            f"latest_retarget_age_ms={retarget_age_ms}, "
            f"latest_raw_motion_age_ms={raw_motion_age_ms}"
        )

    def _raw_sender_loop(self) -> None:
        last_sent_seq = 0

        while not self.stop_event.is_set():
            if not self.vr_frame_event.wait(timeout=0.1):
                continue

            while not self.stop_event.is_set():
                with self.latest_vr_lock:
                    poses = self.latest_vr_poses
                    recv_ns = self.latest_vr_recv_ns
                    seq = self.latest_vr_seq

                if poses is None or seq == last_sent_seq:
                    with self.latest_vr_lock:
                        if self.latest_vr_seq == last_sent_seq:
                            self.vr_frame_event.clear()
                            break
                    continue

                if last_sent_seq != 0 and seq > last_sent_seq + 1:
                    dropped_count = seq - last_sent_seq - 1
                    self.raw_motion_drop_count += int(dropped_count)
                    self._warn_on_raw_motion_drop(
                        dropped_count=dropped_count,
                        latest_seq=seq,
                        last_processed_seq=last_sent_seq,
                    )

                try:
                    self.raw_send_conn.send(
                        {
                            "seq": int(seq),
                            "recv_ns": int(recv_ns),
                            "poses": poses,
                        }
                    )
                except (BrokenPipeError, EOFError, OSError) as exc:
                    print(f"[Warning] raw->worker pipe failed: {exc}")
                    self.stop_event.set()
                    self.vr_frame_event.set()
                    break

                last_sent_seq = seq

    def _worker_result_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                if not self.result_recv_conn.poll(0.1):
                    continue
                payload = self.result_recv_conn.recv()
            except EOFError:
                print("[Warning] worker->main pipe closed")
                self.stop_event.set()
                break
            except Exception as exc:
                print(f"[Warning] worker result recv failed: {exc}")
                self.stop_event.set()
                break

            if not isinstance(payload, dict):
                continue

            payload_type = payload.get("type")
            if payload_type == "worker_ready":
                continue
            if payload_type in ("worker_init_error", "worker_runtime_error"):
                print(f"[Warning] retarget worker error: {payload.get('error')}")
                if payload_type == "worker_init_error":
                    self.stop_event.set()
                continue
            if payload_type != "retarget_result":
                continue

            dropped_before_process = int(payload.get("dropped_before_process", 0))
            if dropped_before_process > 0:
                self.raw_motion_drop_count += dropped_before_process
                self._warn_on_raw_motion_drop(
                    dropped_count=dropped_before_process,
                    latest_seq=int(payload.get("seq", 0)),
                    last_processed_seq=int(payload.get("prev_processed_seq", 0)),
                )

            qpos_curr = np.asarray(payload.get("qpos"), dtype=np.float32).reshape(-1)
            recv_ns = int(payload["recv_ns"])
            self._append_retarget_frame(recv_ns=recv_ns, qpos=qpos_curr)
            self.retarget_count += 1

            try:
                upper_dof_pos = self._extract_upper_dof_pos(qpos_curr)
            except ValueError as exc:
                print(f"[Warning] invalid retarget qpos for upper reference: {exc}")
            else:
                with self.latest_upper_dof_pos_lock:
                    self.latest_upper_dof_pos = upper_dof_pos
                    self.latest_upper_dof_pos_recv_ns = recv_ns

            if self.viewer is not None:
                with self.vis_lock:
                    self.latest_vis_qpos = qpos_curr.astype(np.float32, copy=True)
                    self.latest_vis_human_motion = payload.get("human_motion_data")

    def _drain_requests_blocking(self) -> tuple[Optional[Dict[str, Any]], Optional[int], int]:
        import zmq

        poller = zmq.Poller()
        poller.register(self.req_sock, zmq.POLLIN)

        while not self.stop_event.is_set():
            events = dict(poller.poll(timeout=100))
            if self.req_sock not in events:
                continue

            latest_req: Optional[Dict[str, Any]] = None
            req_recv_ns: Optional[int] = None
            merged_reqs = 0
            any_start = False

            while True:
                try:
                    raw = self.req_sock.recv_string(flags=zmq.NOBLOCK)
                    req_recv_ns = time.monotonic_ns()
                except zmq.Again:
                    break
                except Exception as exc:
                    print(f"[Warning] request recv failed: {exc}")
                    break

                try:
                    req = json.loads(raw)
                except Exception:
                    print("[Warning] bad request JSON")
                    continue
                if not isinstance(req, dict):
                    continue

                merged_reqs += 1
                any_start = any_start or bool(req.get("start", False))
                latest_req = req

            if latest_req is None:
                continue

            latest_req["start"] = any_start
            return latest_req, req_recv_ns, merged_reqs

        return None, None, 0

    def _request_loop(self) -> None:
        import zmq

        while not self.stop_event.is_set():
            req, req_recv_ns, merged_reqs = self._drain_requests_blocking()
            if req is None or req_recv_ns is None:
                continue

            now = time.monotonic()
            self.req_count += 1
            self.req_merged_total += int(merged_reqs)
            self.latest_merged_reqs = int(merged_reqs)
            if self.last_req_monotonic is None:
                self.latest_req_dt_ms = None
            else:
                self.latest_req_dt_ms = (now - self.last_req_monotonic) * 1000.0
            self.last_req_monotonic = now

            out_frames, used_fallback, sample_info = self._build_reply_frames(req_recv_ns=req_recv_ns)
            self._update_debug_info(sample_info=sample_info, req_recv_ns=req_recv_ns)
            if used_fallback:
                self.fallback_count += 1
                self._warn_on_fallback(sample_info=sample_info)
            seq_start = int(self.frame_seq)
            self.frame_seq += len(out_frames)

            retarget_frames = self._get_retarget_frames_snapshot()
            retarget_age_ms = None
            if retarget_frames:
                retarget_age_ms = int((time.monotonic_ns() - retarget_frames[-1].recv_ns) / 1e6)

            payload = {
                "start": bool(req.get("start", False)),
                "no_interp_applied": bool(used_fallback),
                "chunk_size": len(out_frames),
                "frame_seq_start": seq_start,
                "retarget_age_ms": retarget_age_ms,
                "t_rep_ms": int(time.time() * 1000),
                "frames": [self._serialize_qpos_frame(x) for x in out_frames],
            }

            try:
                self.rep_sock.send_string(json.dumps(payload), flags=zmq.NOBLOCK)
                self.reply_count += 1
            except zmq.Again:
                self.reply_drop_count += 1
                print("[Warning] reply queue full, drop one reply")
            except Exception as exc:
                print(f"[Warning] reply send failed: {exc}")

    def _stats_loop(self) -> None:
        while not self.stop_event.is_set():
            if self.stop_event.wait(timeout=self.log_interval_s):
                break

            with self.stats_lock:
                info = dict(self.latest_debug_info)
            with self.latest_vr_lock:
                callback_count = int(self.callback_count)
            retarget_count = int(self.retarget_count)
            req_count = int(self.req_count)
            reply_count = int(self.reply_count)
            reply_drop_count = int(self.reply_drop_count)
            req_merged_total = int(self.req_merged_total)
            fallback_count = int(self.fallback_count)
            raw_motion_drop_count = int(self.raw_motion_drop_count)
            latest_merged_reqs = int(self.latest_merged_reqs)
            latest_req_dt_ms = self.latest_req_dt_ms

            alpha = info.get("alpha")
            alpha_str = "None" if alpha is None else f"{float(alpha):.3f}"
            req_dt_str = "None" if latest_req_dt_ms is None else f"{float(latest_req_dt_ms):.2f}"
            print(
                "[Stats] "
                f"req={req_count}, rep={reply_count}, rep_drop={reply_drop_count}, "
                f"req_merged_total={req_merged_total}, latest_merged={latest_merged_reqs}, "
                f"fallback={fallback_count}, raw_drop={raw_motion_drop_count}, "
                f"cb={callback_count}, retarget={retarget_count}, "
                f"mode={info.get('mode')}, buffer={info.get('buffer_len')}, "
                f"latest_req_dt_ms={req_dt_str}, "
                f"target_age_ms={info.get('target_age_ms')}, "
                f"older_age_ms={info.get('older_age_ms')}, "
                f"newer_age_ms={info.get('newer_age_ms')}, "
                f"span_ms={info.get('span_ms')}, alpha={alpha_str}, "
                f"retarget_age_ms={info.get('retarget_age_ms')}, "
                f"raw_motion_age_ms={info.get('raw_motion_age_ms')}"
            )

    def _control_loop(self) -> None:
        import zmq

        period_s = 1.0 / float(self.ctrl_fps)
        while not self.stop_event.is_set():
            now_ns = time.monotonic_ns()
            with self.latest_vr_lock:
                buttons = dict(self.last_controller_buttons)
                controller_recv_ns = int(self.latest_controller_recv_ns)
            with self.latest_upper_dof_pos_lock:
                upper_pose = None if self.latest_upper_dof_pos is None else self.latest_upper_dof_pos.copy()
                upper_pose_recv_ns = int(self.latest_upper_dof_pos_recv_ns)

            rc_state = self.controller_mapper.update(buttons, controller_recv_ns, now_ns)
            hand_output, hand_status = self.dex3_hand.update(
                buttons, controller_recv_ns, now_ns
            )
            upper_output, upper_status, notice = self.upper_teleop.step(
                now_ns=now_ns,
                live_pose=upper_pose,
                live_pose_recv_ns=upper_pose_recv_ns,
                controller_fresh=bool(rc_state["fresh"]),
                controller_armed=bool(rc_state["armed"]),
                toggle_rising=bool(rc_state["toggle_rising"]),
            )
            if notice:
                print(f"[Info] {notice}")

            payload = self._build_control_payload(
                buttons=buttons,
                upper_status=upper_status,
                wall_time_ms=int(time.time() * 1000),
                hand_status=hand_status,
                hand_enabled=self.publish_lcm_hand,
            )
            try:
                self.ctrl_sock.send_string(json.dumps(payload), flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            except Exception as exc:
                print(f"[Warning] control send failed: {exc}")

            self._publish_upper_dof_pos(upper_output)
            self._publish_rc_command(rc_state)
            self._publish_hand_action(hand_output)

            self.stop_event.wait(timeout=period_s)

    def _visualization_loop(self) -> None:
        if self.viewer is None:
            return

        period_s = 1.0 / float(self.vis_fps)
        while not self.stop_event.is_set():
            with self.vis_lock:
                qpos = None if self.latest_vis_qpos is None else self.latest_vis_qpos.copy()
                human_motion_data = self.latest_vis_human_motion

            if qpos is not None:
                try:
                    self.viewer.step(
                        root_pos=qpos[:3],
                        root_rot=qpos[3:7],
                        dof_pos=qpos[7:36],
                        human_motion_data=human_motion_data,
                        rate_limit=False,
                        follow_camera=True,
                    )
                except Exception as exc:
                    print(f"[Warning] visualization failed, disabling viewer: {exc}")
                    self.viewer.close()
                    self.viewer = None
                    return

            self.stop_event.wait(timeout=period_s)

    def setup(self) -> None:
        try:
            import zmq
        except ImportError as exc:
            raise ImportError("pyzmq is required for the teleop ZMQ server.") from exc

        if self.args.visualize:
            self.viewer = RobotMotionViewer(
                robot_type=self.robot,
                motion_fps=self.vis_fps,
                transparent_robot=1,
            )

        if self.publish_lcm_ref or self.publish_lcm_rc or self.publish_lcm_hand:
            try:
                import lcm
            except ImportError as exc:
                raise ImportError(
                    "Python 'lcm' is required for upper-reference/PICO-RC/Dex3 publishing. "
                    "Install it or disable all LCM outputs."
                ) from exc
            self.lcm = lcm.LCM(self.lcm_url)

        self.raw_recv_conn, self.raw_send_conn = self.mp_ctx.Pipe(duplex=False)
        self.result_recv_conn, self.result_send_conn = self.mp_ctx.Pipe(duplex=False)
        worker_config = {
            "actual_human_height": float(self.args.actual_human_height),
            "gmr_max_iter": int(self.gmr_max_iter),
            "send_human_motion": bool(self.args.visualize),
            "min_link_height": self.min_link_height,
            "min_link_height_align_strategy": self.min_link_height_align_strategy,
            "min_link_height_bootstrap_frames": self.min_link_height_bootstrap_frames,
        }
        self.retarget_process = self.mp_ctx.Process(
            target=_retarget_worker_main,
            args=(self.raw_recv_conn, self.result_send_conn, worker_config),
            name="teleop-retarget-worker",
            daemon=True,
        )
        self.retarget_process.start()
        self.raw_recv_conn.close()
        self.raw_recv_conn = None
        self.result_send_conn.close()
        self.result_send_conn = None

        if not self.result_recv_conn.poll(10.0):
            raise RuntimeError("Retarget worker did not become ready within 10 seconds.")
        worker_msg = self.result_recv_conn.recv()
        if not isinstance(worker_msg, dict) or worker_msg.get("type") != "worker_ready":
            raise RuntimeError(f"Retarget worker failed to start: {worker_msg}")

        xrt.init()
        xrt.register_frame_callback(self._on_vr_frame)

        self.zmq_context = zmq.Context.instance()

        self.req_sock = self.zmq_context.socket(zmq.PULL)
        self.req_sock.setsockopt(zmq.LINGER, 0)
        self.req_sock.setsockopt(zmq.RCVHWM, 500)
        self.req_sock.bind(self.args.req_bind_addr)

        self.rep_sock = self.zmq_context.socket(zmq.PUSH)
        self.rep_sock.setsockopt(zmq.LINGER, 0)
        self.rep_sock.setsockopt(zmq.SNDHWM, 500)
        self.rep_sock.bind(self.args.rep_bind_addr)

        self.ctrl_sock = self.zmq_context.socket(zmq.PUSH)
        self.ctrl_sock.setsockopt(zmq.LINGER, 0)
        self.ctrl_sock.setsockopt(zmq.SNDHWM, 500)
        self.ctrl_sock.bind(self.args.ctrl_bind_addr)

        print("Low-latency teleop ZMQ pose server initialized")
        print(f"  req_bind_addr: {self.args.req_bind_addr}")
        print(f"  rep_bind_addr: {self.args.rep_bind_addr}")
        print(f"  ctrl_bind_addr: {self.args.ctrl_bind_addr}")
        print(f"  ctrl_fps: {self.ctrl_fps}")
        print(f"  gmr_max_iter: {self.gmr_max_iter}")
        print("  chunk_size: fixed to 1 frame per reply")
        print(f"  lookback_ms: {self.lookback_ns / 1e6:.3f}")
        print(f"  retarget_buffer_window_s: {self.retarget_buffer_window_ns / 1e9:.3f}")
        print(f"  log_interval_s: {self.log_interval_s:.3f}")
        print(f"  visualize: {self.args.visualize}")
        print(f"  lcm_ref_enabled: {self.publish_lcm_ref}")
        print(f"  lcm_rc_enabled: {self.publish_lcm_rc}")
        print(f"  lcm_hand_enabled: {self.publish_lcm_hand}")
        if self.publish_lcm_ref or self.publish_lcm_rc or self.publish_lcm_hand:
            print(f"  lcm_url: {self.lcm_url}")
        if self.publish_lcm_ref:
            print(f"  lcm_channel: {self.lcm_channel}")
        if self.publish_lcm_rc:
            print(f"  rc_lcm_channel: {self.rc_lcm_channel}")
        if self.publish_lcm_hand:
            print(f"  hand_lcm_channel: {self.hand_lcm_channel}")
            print(f"  dex3_pose_config: {self.dex3_pose_config}")
        print(f"  upper_teleop_initial_state: {self.upper_teleop.state}")
        print(f"  retarget_worker_pid: {self.retarget_process.pid if self.retarget_process else None}")

    def run(self) -> None:
        self.setup()

        self.raw_sender_thread = threading.Thread(
            target=self._raw_sender_loop,
            name="teleop-raw-sender",
            daemon=True,
        )
        self.worker_result_thread = threading.Thread(
            target=self._worker_result_loop,
            name="teleop-worker-result",
            daemon=True,
        )
        self.request_thread = threading.Thread(
            target=self._request_loop,
            name="teleop-request",
            daemon=True,
        )
        self.control_thread = threading.Thread(
            target=self._control_loop,
            name="teleop-control",
            daemon=True,
        )
        if self.viewer is not None:
            self.visualization_thread = threading.Thread(
                target=self._visualization_loop,
                name="teleop-visualization",
                daemon=True,
            )
        if self.log_interval_s > 0.0:
            self.stats_thread = threading.Thread(
                target=self._stats_loop,
                name="teleop-stats",
                daemon=True,
            )

        self.raw_sender_thread.start()
        self.worker_result_thread.start()
        self.request_thread.start()
        self.control_thread.start()
        if self.visualization_thread is not None:
            self.visualization_thread.start()
        if self.stats_thread is not None:
            self.stats_thread.start()

        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("KeyboardInterrupt, exiting low-latency teleop ZMQ pose server.")
        finally:
            self.stop_event.set()
            self.vr_frame_event.set()
            try:
                xrt.clear_frame_callback()
            except Exception:
                pass

            for thread in (
                self.raw_sender_thread,
                self.worker_result_thread,
                self.request_thread,
                self.control_thread,
                self.visualization_thread,
                self.stats_thread,
            ):
                if thread is not None:
                    thread.join(timeout=1.0)

            if self.raw_send_conn is not None:
                try:
                    self.raw_send_conn.send({"type": "shutdown"})
                except Exception:
                    pass
            if self.raw_send_conn is not None:
                self.raw_send_conn.close()
            if self.raw_recv_conn is not None:
                self.raw_recv_conn.close()
            if self.result_send_conn is not None:
                self.result_send_conn.close()
            if self.result_recv_conn is not None:
                self.result_recv_conn.close()
            if self.retarget_process is not None:
                self.retarget_process.join(timeout=2.0)
                if self.retarget_process.is_alive():
                    self.retarget_process.terminate()
                    self.retarget_process.join(timeout=1.0)

            if self.viewer is not None:
                self.viewer.close()
            if self.req_sock is not None:
                self.req_sock.close(0)
            if self.rep_sock is not None:
                self.rep_sock.close(0)
            if self.ctrl_sock is not None:
                self.ctrl_sock.close(0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Low-latency ZMQ teleop pose server")
    parser.add_argument(
        "--robot",
        choices=["unitree_g1", "unitree_g1_with_hands"],
        default="unitree_g1",
        help="Robot key for defaults",
    )
    parser.add_argument("--actual_human_height", type=float, default=1.6)
    parser.add_argument("--vis_fps", type=int, default=10, help="Viewer update frequency")
    parser.add_argument(
        "--ctrl_fps",
        type=int,
        default=50,
        help="Controller button and LCM output publish frequency",
    )
    parser.add_argument(
        "--lookback_ms",
        type=float,
        default=15.0,
        help="Sample reply frames at request_time - lookback_ms",
    )
    parser.add_argument(
        "--retarget_buffer_window_s",
        type=float,
        default=0.5,
        help="How much retarget history to keep for timestamp interpolation",
    )
    parser.add_argument(
        "--log_interval_s",
        type=float,
        default=1.0,
        help="Periodic debug log interval. Set to 0 to disable.",
    )
    parser.add_argument("--req_bind_addr", type=str, default="tcp://*:28701")
    parser.add_argument("--rep_bind_addr", type=str, default="tcp://*:28702")
    parser.add_argument("--ctrl_bind_addr", type=str, default="tcp://*:28703")
    parser.add_argument("--lcm_url", type=str, default=DEFAULT_LCM_URL)
    parser.add_argument("--lcm_channel", type=str, default=DEFAULT_LCM_CHANNEL)
    parser.add_argument(
        "--disable_lcm_ref",
        action="store_true",
        help="Disable publishing the upper-body reference over LCM",
    )
    parser.add_argument(
        "--enable_lcm_rc",
        action="store_true",
        help="Publish conditioned PICO controls as rc_command_lcmt",
    )
    parser.add_argument("--rc_lcm_channel", type=str, default=DEFAULT_RC_LCM_CHANNEL)
    parser.add_argument(
        "--enable_lcm_hand",
        action="store_true",
        help="Publish A/X-controlled Dex3 targets as hand_action_lcmt",
    )
    parser.add_argument("--hand_lcm_channel", type=str, default=DEFAULT_HAND_LCM_CHANNEL)
    parser.add_argument(
        "--dex3_pose_config",
        type=str,
        default=str(DEFAULT_DEX3_POSE_CONFIG),
        help="JSON file containing left/right open and closed Dex3 poses",
    )
    parser.add_argument("--hand_transition_s", type=float, default=0.5)
    parser.add_argument("--controller_stale_timeout_ms", type=float, default=250.0)
    parser.add_argument("--stick_deadzone", type=float, default=0.1)
    parser.add_argument("--button_press_threshold", type=float, default=0.7)
    parser.add_argument("--button_release_threshold", type=float, default=0.3)
    parser.add_argument("--upper_teleop_toggle_button", type=str, default="right_key_two")
    parser.add_argument("--upper_teleop_blend_s", type=float, default=0.5)
    parser.add_argument("--upper_pose_stale_timeout_ms", type=float, default=250.0)
    parser.add_argument(
        "--start_upper_teleop_active",
        action="store_true",
        help="Start upper teleop after controls are released and a fresh pose is available",
    )
    parser.add_argument("--min_link_height", type=float, default=0.0)
    parser.add_argument(
        "--min_link_height_align_strategy",
        type=str,
        choices=["startup_fixed", "per_frame"],
        default="startup_fixed",
    )
    parser.add_argument("--min_link_height_bootstrap_frames", type=int, default=10)
    parser.add_argument("--visualize", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    _load_runtime_dependencies()
    server = LowLatencyTeleopPoseZMQServer(args)
    server.run()


if __name__ == "__main__":
    main()
