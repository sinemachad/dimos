# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import time

import cv2
import numpy as np

from dimos.manipulation.grasp_planner import (
    OBB,
    GraspPlanner,
    KinpyIKSolver,
    PyBulletCollisionChecker,
    TableSpec,
)
from dimos.manipulation.grasp_planner.integration import (
    grasps_to_candidates,
    rank_and_split_viz_from_results,
    update_pybullet_obstacles_from_results,
)
from dimos.manipulation.manip_aio_processer import ManipulationProcessor

try:
    import pybullet as p  # type: ignore[import-not-found]
except Exception:  # pragma: no cover
    p = None  # type: ignore[assignment]


def load_rgb_depth(rgb_path: str, depth_path: str) -> tuple[np.ndarray, np.ndarray]:
    rgb = cv2.cvtColor(cv2.imread(rgb_path, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    if depth_path.lower().endswith(".npy"):
        depth = np.load(depth_path).astype(np.float32)
    else:
        depth_raw = cv2.imread(depth_path, cv2.IMREAD_ANYDEPTH)
        if depth_raw is None:
            raise FileNotFoundError(f"Cannot read depth image: {depth_path}")
        # Heuristic: if depth is in millimeters (common), convert to meters
        depth = depth_raw.astype(np.float32)
        if depth.max() > 50.0:
            depth *= 0.001
    return rgb, depth


def synthesize_grasps_from_objects(
    objects: list[dict], z_offset: float = 0.06, yaw_samples: int = 8
) -> list[dict]:
    """
    Fallback: generate top-down grasps above object centers if AnyGrasp isn't configured.
    """
    grasps = []
    if not objects:
        return grasps
    for obj in objects:
        pos = obj.get("position")
        size = obj.get("size", {})
        if pos is None:
            continue
        if hasattr(pos, "x"):
            cx, cy, cz = float(pos.x), float(pos.y), float(pos.z)  # type: ignore[attr-defined]
        else:
            cx = float(pos.get("x", 0.0))
            cy = float(pos.get("y", 0.0))
            cz = float(pos.get("z", 0.0))
        width = float(size.get("width", 0.06))
        for k in range(yaw_samples):
            yaw = (2.0 * np.pi * k) / yaw_samples
            c, s = np.cos(yaw), np.sin(yaw)
            Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)
            T = np.eye(4, dtype=float)
            T[:3, :3] = Rz
            T[:3, 3] = np.array([cx, cy, cz + z_offset], dtype=float)
            grasps.append(
                {
                    "translation": T[:3, 3].tolist(),
                    "rotation_matrix": T[:3, :3].tolist(),
                    "width": width,
                    "score": 0.5,
                }
            )
    return grasps


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Run ManipulationProcessor + GraspPlanner with PyBullet collision."
    )
    ap.add_argument("--rgb", type=str, required=True, help="Path to RGB image")
    ap.add_argument(
        "--depth", type=str, required=True, help="Path to depth image (.npy meters or .png mm)"
    )
    ap.add_argument("--fx", type=float, required=True)
    ap.add_argument("--fy", type=float, required=True)
    ap.add_argument("--cx", type=float, required=True)
    ap.add_argument("--cy", type=float, required=True)
    ap.add_argument("--table_height", type=float, default=0.75)
    ap.add_argument("--pybullet_gui", action="store_true")
    ap.add_argument(
        "--grasp_server_url", type=str, default=None, help="ws://... for hosted grasp generator"
    )
    ap.add_argument("--loop", action="store_true", help="Loop display until 'q'")
    ap.add_argument(
        "--sim_render",
        action="store_true",
        help="Render RGB-D from PyBullet instead of reading files",
    )
    ap.add_argument("--sim_width", type=int, default=960)
    ap.add_argument("--sim_height", type=int, default=540)
    ap.add_argument("--sim_fov_deg", type=float, default=60.0)
    ap.add_argument("--sim_near", type=float, default=0.1)
    ap.add_argument("--sim_far", type=float, default=2.5)
    args = ap.parse_args()

    camera_intrinsics = [args.fx, args.fy, args.cx, args.cy]

    # Perception
    mp = ManipulationProcessor(
        camera_intrinsics=camera_intrinsics,
        enable_grasp_generation=bool(args.grasp_server_url),
        grasp_server_url=args.grasp_server_url,
        enable_segmentation=True,
        min_confidence=0.6,
        max_objects=15,
    )

    # Collision checker (PyBullet)
    table_half = np.array([0.6, 0.4, 0.03], dtype=float)
    table_center = np.array([0.5, 0.0, args.table_height - table_half[2]], dtype=float)
    checker = PyBulletCollisionChecker(
        urdf_path="/home/jalaj/DimosGraspPlanner/dimos/dimos/dimos/hardware/xarm6_description.urdf",
        table_spec=TableSpec(size_xyz=table_half * 2, center_world=table_center),
        use_gui=args.pybullet_gui,
        self_collision=True,
    )
    # If rendering in-sim, populate scene with some random OBB "products"
    sim_obstacles: list[OBB] = []
    if "args" in locals() and getattr(args, "sim_render", False):
        rng = np.random.RandomState(42)
        for _ in range(6):
            center = np.array(
                [
                    float(rng.uniform(0.25, 0.6)),
                    float(rng.uniform(-0.25, 0.25)),
                    float(args.table_height + rng.uniform(0.02, 0.08)),
                ]
            )
            yaw = float(rng.uniform(-np.pi, np.pi))
            c, s = np.cos(yaw), np.sin(yaw)
            Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)
            half = np.array(
                [
                    float(rng.uniform(0.03, 0.07)),
                    float(rng.uniform(0.02, 0.06)),
                    float(rng.uniform(0.02, 0.06)),
                ]
            )
            sim_obstacles.append(OBB(center=center, rotation=Rz, half_extents=half))
        checker.set_obstacles(sim_obstacles)

    # IK + Planner
    ik = KinpyIKSolver(urdf_path="dimos/hardware/piper_description.urdf", end_link="gripper_base")
    planner = GraspPlanner(ik, checker, topdown_max_angle_deg=25.0, table_margin_m=0.02)

    # Robot state (example; replace with your live state)
    current_q = np.zeros(6, dtype=float)
    ql = np.array([-2.8, -2.1, -2.8, -2.8, -2.8, -2.8], dtype=float)
    qh = np.array([2.8, 2.1, 2.8, 2.8, 2.8, 2.8], dtype=float)

    while True:
        if args.sim_render:
            if p is None:
                raise RuntimeError("pybullet not installed; pip install pybullet")
            cam_pos = np.array([0.5, -0.6, args.table_height + 0.55])
            cam_target = np.array([0.5, 0.0, args.table_height])
            cam_up = np.array([0.0, 0.0, 1.0])
            view = p.computeViewMatrix(
                cameraEyePosition=cam_pos.tolist(),
                cameraTargetPosition=cam_target.tolist(),
                cameraUpVector=cam_up.tolist(),
                physicsClientId=checker.client,  # type: ignore[attr-defined]
            )
            proj = p.computeProjectionMatrixFOV(
                fov=args.sim_fov_deg,
                aspect=float(args.sim_width) / float(args.sim_height),
                nearVal=args.sim_near,
                farVal=args.sim_far,
                physicsClientId=checker.client,  # type: ignore[attr-defined]
            )
            imgw, imgh = int(args.sim_width), int(args.sim_height)
            _w, _h, rgba, depth_buf, _seg = p.getCameraImage(
                width=imgw,
                height=imgh,
                viewMatrix=view,
                projectionMatrix=proj,
                renderer=p.ER_TINY_RENDERER,
                physicsClientId=checker.client,  # type: ignore[attr-defined]
            )
            rgba = np.array(rgba, dtype=np.uint8).reshape(imgh, imgw, 4)
            rgb = cv2.cvtColor(rgba[..., :3], cv2.COLOR_BGR2RGB)
            depth_buf = np.array(depth_buf, dtype=np.float32).reshape(imgh, imgw)
            near, far = float(args.sim_near), float(args.sim_far)
            depth = (far * near) / (far - (far - near) * depth_buf + 1e-9)
            # Update intrinsics from FOV
            fx = 0.5 * imgw / np.tan(np.deg2rad(args.sim_fov_deg) * 0.5)
            fy = 0.5 * imgh / np.tan(np.deg2rad(args.sim_fov_deg) * 0.5)
            cx = imgw / 2.0
            cy = imgh / 2.0
            camera_intrinsics = [fx, fy, cx, cy]
            mp.camera_intrinsics = camera_intrinsics
        else:
            rgb, depth = load_rgb_depth(args.rgb, args.depth)
        results = mp.process_frame(rgb, depth)

        # Ensure grasps exist; if not, synthesize from objects
        if not results.get("grasps"):
            results["grasps"] = synthesize_grasps_from_objects(results.get("all_objects", []) or [])

        # If perception failed to produce objects and we rendered the scene, fall back to sim obstacles for objects
        if (not results.get("all_objects")) and sim_obstacles:
            sim_objs = []
            for ob in sim_obstacles:
                # convert OBB to object dict with position/rotation(size) expected keys
                center = ob.center
                R = ob.rotation
                sy = float(np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
                singular = sy < 1e-6
                if not singular:
                    roll = float(np.arctan2(R[2, 1], R[2, 2]))
                    pitch = float(np.arctan2(-R[2, 0], sy))
                    yaw = float(np.arctan2(R[1, 0], R[0, 0]))
                else:
                    roll = float(np.arctan2(-R[1, 2], R[1, 1]))
                    pitch = float(np.arctan2(-R[2, 0], sy))
                    yaw = 0.0
                sim_objs.append(
                    {
                        "position": {
                            "x": float(center[0]),
                            "y": float(center[1]),
                            "z": float(center[2]),
                        },
                        "rotation": {"x": roll, "y": pitch, "z": yaw},
                        "size": {
                            "width": float(ob.half_extents[0] * 2.0),
                            "height": float(ob.half_extents[1] * 2.0),
                            "depth": float(ob.half_extents[2] * 2.0),
                        },
                    }
                )
            results["all_objects"] = sim_objs

        # Update PyBullet obstacles from objects
        update_pybullet_obstacles_from_results(checker, results, inflate=0.005)

        # Visual overlays and ranking
        viz_all, viz_feasible, ranked = rank_and_split_viz_from_results(
            results,
            planner,
            current_q=current_q,
            joint_limits_low=ql,
            joint_limits_high=qh,
            table_height=args.table_height,
            camera_intrinsics=camera_intrinsics,
            rgb_image=rgb,
            ee_start_pose_world=None,
            top_k=10,
        )

        # Best grasp + q
        best = planner.plan(
            [r[0] for r in ranked],
            current_q=current_q,
            joint_limits_low=ql,
            joint_limits_high=qh,
            table_height=args.table_height,
            ee_start_pose_world=None,
        )
        if best:
            cand, q_sol = best
            checker._set_joint_state(q_sol)  # type: ignore[attr-defined]
            print(
                f"Best score={ranked[0][1]:.3f}, pos={cand.pose_world[:3, 3].tolist()}, q={q_sol.tolist()}"
            )
        else:
            print("No feasible grasp this frame.")

        # Compose split view
        left = cv2.cvtColor(viz_all, cv2.COLOR_RGB2BGR)
        right = cv2.cvtColor(viz_feasible, cv2.COLOR_RGB2BGR)
        h = max(left.shape[0], right.shape[0])
        left = (
            cv2.resize(left, (right.shape[1], right.shape[0]))
            if left.shape != right.shape
            else left
        )
        split = np.hstack([left, right])
        cv2.imshow("Grasps: all (left, red) vs feasible (right, green)", split)
        key = cv2.waitKey(1) & 0xFF
        if not args.loop or key == ord("q"):
            break
        time.sleep(0.05)

    checker.close()
    mp.cleanup()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
