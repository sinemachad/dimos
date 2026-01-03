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

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Protocol

import numpy as np

from dimos.perception.grasp_generation.utils import (
    create_grasp_overlay_colored,
)

from .types import GraspCandidate
from .world_collision import OBB

if TYPE_CHECKING:
    from collections.abc import Iterable

    from .planner import GraspPlanner


class _ObstacleSink(Protocol):
    def set_obstacles(self, obstacles: list[OBB]) -> None: ...


def grasps_to_candidates(grasps: Iterable[dict]) -> list[GraspCandidate]:
    """
    Convert HostedGraspGenerator results (list of dicts with 'translation' and 'rotation_matrix')
    into GraspCandidate objects.
    """
    candidates: list[GraspCandidate] = []
    for g in grasps:
        try:
            t = np.array(g.get("translation", [0, 0, 0]), dtype=float).reshape(3)
            R = np.array(g.get("rotation_matrix", np.eye(3)), dtype=float).reshape(3, 3)
            T = np.eye(4, dtype=float)
            T[:3, :3] = R
            T[:3, 3] = t
            score = float(g.get("score", 0.0))
            width = float(g.get("width", 0.0)) if "width" in g and g["width"] is not None else None
            candidates.append(GraspCandidate(pose_world=T, score=score, width=width))
        except Exception:
            continue
    return candidates


def select_grasp_from_processor_results(
    results: dict,
    planner: GraspPlanner,
    current_q: np.ndarray,
    joint_limits_low: np.ndarray,
    joint_limits_high: np.ndarray,
    table_height: float,
    ee_start_pose_world: np.ndarray | None = None,
) -> tuple[GraspCandidate, np.ndarray] | None:
    """
    Convenience function to plug planner on top of ManipulationProcessor.process_frame() outputs.
    Expects results['grasps'] to be a list of dictionaries in HostedGraspGenerator format.
    """
    grasps = results.get("grasps", []) or []
    grasp_candidates = grasps_to_candidates(grasps)
    if not grasp_candidates:
        return None
    return planner.plan(
        grasp_candidates,
        current_q=current_q,
        joint_limits_low=joint_limits_low,
        joint_limits_high=joint_limits_high,
        table_height=table_height,
        ee_start_pose_world=ee_start_pose_world,
    )


def objects_to_obstacles(objects: list[dict], inflate: float = 0.0) -> list[OBB]:
    """
    Convert ManipulationProcessor `objects` entries into OBBs for collision.
    Expects each object to have 'position', 'rotation' (roll,pitch,yaw), and 'size'.
    """
    obstacles: list[OBB] = []
    for obj in objects:
        try:
            pos = obj.get("position")
            rot = obj.get("rotation")
            size = obj.get("size")
            if pos is None or rot is None or size is None:
                continue
            center = np.array([pos.x, pos.y, pos.z], dtype=float)
            roll, pitch, yaw = float(rot.x), float(rot.y), float(rot.z)
            R = _euler_zyx_to_matrix(roll, pitch, yaw)
            half = np.array(
                [
                    float(size.get("width", 0.1)) * 0.5 + inflate,
                    float(size.get("height", 0.1)) * 0.5 + inflate,
                    float(size.get("depth", 0.1)) * 0.5 + inflate,
                ],
                dtype=float,
            )
            source_id = str(obj.get("object_id", "")) if "object_id" in obj else None
            obstacles.append(OBB(center=center, rotation=R, half_extents=half, source_id=source_id))
        except Exception:
            continue
    return obstacles


def _euler_zyx_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Return rotation matrix for ZYX euler angles defined as roll=x, pitch=y, yaw=z.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    return Rz @ Ry @ Rx


def rank_and_split_viz_from_results(
    results: dict,
    planner: GraspPlanner,
    current_q: np.ndarray,
    joint_limits_low: np.ndarray,
    joint_limits_high: np.ndarray,
    table_height: float,
    camera_intrinsics: np.ndarray | list[float],
    rgb_image: np.ndarray,
    ee_start_pose_world: np.ndarray | None = None,
    top_k: int | None = None,
) -> tuple[np.ndarray, np.ndarray, list[tuple[GraspCandidate, float]]]:
    """
    Produce two overlays (all vs feasible) and a ranked list of (candidate, score).
    """
    grasps = results.get("grasps", []) or []
    grasp_candidates = grasps_to_candidates(grasps)
    ranked = planner.plan_ranked(
        grasp_candidates,
        current_q=current_q,
        joint_limits_low=joint_limits_low,
        joint_limits_high=joint_limits_high,
        table_height=table_height,
        ee_start_pose_world=ee_start_pose_world,
        top_k=top_k,
    )

    # Overlays
    overlay_all = create_grasp_overlay_colored(
        rgb_image, grasps, camera_intrinsics, color_bgr=(0, 0, 255)
    )
    feasible_grasps = [p.candidate for p in ranked]
    feasible_dicts = [
        {
            "translation": c.pose_world[:3, 3].tolist(),
            "rotation_matrix": c.pose_world[:3, :3].tolist(),
            "width": float(c.width) if c.width is not None else 0.04,
            "score": float(p.combined_score),
        }
        for c, p in zip(feasible_grasps, ranked, strict=False)
    ]
    overlay_feasible = create_grasp_overlay_colored(
        rgb_image, feasible_dicts, camera_intrinsics, color_bgr=(0, 255, 0)
    )

    ranked_pairs = [(p.candidate, float(p.combined_score)) for p in ranked]
    return overlay_all, overlay_feasible, ranked_pairs


def update_pybullet_obstacles_from_results(
    checker: _ObstacleSink,
    results: dict,
    inflate: float = 0.005,
) -> None:
    """
    Update a collision checker that supports set_obstacles(...) with obstacles built
    from ManipulationProcessor 'all_objects' entries.
    """
    objs = results.get("all_objects", []) or []
    obs = objects_to_obstacles(objs, inflate=inflate)
    checker.set_obstacles(obs)
