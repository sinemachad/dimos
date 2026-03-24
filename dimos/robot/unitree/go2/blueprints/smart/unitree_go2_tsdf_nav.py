#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Unitree Go2 blueprint with TSDF-based dynamic global mapping.

Adds TSDFMap on top of the base Go2 stack, remapping:
  - ``lidar``     -> ``registered_scan`` (TSDFMap input)
  - ``go2_odom``  -> ``raw_odom``        (TSDFMap input)

The TSDFMap publishes:
  - ``global_map``  - occupied-voxel point cloud (replaces VoxelGridMapper output)
  - ``odom``        - pass-through of raw_odom for downstream planners
"""

from dimos.core.blueprints import autoconnect
from dimos.navigation.tsdf_map.module import TSDFMap
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

unitree_go2_tsdf_nav = (
    autoconnect(
        unitree_go2,
        TSDFMap.blueprint(
            voxel_size=0.15,
            sdf_trunc=0.3,
            max_range=15.0,
            map_publish_rate=0.5,
        ),
    )
    .remappings(
        [
            # Wire the robot's LiDAR output to TSDFMap's scan input
            (TSDFMap, "registered_scan", "lidar"),
            # Wire the robot's odometry to TSDFMap's pose input
            (TSDFMap, "raw_odom", "go2_odom"),
        ]
    )
    .global_config(n_workers=8, robot_model="unitree_go2")
)

__all__ = ["unitree_go2_tsdf_nav"]
