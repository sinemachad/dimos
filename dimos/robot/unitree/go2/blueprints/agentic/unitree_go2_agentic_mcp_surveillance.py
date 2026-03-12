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

from dimos_lcm.foxglove_msgs.ImageAnnotations import (
    ImageAnnotations,  # type: ignore[import-untyped]
)

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport, pLCMTransport
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.module2D import Detection2DModule
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic_mcp import (
    unitree_go2_agentic_mcp,
)
from dimos.robot.unitree.go2.connection import GO2Connection
from hackathon.surveillance_skill import SurveillanceSkill

unitree_go2_agentic_mcp_surveillance = (
    autoconnect(
        unitree_go2_agentic_mcp,
        SurveillanceSkill.blueprint(),
        Detection2DModule.blueprint(
            camera_info=GO2Connection.camera_info_static,
            max_freq=2,  # 2Hz is enough for people tracking
        ),
    )
    .transports(
        {
            ("detections", Detection2DArray): LCMTransport(
                "/detector2d/detections", Detection2DArray
            ),
            ("annotations", ImageAnnotations): pLCMTransport("/detector2d/annotations"),
            ("detected_image_0", Image): LCMTransport("/detector2d/image/0", Image),
            ("detected_image_1", Image): LCMTransport("/detector2d/image/1", Image),
            ("detected_image_2", Image): LCMTransport("/detector2d/image/2", Image),
        }
    )
    .global_config(n_workers=10)
)

__all__ = ["unitree_go2_agentic_mcp_surveillance"]
