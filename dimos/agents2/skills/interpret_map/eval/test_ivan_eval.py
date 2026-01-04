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

from collections.abc import Callable
from dataclasses import dataclass
from functools import cached_property
import json
from pathlib import Path
import pickle
import re
from typing import TYPE_CHECKING

import cv2
from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations
import numpy as np
import pytest

from dimos.core import LCMTransport
from dimos.models.vl.base import VlModel
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.models.vl.qwen import QwenVlModel
from dimos.msgs.geometry_msgs import Pose, PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.type import Detection2DBBox, Detection2DPoint, ImageDetections2D
from dimos.protocol.tf import TF
from dimos.utils.data import get_data
from dimos.utils.generic import extract_json_from_llm_response


def goal_placement_prompt(description: str) -> str:
    prompt = (
        "Look at this image carefully \n"
        "it represents a 2D map map perceived by a robot looked from above (like a floor plan).\n"
        " - red circle is the robot, UP in the image is always FORWARD for the robot.\n"
        " - white color represents free space, black color are walls or obstacles\n"
        f"Identify a location in free space based on the following description:\n{description}\n"
    )

    return prompt


@dataclass
class State:
    robot_pose: PoseStamped | None = None
    target: PoseStamped | None = None
    transforms: list[Transform] | None = None
    image: Image | None = None
    resolution: float = 0.05
    model: VlModel | Callable[[], VlModel] = MoondreamVlModel
    detections: ImageDetections2D | None = None

    def __post_init__(self):
        if callable(self.model):
            self.model = self.model()

    @classmethod
    def from_image(cls, name: str, **kwargs):
        return cls(
            image=Image.from_file(get_data("agent_occupancygrid_experiments") / name), **kwargs
        )

    def query(self, query: str) -> PoseStamped:
        query = goal_placement_prompt(query)
        print(query)
        self.detections = self.model.query_points(self.image, query)
        print(self.detections)
        return self.detections

    @property
    def costmap(self) -> OccupancyGrid:
        """
        Build OccupancyGrid from map image`.
        """
        # read image and convert to grid 1:1
        # expects rgb image with black as obstacles, white as free space and gray as unknown
        image_arr = self.image.to_rgb().data
        height, width = image_arr.shape[:2]
        grid = np.full((height, width), 100, dtype=np.int8)  # obstacle by default

        # drop alpha channel if present
        if image_arr.shape[2] == 4:
            image_arr = image_arr[:, :, :3]

        # define colors and threshold
        WHITE = np.array([255, 255, 255], dtype=np.float32)
        GRAY = np.array([127, 127, 127], dtype=np.float32)  # approx RGB for 127 gray
        white_threshold = 30
        gray_threshold = 10

        # convert to float32 for distance calculations
        image_float = image_arr.astype(np.float32)

        # calculate distances to target colors using broadcasting
        white_dist = np.sqrt(np.sum((image_float - WHITE) ** 2, axis=2))
        gray_dist = np.sqrt(np.sum((image_float - GRAY) ** 2, axis=2))

        # assign based on closest color within threshold
        grid[white_dist <= white_threshold] = 0  # Free space
        grid[gray_dist <= gray_threshold] = -1  # Unknown space

        # build OccupancyGrid object
        occupancy_grid = OccupancyGrid()
        occupancy_grid.info.width = width
        occupancy_grid.info.height = height
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.grid = grid
        occupancy_grid.frame_id = "world"
        occupancy_grid.info.origin.position = Vector3(0.0, 0.0, 0.0)
        occupancy_grid.info.origin.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        return occupancy_grid


@pytest.fixture
def vl_model():
    return QwenVlModel()


@pytest.fixture(scope="session")
def publish_state():
    def publish(state: State):
        if state.transforms:
            tf = TF()
            tf.publish(*state.transforms)
            tf.stop()

        if state.target:
            pose: LCMTransport[PoseStamped] = LCMTransport("/target", PoseStamped)
            pose.publish(target)
            pose.lcm.stop()

        if state.costmap:
            costmap: LCMTransport[OccupancyGrid] = LCMTransport("/costmap", OccupancyGrid)
            costmap.publish(state.costmap)
            costmap.lcm.stop()

        if state.image:
            agent_image: LCMTransport[OccupancyGrid] = LCMTransport("/agent_image", Image)
            agent_image.publish(state.image)
            agent_image.lcm.stop()

        if state.detections:
            annotations: LCMTransport[ImageAnnotations] = LCMTransport(
                "/annotations", ImageAnnotations
            )
            annotations.publish(state.detections.to_foxglove_annotations())
            annotations.lcm.stop()

    yield publish


def test_basic_image(publish_state, vl_model):
    state = State.from_image("ivan1.png")
    state.query("open area")
    publish_state(state)
    # target = target_from_llm(
    #     vl_model,
    #     grid_generator,
    #     "hallway in front of the robot",
    # )

    # publish_state(grid_generator)
