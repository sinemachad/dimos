# Copyright 2026 Dimensional Inc.
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

"""Memory module — ingests Image, PointCloud2, and pose into dimos.memory streams."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import TYPE_CHECKING, Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.memory.impl.sqlite import SqliteStore
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.memory.store import Session
    from dimos.memory.stream import EmbeddingStream, Stream

logger = setup_logger()


@dataclass
class MemoryModuleConfig(ModuleConfig):
    db_path: str = "memory.db"
    world_frame: str = "world"
    robot_frame: str = "base_link"
    image_fps: float = 5.0
    # CLIP embedding pipeline
    enable_clip: bool = False
    sharpness_window: float = 0.5


class MemoryModule(Module[MemoryModuleConfig]):
    """Ingests images and point clouds into persistent memory streams.

    Pose is obtained implicitly from the TF system (world -> base_link).
    Optionally builds a CLIP embedding index with sharpness-based quality filtering.

    Usage::

        memory = dimos.deploy(MemoryModule, db_path="/data/robot.db")
        memory.color_image.connect(camera.color_image)
        memory.pointcloud.connect(lidar.pointcloud)
        memory.start()

        # Query via session
        session = memory.session
        results = session.stream("images").after(t).near(pose, 5.0).fetch()
    """

    color_image: In[Image]
    lidar: In[PointCloud2]

    default_config: type[MemoryModuleConfig] = MemoryModuleConfig

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._store: SqliteStore | None = None
        self._sessions: list[Session] = []
        self._session: Session | None = None
        self._images: Stream[Image] | None = None
        self._pointclouds: Stream[PointCloud2] | None = None
        self._embeddings: EmbeddingStream[Any] | None = None

    # ── Lifecycle ─────────────────────────────────────────────────────

    def _open_session(self) -> Session:
        """Open a new session (own connection) from the shared store."""
        assert self._store is not None
        session = self._store.session()
        self._sessions.append(session)
        return session

    @rpc
    def start(self) -> None:
        super().start()

        import cv2

        cv2.setNumThreads(1)

        self._store = SqliteStore(self.config.db_path)

        pose_fn = lambda: self.tf.get_pose(  # noqa: E731
            self.config.world_frame, self.config.robot_frame
        )

        # Each session opens its own connection (WAL mode allows concurrent writes).

        # Image stream (best-sharpness per window, no rx windowing overhead)
        img_session = self._open_session()
        self._images = img_session.stream("images", Image, pose_provider=pose_fn)
        self._img_window = 1.0 / self.config.image_fps
        self._img_best: Image | None = None
        self._img_best_score: float = -1.0
        self._img_window_start: float = 0.0
        self._disposables.add(self.color_image.observable().subscribe(on_next=self._on_image))

        # Pointcloud stream (only if transport is connected)
        if self.lidar._transport is not None:
            pc_session = self._open_session()
            self._pointclouds = pc_session.stream("pointclouds", PointCloud2, pose_provider=pose_fn)
            self._disposables.add(self.lidar.observable().subscribe(on_next=self._on_pointcloud))

        # Read session (for queries / list_streams)
        self._session = self._open_session()

        # Optional CLIP embedding pipeline
        if self.config.enable_clip:
            self._setup_clip_pipeline()

        logger.info("MemoryModule started (db=%s)", self.config.db_path)

    def _setup_clip_pipeline(self) -> None:
        from dimos.memory.transformer import EmbeddingTransformer, QualityWindowTransformer
        from dimos.models.embedding.clip import CLIPModel

        assert self._images is not None

        clip = CLIPModel()
        clip.start()

        sharp = self._images.transform(
            QualityWindowTransformer(
                lambda img: img.sharpness, window=self.config.sharpness_window
            ),
            live=True,
        ).store("sharp_frames", Image)

        self._embeddings = sharp.transform(  # type: ignore[assignment]
            EmbeddingTransformer(clip), live=True
        ).store("clip_embeddings")

        logger.info("CLIP embedding pipeline active")

    @rpc
    def stop(self) -> None:
        # Flush the last sharpness window so the final image isn't lost
        if self._img_best is not None and self._images is not None:
            self._images.append(self._img_best, ts=self._img_best.ts)
            self._img_best = None
        self._session = None
        for session in self._sessions:
            session.close()
        self._sessions.clear()
        if self._store is not None:
            self._store.close()
            self._store = None
        super().stop()

    # ── Callbacks ─────────────────────────────────────────────────────

    def _on_image(self, img: Image) -> None:
        if self._images is None:
            return
        now = time.monotonic()
        score = img.sharpness
        if now - self._img_window_start >= self._img_window:
            # Window elapsed — flush best from previous window, start new one
            if self._img_best is not None:
                self._images.append(self._img_best, ts=self._img_best.ts)
            self._img_best = img
            self._img_best_score = score
            self._img_window_start = now
        elif score > self._img_best_score:
            self._img_best = img
            self._img_best_score = score

    def _on_pointcloud(self, pc: PointCloud2) -> None:
        if self._pointclouds is not None:
            self._pointclouds.append(pc, ts=pc.ts)

    # ── Public API ────────────────────────────────────────────────────

    @property
    def session(self) -> Session:
        if self._session is None:
            raise RuntimeError("MemoryModule not started")
        return self._session

    @property
    def images(self) -> Stream[Image]:
        if self._images is None:
            raise RuntimeError("MemoryModule not started")
        return self._images

    @property
    def pointclouds(self) -> Stream[PointCloud2]:
        if self._pointclouds is None:
            raise RuntimeError("MemoryModule not started or no pointcloud connected")
        return self._pointclouds

    @property
    def embeddings(self) -> EmbeddingStream[Any] | None:
        return self._embeddings

    @rpc
    def get_stats(self) -> dict[str, int]:
        if self._session is None:
            return {}
        return {s.name: s.count for s in self._session.list_streams()}


memory_module = MemoryModule.blueprint
