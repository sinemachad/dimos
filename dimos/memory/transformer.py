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

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Generic, TypeVar

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.models.embedding.base import Embedding, EmbeddingModel

    from .stream import Stream
    from .types import Observation

T = TypeVar("T")
R = TypeVar("R")


class Transformer(ABC, Generic[T, R]):
    """Transforms a source stream into results on a target stream."""

    supports_backfill: bool = True
    supports_live: bool = True

    @abstractmethod
    def process(self, source: Stream[T], target: Stream[R]) -> None:
        """Batch/historical processing.

        Has full access to the source stream — can query, filter, batch, skip, etc.
        """

    def on_append(self, obs: Observation, target: Stream[R]) -> None:
        """Reactive per-item processing. Called for each new item."""


class PerItemTransformer(Transformer[T, R]):
    """Wraps a simple callable as a per-item Transformer."""

    def __init__(self, fn: Callable[[T], R | list[R] | None]) -> None:
        self._fn = fn

    def process(self, source: Stream[T], target: Stream[R]) -> None:
        for page in source.fetch_pages():
            for obs in page:
                self._apply(obs, target)

    def on_append(self, obs: Observation, target: Stream[R]) -> None:
        self._apply(obs, target)

    def _apply(self, obs: Observation, target: Stream[R]) -> None:
        result = self._fn(obs.data)
        if result is None:
            return
        if isinstance(result, list):
            for item in result:
                target.append(item, ts=obs.ts, pose=obs.pose, tags=obs.tags)
        else:
            target.append(result, ts=obs.ts, pose=obs.pose, tags=obs.tags)


class EmbeddingTransformer(Transformer[Any, "Embedding"]):
    """Wraps an EmbeddingModel as a Transformer that produces Embedding output.

    When stored, the output stream becomes an EmbeddingStream with vector index.
    """

    supports_backfill: bool = True
    supports_live: bool = True

    def __init__(self, model: EmbeddingModel) -> None:
        self.model = model

    def process(self, source: Stream[Any], target: Stream[Embedding]) -> None:
        for page in source.fetch_pages():
            images = [obs.data for obs in page]
            if not images:
                continue
            embeddings = self.model.embed(*images)
            if not isinstance(embeddings, list):
                embeddings = [embeddings]
            for obs, emb in zip(page, embeddings, strict=True):
                target.append(emb, ts=obs.ts, pose=obs.pose, tags=obs.tags)

    def on_append(self, obs: Observation, target: Stream[Embedding]) -> None:
        emb = self.model.embed(obs.data)
        if isinstance(emb, list):
            emb = emb[0]
        target.append(emb, ts=obs.ts, pose=obs.pose, tags=obs.tags)
