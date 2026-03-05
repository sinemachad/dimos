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

import importlib
import pickle
from typing import TYPE_CHECKING, Any, Protocol, TypeVar

if TYPE_CHECKING:
    from dimos.msgs.protocol import DimosMsg

T = TypeVar("T")


class Codec(Protocol[T]):
    """Encodes/decodes payloads for storage."""

    def encode(self, value: T) -> bytes: ...
    def decode(self, data: bytes) -> T: ...


class LcmCodec:
    """Codec for DimosMsg types — uses lcm_encode/lcm_decode."""

    def __init__(self, msg_type: type[DimosMsg]) -> None:
        self._msg_type = msg_type

    def encode(self, value: DimosMsg) -> bytes:
        return value.lcm_encode()

    def decode(self, data: bytes) -> DimosMsg:
        return self._msg_type.lcm_decode(data)


class PickleCodec:
    """Fallback codec for arbitrary Python objects."""

    def encode(self, value: Any) -> bytes:
        return pickle.dumps(value, protocol=pickle.HIGHEST_PROTOCOL)

    def decode(self, data: bytes) -> Any:
        return pickle.loads(data)


_POSE_CODEC: LcmCodec | None = None


def _pose_codec() -> LcmCodec:
    global _POSE_CODEC
    if _POSE_CODEC is None:
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        _POSE_CODEC = LcmCodec(PoseStamped)
    return _POSE_CODEC


def codec_for_type(payload_type: type | None) -> LcmCodec | PickleCodec:
    """Auto-select codec based on payload type."""
    if (
        payload_type is not None
        and hasattr(payload_type, "lcm_encode")
        and hasattr(payload_type, "lcm_decode")
    ):
        return LcmCodec(payload_type)  # type: ignore[arg-type]
    return PickleCodec()


def type_to_module_path(t: type) -> str:
    """Return fully qualified module path for a type, e.g. 'dimos.msgs.sensor_msgs.Image.Image'."""
    return f"{t.__module__}.{t.__qualname__}"


def module_path_to_type(path: str) -> type | None:
    """Resolve a fully qualified module path back to a type. Returns None on failure."""
    parts = path.rsplit(".", 1)
    if len(parts) != 2:
        return None
    module_path, class_name = parts
    try:
        mod = importlib.import_module(module_path)
        return getattr(mod, class_name, None)  # type: ignore[no-any-return]
    except (ImportError, AttributeError):
        return None
