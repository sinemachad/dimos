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

"""Tests for TSDFMap module."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.navigation.tsdf_map.module import TSDFMap, TSDFMapConfig


class TestTSDFMapConfig:
    def test_defaults(self) -> None:
        cfg = TSDFMapConfig()
        assert cfg.voxel_size == pytest.approx(0.15)
        assert cfg.sdf_trunc == pytest.approx(0.3)
        assert cfg.max_range == pytest.approx(15.0)
        assert cfg.max_weight == pytest.approx(50.0)

    def test_custom(self) -> None:
        cfg = TSDFMapConfig(voxel_size=0.1, sdf_trunc=0.2)
        assert cfg.voxel_size == pytest.approx(0.1)


class TestTSDFCore:
    def _make(self) -> TSDFMap:
        return TSDFMap()

    def test_empty_map(self) -> None:
        m = self._make()
        try:
            assert len(m._voxels) == 0
        finally:
            m.stop()

    def test_integrate_creates_voxels(self) -> None:
        m = self._make()
        try:
            origin = np.array([0.0, 0.0, 0.0])
            pts = np.array([[3.0, y, 0.5] for y in np.arange(-1, 1, 0.3)])
            m._integrate_points(pts, origin)
            assert len(m._voxels) > 0
        finally:
            m.stop()

    def test_dynamic_clearing(self) -> None:
        """Obstacle moves within truncation range — SDF at old position shifts.

        TSDF clearing is truncation-range limited: new hits update voxels
        within ±sdf_trunc of the new hit. A large sdf_trunc allows clearing
        old obstacles when the new hit is within that range.
        """
        m = TSDFMap(sdf_trunc=3.0)  # large trunc so clearing rays reach old position
        try:
            origin = np.array([0.0, 0.0, 0.0])

            # Wall at x=3 (axis-aligned, ray perfectly along x)
            wall3 = np.array([[3.0, 0.0, 0.0]])
            for _ in range(5):
                m._integrate_points(wall3, origin)

            vs = m.config.voxel_size
            vk3 = (int(np.floor(3.0 / vs)), 0, 0)
            assert vk3 in m._voxels, f"Expected voxel at {vk3}"
            sdf_before = m._voxels[vk3][0]

            # Wall moved to x=5 (within sdf_trunc=3.0 of x=3).
            # Ray from origin hits x=5; with trunc=3.0 the update band is
            # t=[2.0, 8.0], which includes x=3.
            wall5 = np.array([[5.0, 0.0, 0.0]])
            for _ in range(20):
                m._integrate_points(wall5, origin)

            assert vk3 in m._voxels
            sdf_after = m._voxels[vk3][0]
            # SDF at x=3 was ≈0 (surface). After rays hit x=5 passing through
            # x=3, the SDF = 5-3 = 2.0 (free space). Weighted average shifts positive.
            assert sdf_after > sdf_before, (
                f"SDF should increase: {sdf_before:.3f} -> {sdf_after:.3f}"
            )
        finally:
            m.stop()

    def test_weight_capping(self) -> None:
        m = self._make()
        try:
            origin = np.array([0.0, 0.0, 0.0])
            pt = np.array([[2.0, 0.0, 0.0]])
            for _ in range(100):
                m._integrate_points(pt, origin)
            for _, (_, w) in m._voxels.items():
                assert w <= m.config.max_weight
        finally:
            m.stop()

    def test_zero_crossing_extraction(self) -> None:
        m = self._make()
        try:
            origin = np.array([0.0, 0.0, 0.0])
            wall = np.array([[3.0, y, 0.5] for y in np.arange(-1, 1, 0.2)])
            for _ in range(5):
                m._integrate_points(wall, origin)

            threshold = m.config.voxel_size
            near_zero = [
                (vk, s, w) for vk, (s, w) in m._voxels.items() if abs(s) < threshold and w > 2.0
            ]
            assert len(near_zero) > 0
        finally:
            m.stop()

    def test_keyframe_gating(self) -> None:
        m = self._make()
        try:
            pos1 = np.array([0.0, 0.0, 0.0])
            assert m._should_integrate(pos1) is True
            # Same position — should skip
            assert m._should_integrate(pos1) is False
            # Move past threshold
            pos2 = np.array([1.0, 0.0, 0.0])
            assert m._should_integrate(pos2) is True
        finally:
            m.stop()


class TestTSDFMapStreams:
    def test_streams_declared(self) -> None:
        """TSDFMap must declare the four expected streams."""
        from typing import get_origin, get_type_hints

        from dimos.core.stream import In, Out

        hints = get_type_hints(TSDFMap, include_extras=True)
        in_streams = {k for k, v in hints.items() if get_origin(v) is In}
        out_streams = {k for k, v in hints.items() if get_origin(v) is Out}

        assert "registered_scan" in in_streams
        assert "raw_odom" in in_streams
        assert "global_map" in out_streams
        assert "odom" in out_streams


class TestTSDFMapPickle:
    def test_module_pickle(self) -> None:
        """TSDFMap must survive a pickle/unpickle round-trip."""
        import pickle

        m = TSDFMap(voxel_size=0.2, sdf_trunc=0.4)
        data = pickle.dumps(m)
        m2 = pickle.loads(data)
        import pytest as _pytest

        assert isinstance(m2, TSDFMap)
        assert m2.config.voxel_size == _pytest.approx(0.2)
        m.stop()
        m2.stop()


class TestTSDFMapRangeFilter:
    def test_max_range_config(self) -> None:
        """max_range config is stored correctly."""
        m = TSDFMap(max_range=3.0)
        try:
            assert m.config.max_range == pytest.approx(3.0)
        finally:
            m.stop()

    def test_close_points_integrate(self) -> None:
        """Points within max_range should create voxels."""
        m = TSDFMap(max_range=10.0)
        try:
            origin = np.array([0.0, 0.0, 0.0])
            close_pts = np.array([[2.0, 0.0, 0.0]])
            m._integrate_points(close_pts, origin)
            assert len(m._voxels) > 0
        finally:
            m.stop()
