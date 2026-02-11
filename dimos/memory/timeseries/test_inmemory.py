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
"""Tests specific to InMemoryStore."""

from dataclasses import dataclass

import pytest

from dimos.memory.timeseries.inmemory import InMemoryStore
from dimos.types.timestamped import Timestamped


@dataclass
class SampleData(Timestamped):
    """Simple timestamped data for testing."""

    value: str

    def __init__(self, value: str, ts: float) -> None:
        super().__init__(ts)
        self.value = value

    def __eq__(self, other: object) -> bool:
        if isinstance(other, SampleData):
            return self.value == other.value and self.ts == other.ts
        return False


class TestInMemoryStoreOperations:
    """Test InMemoryStore-specific operations."""

    def test_delete(self):
        store: InMemoryStore[SampleData] = InMemoryStore()
        store.save(SampleData("a", 1.0))
        store.save(SampleData("b", 2.0))
        assert len(store) == 2
        deleted = store._delete(1.0)
        assert deleted == SampleData("a", 1.0)
        assert len(store) == 1
        assert store.load(1.0) is None


@pytest.mark.tool
class TestPerformance:
    """Benchmarks comparing InMemoryStore vs TimestampedCollection.

    GC is disabled during measurements to avoid non-deterministic pauses.
    """

    N = 100_000

    def _make_populated_store(self) -> InMemoryStore[SampleData]:
        store: InMemoryStore[SampleData] = InMemoryStore()
        for i in range(self.N):
            store.save(SampleData(f"v{i}", float(i)))
        return store

    def _make_populated_collection(self) -> "TimestampedCollection[SampleData]":
        from dimos.types.timestamped import TimestampedCollection

        coll: TimestampedCollection[SampleData] = TimestampedCollection()
        for i in range(self.N):
            coll.add(SampleData(f"v{i}", float(i)))
        return coll

    def test_insert_performance(self) -> None:
        """Insert N items. InMemoryStore should be within 3x of TimestampedCollection."""
        import gc
        import time as time_mod

        from dimos.types.timestamped import TimestampedCollection

        store: InMemoryStore[SampleData] = InMemoryStore()
        gc.collect()
        gc.disable()
        t0 = time_mod.perf_counter()
        for i in range(self.N):
            store.save(SampleData(f"v{i}", float(i)))
        store_time = time_mod.perf_counter() - t0
        gc.enable()

        coll: TimestampedCollection[SampleData] = TimestampedCollection()
        gc.collect()
        gc.disable()
        t0 = time_mod.perf_counter()
        for i in range(self.N):
            coll.add(SampleData(f"v{i}", float(i)))
        coll_time = time_mod.perf_counter() - t0
        gc.enable()

        print(f"\nInsert {self.N}: store={store_time:.3f}s, collection={coll_time:.3f}s")
        assert store_time < coll_time * 3

    def test_find_closest_performance(self) -> None:
        """find_closest on N items. Both should be O(log n)."""
        import gc
        import random
        import time as time_mod

        store = self._make_populated_store()
        coll = self._make_populated_collection()

        queries = [random.uniform(0, self.N) for _ in range(10_000)]

        gc.collect()
        gc.disable()
        t0 = time_mod.perf_counter()
        for q in queries:
            store.find_closest(q)
        store_time = time_mod.perf_counter() - t0

        t0 = time_mod.perf_counter()
        for q in queries:
            coll.find_closest(q)
        coll_time = time_mod.perf_counter() - t0
        gc.enable()

        print(
            f"\nfind_closest 10k on {self.N}: store={store_time:.3f}s, collection={coll_time:.3f}s"
        )
        assert store_time < coll_time * 3

    def test_interleaved_write_read(self) -> None:
        """Alternating write + find_closest. Old InMemoryStore was O(n log n) per read."""
        import gc
        import time as time_mod

        store: InMemoryStore[SampleData] = InMemoryStore()

        gc.collect()
        gc.disable()
        t0 = time_mod.perf_counter()
        for i in range(self.N):
            store.save(SampleData(f"v{i}", float(i)))
            if i % 10 == 0:
                store.find_closest(float(i) / 2)
        elapsed = time_mod.perf_counter() - t0
        gc.enable()

        print(f"\nInterleaved write+read {self.N}: {elapsed:.3f}s")
        assert elapsed < 10.0

    def test_iteration_performance(self) -> None:
        """Full iteration over N items."""
        import gc
        import time as time_mod

        store = self._make_populated_store()
        coll = self._make_populated_collection()

        gc.collect()
        gc.disable()
        t0 = time_mod.perf_counter()
        count_store = sum(1 for _ in store)
        store_time = time_mod.perf_counter() - t0

        t0 = time_mod.perf_counter()
        count_coll = sum(1 for _ in coll)
        coll_time = time_mod.perf_counter() - t0
        gc.enable()

        assert count_store == count_coll == self.N
        print(f"\nIterate {self.N}: store={store_time:.3f}s, collection={coll_time:.3f}s")
        assert store_time < coll_time * 3
