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

"""Grid tests for Store implementations.

Runs the same test logic against every Store backend (MemoryStore, SqliteStore, …).
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import pytest

if TYPE_CHECKING:
    from collections.abc import Callable, Generator

    from dimos.memory2.store import Session

# ── Case definition ────────────────────────────────────────────────


@dataclass
class Case:
    name: str
    session_factory: Callable[[], Generator[Session, None, None]]
    tags: set[str] = field(default_factory=set)


# ── Context managers ───────────────────────────────────────────────


@contextmanager
def memory_session() -> Generator[Session, None, None]:
    from dimos.memory2.impl.memory import MemoryStore

    store = MemoryStore()
    with store.session() as session:
        yield session


@contextmanager
def sqlite_session() -> Generator[Session, None, None]:
    import tempfile

    from dimos.memory2.impl.sqlite import SqliteStore

    with tempfile.NamedTemporaryFile(suffix=".db") as f:
        store = SqliteStore(path=f.name)
        with store.session() as session:
            yield session


# ── Test cases ─────────────────────────────────────────────────────

testcases = [
    Case(name="memory", session_factory=memory_session, tags={"basic", "live"}),
    Case(
        name="sqlite",
        session_factory=sqlite_session,
        tags={"basic"},
    ),
]

basic_cases = [c for c in testcases if "basic" in c.tags]


# ── Tests ──────────────────────────────────────────────────────────


@pytest.mark.parametrize("case", basic_cases, ids=lambda c: c.name)
class TestStoreBasic:
    """Core store operations that every backend must support."""

    def test_create_stream_and_append(self, case: Case) -> None:
        with case.session_factory() as session:
            s = session.stream("images", bytes)
            obs = s.append(b"frame1", tags={"camera": "front"})

            assert obs.data == b"frame1"
            assert obs.tags["camera"] == "front"
            assert obs.ts > 0

    def test_append_multiple_and_fetch(self, case: Case) -> None:
        with case.session_factory() as session:
            s = session.stream("sensor", float)
            s.append(1.0, ts=100.0)
            s.append(2.0, ts=200.0)
            s.append(3.0, ts=300.0)

            results = s.fetch()
            assert len(results) == 3
            assert [o.data for o in results] == [1.0, 2.0, 3.0]

    def test_iterate_stream(self, case: Case) -> None:
        with case.session_factory() as session:
            s = session.stream("log", str)
            s.append("a", ts=1.0)
            s.append("b", ts=2.0)

            collected = [obs.data for obs in s]
            assert collected == ["a", "b"]

    def test_count(self, case: Case) -> None:
        with case.session_factory() as session:
            s = session.stream("events", str)
            assert s.count() == 0
            s.append("x")
            s.append("y")
            assert s.count() == 2

    def test_first_and_last(self, case: Case) -> None:
        with case.session_factory() as session:
            s = session.stream("data", int)
            s.append(10, ts=1.0)
            s.append(20, ts=2.0)
            s.append(30, ts=3.0)

            assert s.first().data == 10
            assert s.last().data == 30

    def test_first_empty_raises(self, case: Case) -> None:
        with case.session_factory() as session:
            s = session.stream("empty", int)
            with pytest.raises(LookupError):
                s.first()

    def test_exists(self, case: Case) -> None:
        with case.session_factory() as session:
            s = session.stream("check", str)
            assert not s.exists()
            s.append("hi")
            assert s.exists()

    def test_filter_after(self, case: Case) -> None:
        with case.session_factory() as session:
            s = session.stream("ts_data", int)
            s.append(1, ts=10.0)
            s.append(2, ts=20.0)
            s.append(3, ts=30.0)

            results = s.after(15.0).fetch()
            assert [o.data for o in results] == [2, 3]

    def test_filter_before(self, case: Case) -> None:
        with case.session_factory() as session:
            s = session.stream("ts_data", int)
            s.append(1, ts=10.0)
            s.append(2, ts=20.0)
            s.append(3, ts=30.0)

            results = s.before(25.0).fetch()
            assert [o.data for o in results] == [1, 2]

    def test_filter_time_range(self, case: Case) -> None:
        with case.session_factory() as session:
            s = session.stream("ts_data", int)
            s.append(1, ts=10.0)
            s.append(2, ts=20.0)
            s.append(3, ts=30.0)

            results = s.time_range(15.0, 25.0).fetch()
            assert [o.data for o in results] == [2]

    def test_filter_tags(self, case: Case) -> None:
        with case.session_factory() as session:
            s = session.stream("tagged", str)
            s.append("a", tags={"kind": "info"})
            s.append("b", tags={"kind": "error"})
            s.append("c", tags={"kind": "info"})

            results = s.filter_tags(kind="info").fetch()
            assert [o.data for o in results] == ["a", "c"]

    def test_limit_and_offset(self, case: Case) -> None:
        with case.session_factory() as session:
            s = session.stream("paged", int)
            for i in range(5):
                s.append(i, ts=float(i))

            page = s.offset(1).limit(2).fetch()
            assert [o.data for o in page] == [1, 2]

    def test_order_by_desc(self, case: Case) -> None:
        with case.session_factory() as session:
            s = session.stream("ordered", int)
            s.append(1, ts=10.0)
            s.append(2, ts=20.0)
            s.append(3, ts=30.0)

            results = s.order_by("ts", desc=True).fetch()
            assert [o.data for o in results] == [3, 2, 1]

    def test_separate_streams_isolated(self, case: Case) -> None:
        with case.session_factory() as session:
            a = session.stream("stream_a", str)
            b = session.stream("stream_b", str)

            a.append("in_a")
            b.append("in_b")

            assert [o.data for o in a] == ["in_a"]
            assert [o.data for o in b] == ["in_b"]

    def test_same_stream_on_repeated_calls(self, case: Case) -> None:
        with case.session_factory() as session:
            s1 = session.stream("reuse", str)
            s2 = session.stream("reuse", str)
            assert s1 is s2

    def test_append_with_embedding(self, case: Case) -> None:
        import numpy as np

        from dimos.memory2.type import EmbeddedObservation
        from dimos.models.embedding.base import Embedding

        with case.session_factory() as session:
            s = session.stream("vectors", str)
            emb = Embedding(vector=np.array([1.0, 0.0, 0.0], dtype=np.float32))
            obs = s.append("hello", embedding=emb)
            assert isinstance(obs, EmbeddedObservation)
            assert obs.embedding is emb

    def test_search_top_k(self, case: Case) -> None:
        import numpy as np

        from dimos.models.embedding.base import Embedding

        def _emb(v: list[float]) -> Embedding:
            a = np.array(v, dtype=np.float32)
            return Embedding(vector=a / (np.linalg.norm(a) + 1e-10))

        with case.session_factory() as session:
            s = session.stream("searchable", str)
            s.append("north", embedding=_emb([0, 1, 0]))
            s.append("east", embedding=_emb([1, 0, 0]))
            s.append("south", embedding=_emb([0, -1, 0]))

            results = s.search(_emb([0, 1, 0]), k=2).fetch()
            assert len(results) == 2
            assert results[0].data == "north"
            assert results[0].similarity > 0.99

    def test_search_text(self, case: Case) -> None:
        with case.session_factory() as session:
            s = session.stream("logs", str)
            s.append("motor fault")
            s.append("temperature ok")

            results = s.search_text("motor").fetch()
            assert len(results) == 1
            assert results[0].data == "motor fault"
