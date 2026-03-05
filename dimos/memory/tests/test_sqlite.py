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

"""Tests for SQLite-backed memory store."""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest

from dimos.memory.impl.sqlite import SqliteSession, SqliteStore
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.testing import TimedSensorReplay

if TYPE_CHECKING:
    from dimos.memory.types import Observation


@pytest.fixture(scope="module")
def replay() -> TimedSensorReplay:  # type: ignore[type-arg]
    return TimedSensorReplay("unitree_go2_bigoffice/video")


@pytest.fixture(scope="module")
def images(replay: TimedSensorReplay) -> list[Image]:  # type: ignore[type-arg]
    """Load 5 images from replay at 1s intervals."""
    imgs = [replay.find_closest_seek(float(i)) for i in range(1, 6)]
    assert all(isinstance(im, Image) for im in imgs)
    return imgs  # type: ignore[return-value]


@pytest.fixture
def store(tmp_path: object) -> SqliteStore:
    from pathlib import Path

    assert isinstance(tmp_path, Path)
    return SqliteStore(str(tmp_path / "test.db"))


@pytest.fixture
def session(store: SqliteStore) -> SqliteSession:
    return store.session()


class TestStreamBasics:
    def test_create_stream(self, session: SqliteSession) -> None:
        s = session.stream("images", Image)
        assert s is not None

    def test_append_and_fetch(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("images", Image)
        obs = s.append(images[0])
        assert obs.id == 1
        assert obs.data == images[0]
        assert obs.ts is not None

        rows = s.fetch()
        assert len(rows) == 1
        assert rows[0].data == images[0]
        assert rows[0].id == 1

    def test_append_multiple(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("images", Image)
        for img in images[:3]:
            s.append(img)

        assert s.count() == 3
        rows = s.fetch()
        assert [r.data for r in rows] == images[:3]

    def test_append_with_tags(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("images", Image)
        s.append(images[0], tags={"cam": "front", "quality": "high"})

        rows = s.fetch()
        assert rows[0].tags == {"cam": "front", "quality": "high"}

    def test_last(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("images", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=2.0)
        s.append(images[2], ts=3.0)

        obs = s.last()
        assert obs.data == images[2]
        assert obs.ts == 3.0

    def test_one(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("images", Image)
        s.append(images[0])

        obs = s.one()
        assert obs.data == images[0]

    def test_one_empty_raises(self, session: SqliteSession) -> None:
        s = session.stream("images", Image)
        with pytest.raises(LookupError):
            s.one()


class TestFilters:
    def test_after(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=10.0)

        rows = s.after(5.0).fetch()
        assert len(rows) == 1
        assert rows[0].data == images[1]

    def test_before(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=10.0)

        rows = s.before(5.0).fetch()
        assert len(rows) == 1
        assert rows[0].data == images[0]

    def test_time_range(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=5.0)
        s.append(images[2], ts=10.0)

        rows = s.time_range(3.0, 7.0).fetch()
        assert len(rows) == 1
        assert rows[0].data == images[1]

    def test_at(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=5.0)
        s.append(images[2], ts=10.0)

        rows = s.at(5.5, tolerance=1.0).fetch()
        assert len(rows) == 1
        assert rows[0].data == images[1]

    def test_filter_tags(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], tags={"cam": "front"})
        s.append(images[1], tags={"cam": "rear"})

        rows = s.filter_tags(cam="front").fetch()
        assert len(rows) == 1
        assert rows[0].data == images[0]

    def test_chained_filters(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0, tags={"cam": "front"})
        s.append(images[1], ts=5.0, tags={"cam": "front"})
        s.append(images[2], ts=5.0, tags={"cam": "rear"})

        rows = s.after(3.0).filter_tags(cam="front").fetch()
        assert len(rows) == 1
        assert rows[0].data == images[1]


class TestOrdering:
    def test_order_by_ts(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[1], ts=2.0)
        s.append(images[0], ts=1.0)
        s.append(images[2], ts=3.0)

        rows = s.order_by("ts").fetch()
        assert [r.data for r in rows] == [images[0], images[1], images[2]]

    def test_order_by_desc(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=2.0)
        s.append(images[2], ts=3.0)

        rows = s.order_by("ts", desc=True).fetch()
        assert [r.data for r in rows] == [images[2], images[1], images[0]]

    def test_limit_offset(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        for i, img in enumerate(images):
            s.append(img, ts=float(i))

        rows = s.order_by("ts").limit(2).offset(1).fetch()
        assert len(rows) == 2
        assert [r.data for r in rows] == [images[1], images[2]]


class TestFetchPages:
    def test_basic_pagination(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        for i, img in enumerate(images):
            s.append(img, ts=float(i))

        pages = list(s.fetch_pages(batch_size=2))
        assert len(pages) == 3  # 2+2+1
        assert len(pages[0]) == 2
        assert len(pages[-1]) == 1

        all_items = [obs.data for page in pages for obs in page]
        assert all_items == images


class TestTextStream:
    def test_create_and_append(self, session: SqliteSession) -> None:
        s = session.text_stream("logs", str)
        s.append("Motor fault on joint 3")
        s.append("Battery low warning")

        assert s.count() == 2

    def test_text_search(self, session: SqliteSession) -> None:
        s = session.text_stream("logs", str)
        s.append("Motor fault on joint 3")
        s.append("Battery low warning")
        s.append("Motor overheating on joint 5")

        rows = s.search_text("motor", k=10).fetch()
        assert len(rows) == 2
        assert all("Motor" in r.data for r in rows)


class TestListStreams:
    def test_list_empty(self, session: SqliteSession) -> None:
        assert session.list_streams() == []

    def test_list_after_create(self, session: SqliteSession) -> None:
        session.stream("images", Image)
        session.text_stream("logs", str)

        infos = session.list_streams()
        names = {i.name for i in infos}
        assert names == {"images", "logs"}


class TestReactive:
    def test_appended_observable(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("images", Image)
        received: list[Observation] = []
        s.appended.subscribe(on_next=received.append)

        s.append(images[0])
        s.append(images[1])

        assert len(received) == 2
        assert received[0].data == images[0]
        assert received[1].data == images[1]


class TestTransformInMemory:
    def test_lambda_transform(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=2.0)

        shapes = s.transform(lambda im: f"{im.width}x{im.height}")
        results = shapes.fetch()
        assert len(results) == 2
        assert results[0].data == f"{images[0].width}x{images[0].height}"

    def test_lambda_filter_none(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=2.0)
        s.append(images[2], ts=3.0)

        # Only keep images wider than 0 (all pass), filter second by index trick
        idx = iter(range(3))
        big = s.transform(lambda im: im if next(idx) % 2 == 0 else None)
        results = big.fetch()
        assert len(results) == 2  # indices 0 and 2

    def test_lambda_expand_list(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)

        # Extract format and frame_id as two separate results
        results = s.transform(lambda im: [im.format.value, im.frame_id]).fetch()
        assert len(results) == 2
        assert results[0].data == images[0].format.value


class TestTransformStore:
    def test_transform_store_backfill(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)
        s.append(images[1], ts=2.0)

        stored = s.transform(lambda im: f"{im.width}x{im.height}").store("shapes")
        rows = stored.fetch()
        assert len(rows) == 2
        expected = f"{images[0].width}x{images[0].height}"
        assert rows[0].data == expected

        reloaded = session.stream("shapes")
        assert reloaded.count() == 2

    def test_transform_store_live(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)

        stored = s.transform(lambda im: im.height, live=True).store("heights")
        assert stored.count() == 0  # no backfill

        s.append(images[1], ts=2.0)
        assert stored.count() == 1
        assert stored.last().data == images[1].height

    def test_transform_store_backfill_only(
        self, session: SqliteSession, images: list[Image]
    ) -> None:
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)

        stored = s.transform(lambda im: im.height, backfill_only=True).store("heights_bo")
        assert stored.count() == 1
        assert stored.one().data == images[0].height

        s.append(images[1], ts=2.0)
        assert stored.count() == 1  # still 1


class TestLazyData:
    def test_data_lazy_loaded(self, session: SqliteSession, images: list[Image]) -> None:
        """Fetched observations should not eagerly load payload."""
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0)

        rows = s.fetch()
        obs = rows[0]
        from dimos.memory.types import _UNSET

        assert obs._data is _UNSET
        assert obs._data_loader is not None
        assert obs.data == images[0]
        assert obs._data == images[0]

    def test_metadata_without_payload(self, session: SqliteSession, images: list[Image]) -> None:
        """Metadata (ts, tags) should be available without loading payload."""
        s = session.stream("data", Image)
        s.append(images[0], ts=1.0, tags={"key": "val"})

        rows = s.fetch()
        obs = rows[0]
        assert obs.ts == 1.0
        assert obs.tags == {"key": "val"}
        assert obs.id == 1


class TestIteration:
    def test_iter(self, session: SqliteSession, images: list[Image]) -> None:
        s = session.stream("data", Image)
        for i, img in enumerate(images[:3]):
            s.append(img, ts=float(i))

        items = [obs.data for obs in s]
        assert items == images[:3]


class TestStoreReopen:
    def test_data_persists(self, tmp_path: object, images: list[Image]) -> None:
        from pathlib import Path

        assert isinstance(tmp_path, Path)
        db_path = str(tmp_path / "persist.db")

        store1 = SqliteStore(db_path)
        s1 = store1.session()
        s1.stream("data", Image).append(images[0], ts=1.0)
        s1.close()
        store1.close()

        store2 = SqliteStore(db_path)
        s2 = store2.session()
        rows = s2.stream("data", Image).fetch()
        assert len(rows) == 1
        assert rows[0].data == images[0]
        s2.close()
        store2.close()
