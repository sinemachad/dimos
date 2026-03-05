# DimOS Memory2 Spec v2.1

Status: implementation-oriented draft for a coding agent.

This spec is intentionally code/example focused. It defines the public API shape, core invariants, and the minimum execution model needed to implement a useful local-first multimodal memory system.

---

# 0. Goals

Memory2 stores and queries multimodal robot observations.

Primary use cases:

1. Store raw streams: images, lidar, poses, logs, narration.
2. Generate streams from streams: embeddings from images, captions from images, detections from frames.
3. Narrow data without loading payloads: top-k matches, time windows, spatial subsets.
4. Re-query narrowed results.
5. Correlate across streams.
6. Keep payload loading lazy.

Non-goal:

- Do not implement high-level search policies here (hotspot search, VLM orchestration, semantic map UI).

---

# 1. Terminology

## 1.1 Observation

A single stored item.

Examples:

- one RGB frame
- one lidar scan
- one log line
- one CLIP embedding
- one VLM caption

## 1.2 Stream

Appendable collection of observations with a shared payload type and capability set.

Examples:

- `rgb_front`
- `lidar_mid360`
- `robot_pose`
- `tool_logs`
- `image_embeddings_clip`

## 1.3 ObservationSet

A lazy, read-only, queryable view over observations.

Important:

- an `ObservationSet` is **not** a Python set
- it is usually **lazy**
- it usually contains **refs + metadata**, not payloads
- it may represent a subset of one stream or a projection/correlation over multiple streams

## 1.4 DerivedStream

A stream generated from upstream streams or observation sets.

Examples:

- embeddings generated from images
- captions generated from images
- detections generated from frames

Rule:

- same observation identity -> `ObservationSet`
- new observation identity -> `DerivedStream`

---

# 2. Core invariants

These are hard requirements.

## 2.1 Stable identity

Every observation has a stable reference independent of timestamp.

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class ObservationRef:
    stream: str
    id: str
```

Never use timestamp as the primary load key.

Bad:

```python
images.load(hit.ts)
```

Good:

```python
images.load(hit.ref)
```

## 2.2 Payloads are lazy

Queries and observation sets must not load full payloads unless explicitly requested.

Examples of payloads that must stay lazy:

- images
- point clouds
- audio chunks
- voxel blocks

## 2.3 Metadata may be materialized

It is acceptable to materialize lightweight metadata for result sets:

- ref
- timestamp
- pose
- scores
- tags
- lineage pointers

## 2.4 Query results are re-queryable

A narrowed result should still support `.query()` and further filtering/ranking.

## 2.5 Query results are not appendable

`ObservationSet` is read-only.

Only `Stream` is appendable.

## 2.6 Spatially unknown != spatially everywhere

Unlocalized observations do not match spatial queries by default.

## 2.7 Derived outputs must carry lineage

Any derived stream should record parent streams and parent refs or parent query provenance.

---

# 3. Public API

## 3.1 Top-level objects

```python
class DB: ...
class Session: ...
class Stream[T]: ...
class ObservationSet[T]: ...
class Query[T]: ...
class Correlator: ...
```

## 3.2 Shared read/query protocol

`Stream` and `ObservationSet` should share the same read/query protocol.

```python
from typing import Protocol, Iterable, Iterator, Generic, TypeVar, Any

T = TypeVar("T")

class QueryableObservationSet(Protocol, Generic[T]):
    def query(self) -> "Query[T]": ...
    def load(self, ref: ObservationRef) -> T: ...
    def load_many(self, refs: list[ObservationRef], *, batch_size: int = 32) -> list[T]: ...
    def iter_meta(self, *, page_size: int = 128) -> Iterator[list["ObservationRow"]]: ...
    def count(self) -> int: ...
    def capabilities(self) -> set[str]: ...
```

`Stream` extends this with append/introspection.

---

# 4. Core data structures

## 4.1 Observation metadata

```python
from dataclasses import dataclass, field
from typing import Any

@dataclass
class Pose:
    xyz: tuple[float, float, float]
    quat_xyzw: tuple[float, float, float, float] | None = None

@dataclass
class ObservationMeta:
    ref: ObservationRef
    ts_start: float | None = None
    ts_end: float | None = None
    robot_id: str | None = None
    frame_id: str | None = None
    pose: Pose | None = None
    pose_source: str | None = None
    pose_confidence: float | None = None
    transform_version: str | None = None
    timestamp_uncertainty: float | None = None
    payload_codec: str | None = None
    payload_size_bytes: int | None = None
    tags: dict[str, Any] = field(default_factory=dict)
```

Notes:

- point observations use `ts_start == ts_end`
- interval observations use `[ts_start, ts_end]`
- `pose` is a denormalized snapshot for fast filtering
- provenance fields allow later reinterpretation after better localization

## 4.2 Query/ObservationSet row

An `ObservationSet` should expose rows with lightweight metadata and scores.

```python
@dataclass
class ObservationRow:
    ref: ObservationRef
    ts_start: float | None = None
    ts_end: float | None = None
    pose: Pose | None = None
    scores: dict[str, float] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)
```

Expected score keys:

- `embedding_distance`
- `text_rank`
- `spatial_distance`
- `temporal_distance`
- `final_rank`

## 4.3 Lineage

```python
@dataclass
class Lineage:
    parents: list[str] = field(default_factory=list)
    parent_refs: list[ObservationRef] = field(default_factory=list)
    query_repr: str | None = None
    transform_name: str | None = None
    transform_version: str | None = None
```

This can be attached to streams, rows, or derived outputs.

---

# 5. Stream API

## 5.1 Stream creation

```python
with db.session() as s:
    images = s.stream(
        name="rgb_front",
        payload_type=Image,
        capabilities={"temporal", "spatial", "load"},
        retention="run",
    )

    logs = s.stream(
        name="tool_logs",
        payload_type=str,
        capabilities={"temporal", "text", "load"},
        retention="run",
    )

    image_embeddings = s.stream(
        name="image_embeddings_clip",
        payload_type=Embedding,
        capabilities={"temporal", "spatial", "embedding", "load"},
        retention="derived",
        config={"dim": 512, "metric": "cosine"},
    )
```

## 5.2 Stream interface

```python
class Stream(QueryableObservationSet[T], Generic[T]):
    def append(self, payload: T, **meta: Any) -> ObservationRef: ...
    def append_many(self, payloads: Iterable[T], metas: Iterable[dict[str, Any]]) -> list[ObservationRef]: ...
    def meta(self, ref: ObservationRef) -> ObservationMeta: ...
    def info(self) -> dict[str, Any]: ...
    def stats(self) -> dict[str, Any]: ...
    def retention(self) -> str: ...
```

## 5.3 Append examples

```python
frame_ref = images.append(
    frame,
    ts_start=now,
    ts_end=now,
    robot_id="go2_01",
    frame_id="map",
    pose=current_pose,
    pose_source="slam_localization",
    transform_version="loc_epoch_17",
)

log_ref = logs.append(
    "planner timeout on task 42",
    ts_start=now,
    ts_end=now,
    tags={"level": "warning", "module": "planner"},
)
```

---

# 6. ObservationSet API

## 6.1 Design intent

`ObservationSet` is the key abstraction for narrowed/re-queryable results.

It should:

- be lazy by default
- usually avoid payload loading
- support `.query()`
- support loading payloads one-by-one or in batches
- support projection to related streams
- support materialization when needed

## 6.2 Interface

```python
class ObservationSet(QueryableObservationSet[T], Generic[T]):
    def refs(self, *, limit: int | None = None) -> list[ObservationRef]: ...
    def rows(self, *, limit: int | None = None) -> list[ObservationRow]: ...
    def one(self) -> ObservationRow: ...
    def fetch_page(self, *, limit: int = 128, offset: int = 0) -> list[ObservationRow]: ...
    def project_to(self, stream: "Stream[Any]") -> "ObservationSet[Any]": ...
    def materialize(self, *, name: str | None = None, retention: str = "ephemeral") -> "ObservationSet[T]": ...
    def derive(self, *, name: str, transform: "Transform[T, Any]", retention: str = "derived", payload_type: type | None = None) -> "Stream[Any]": ...
    def lineage(self) -> Lineage: ...
```

## 6.3 Example: narrowing data and re-querying

```python
recent_images = (
    images.query()
      .filter_time(now - 600, now)
      .fetch_set()
)

recent_nearby_images = (
    recent_images.query()
      .filter_near(current_pose, radius=3.0)
      .fetch_set()
)
```

## 6.4 Example: embedding search without loading images

```python
matches = (
    image_embeddings.query()
      .search_embedding(query_vec, candidate_k=2000)
      .filter_time(now - 3600, now)
      .filter_near(current_pose, radius=8.0)
      .rank(embedding=1.0, recency=0.2, distance=0.3)
      .limit(1000)
      .fetch_set()
)
```

Important:

- `matches` should not contain 1000 image payloads in RAM
- it should usually contain refs + lightweight metadata/scores only

## 6.5 Example: payload access stays explicit

```python
rows = matches.fetch_page(limit=20, offset=0)
first_payload = image_embeddings.load(rows[0].ref)

candidate_refs = matches.refs(limit=16)
embeddings_batch = image_embeddings.load_many(candidate_refs, batch_size=16)
```

## 6.6 Example: projecting embedding matches to images

Assume each embedding row records its parent image ref.

```python
matched_frames = matches.project_to(images)
preview_rows = matched_frames.fetch_page(limit=12)
preview_frames = images.load_many([r.ref for r in preview_rows], batch_size=12)
```

## 6.7 Example: deriving a caption stream from a narrowed image set

```python
captions = matched_frames.derive(
    name="vlm_captions_shoe_candidates",
    transform=caption_model,
    retention="derived",
    payload_type=str,
)
```

This creates a new stream because it creates new observation identities.

---

# 7. Query API

## 7.1 Query design

Query should be composable and capability-based.

It should support:

- hard filters
- candidate generation
- soft ranking
- terminal materialization

## 7.2 Interface

```python
class Query(Generic[T]):
    def filter_time(self, t1: float, t2: float) -> "Query[T]": ...
    def filter_before(self, t: float) -> "Query[T]": ...
    def filter_after(self, t: float) -> "Query[T]": ...
    def filter_near(self, pose: Pose, radius: float, *, include_unlocalized: bool = False) -> "Query[T]": ...
    def filter_tags(self, **tags: Any) -> "Query[T]": ...
    def filter_refs(self, refs: list[ObservationRef]) -> "Query[T]": ...

    def search_text(self, text: str, *, candidate_k: int | None = None) -> "Query[T]": ...
    def search_embedding(self, vector: list[float], *, candidate_k: int) -> "Query[T]": ...

    def rank(self, **weights: float) -> "Query[T]": ...
    def limit(self, k: int) -> "Query[T]": ...

    def fetch(self) -> list[ObservationRow]: ...
    def fetch_set(self) -> ObservationSet[T]: ...
    def count(self) -> int: ...
    def one(self) -> ObservationRow: ...
```

## 7.3 Hard filters vs ranking

This distinction must stay explicit.

Example:

```python
hits = (
    image_embeddings.query()
      .search_embedding(query_vec, candidate_k=1000)
      .filter_time(t1, t2)
      .filter_near(current_pose, radius=5.0)
      .rank(embedding=1.0, recency=0.15, distance=0.35)
      .limit(50)
      .fetch()
)
```

Execution meaning:

1. embedding search creates candidates
2. time/space filters remove candidates
3. ranking combines scores on remaining rows
4. limit applies at the end

Do not leave this ambiguous.

---

# 8. Under-the-hood model for ObservationSet

## 8.1 Default behavior

`ObservationSet` should be lazy/unresolved until needed.

It must not eagerly decode payloads.

## 8.2 Internal backing kinds

Publicly there is one `ObservationSet` class. Internally it may have multiple backing strategies.

```python
from dataclasses import dataclass
from typing import Literal

@dataclass
class PredicateBacking:
    source_name: str
    query_repr: str

@dataclass
class RefTableBacking:
    table_name: str
    source_streams: list[str]
    ordered: bool = False

@dataclass
class CompositeBacking:
    op: Literal["union", "intersection", "difference", "project", "join"]
    input_ids: list[str]
    query_repr: str
```

Recommended internal shape:

```python
class ObservationSet(QueryableObservationSet[T], Generic[T]):
    _backing: PredicateBacking | RefTableBacking | CompositeBacking
    _capabilities: set[str]
    _lineage: Lineage
```

## 8.3 Predicate-backed set

Use when the set is still naturally expressible as a query over the underlying source.

Examples:

- time range over one stream
- tag filter over one stream
- spatial filter over one stream
- text search over one stream

No payloads need to be materialized.

## 8.4 Ref-table-backed set

Use when a query creates an explicit candidate pool.

Examples:

- top-k embedding matches
- correlation results
- reranked subsets
- cluster representatives

Important:

- refs do not need to live in Python memory
- they can live in a SQLite temp table
- metadata rows may be fetched page-wise

## 8.5 Composite-backed set

Use for union/intersection/project/join style operations over other observation sets.

---

# 9. Payload loading rules

## 9.1 Allowed eager data

Eagerly loaded into Python is acceptable for:

- small metadata rows
- refs
- scores
- tags

## 9.2 Disallowed by default

Do not eagerly load by default:

- all image payloads
- all point clouds
- all audio blobs
- all voxel blocks

## 9.3 Required explicit methods

```python
payload = images.load(ref)
payloads = images.load_many(refs, batch_size=32)

for page in image_set.iter_meta(page_size=128):
    ...
```

No API should silently decode a thousand images just because `.fetch_set()` was called.

---

# 10. Stream generation from streams

This is a central use case.

## 10.1 Example: embeddings from images

```python
frames = (
    images.query()
      .filter_time(now - 60, now)
      .fetch_set()
)

embeddings = frames.derive(
    name="image_embeddings_clip_recent",
    transform=clip_embedder,
    retention="derived",
    payload_type=Embedding,
)
```

Implementation expectation:

- `derive()` iterates source payloads in batches
- output rows record lineage to input refs
- output stream stores its own payloads/metadata/indexes

## 10.2 Example transform protocol

```python
U = TypeVar("U")

class Transform(Protocol, Generic[T, U]):
    name: str
    version: str

    def map_batch(self, rows: list[ObservationRow], payloads: list[T]) -> list[tuple[U, dict[str, Any]]]: ...
```

This allows a coding agent to implement batch transforms cleanly.

---

# 11. Correlation API

Correlation is first-class.

## 11.1 Example

```python
bundle = s.correlate(
    anchor=log_ref,
    with_streams=[images, lidar, poses],
    by={
        "rgb_front": {"mode": "nearest_time", "tolerance": 0.2},
        "lidar_mid360": {"mode": "nearest_time", "tolerance": 0.1},
        "robot_pose": {"mode": "nearest_time", "tolerance": 0.05},
    },
)
```

## 11.2 Correlation result shape

```python
@dataclass
class CorrelatedItem:
    stream: str
    row: ObservationRow | None
    reason: dict[str, Any]

@dataclass
class CorrelationBundle:
    anchor: ObservationRef
    items: list[CorrelatedItem]
```

At minimum support:

- nearest in time
- overlapping interval

Later support:

- nearest in space
- same robot\_id
- same frame\_id

---

# 12. Introspection

These are needed for human tooling and debugging.

```python
s.list_streams()
images.info()
images.stats()
matches.capabilities()
matches.lineage()
```

Recommended fields for `stream.info()`:

```python
{
    "name": "rgb_front",
    "payload_type": "Image",
    "row_count": 12345,
    "retention": "run",
    "capabilities": ["temporal", "spatial", "load"],
    "time_bounds": [1700000000.0, 1700003600.0],
    "spatial_bounds": [xmin, ymin, zmin, xmax, ymax, zmax],
    "payload_codec": "jpeg",
}
```

---

# 13. Backend implementation target

SQLite-first, but backend-specific details should stay behind the API.

## 13.1 Expected SQLite tools

- normal tables for metadata
- temp tables for candidate refs
- FTS5 for text search
- R-tree for spatial indexing
- vector extension when available

## 13.2 Suggested mapping per stream

- metadata table
- payload table or blob column
- optional FTS table
- optional vector index table
- optional spatial index table

## 13.3 Important backend rule

Unlocalized rows should not be inserted into the spatial index.

---

# 14. Concrete execution examples

## 14.1 Time-filtered image subset stays lazy

```python
recent = (
    images.query()
      .filter_time(now - 300, now)
      .fetch_set()
)
```

Expected implementation:

- create predicate-backed `ObservationSet`
- do not decode image payloads
- only execute SQL when rows/count/payloads are requested

## 14.2 Embedding search becomes ref-table-backed

```python
matches = (
    image_embeddings.query()
      .search_embedding(query_vec, candidate_k=5000)
      .filter_time(now - 7200, now)
      .limit(1000)
      .fetch_set()
)
```

Expected implementation:

- run vector search
- write candidate refs + scores to temp table
- return ref-table-backed `ObservationSet`
- allow further `.query()` by restricting to that candidate table

## 14.3 Re-query candidate set without loading payloads

```python
nearby_matches = (
    matches.query()
      .filter_near(current_pose, radius=6.0)
      .limit(100)
      .fetch_set()
)
```

Expected implementation:

- join source metadata with candidate ref table
- apply spatial filter in backend
- return new lazy observation set

## 14.4 Paginated preview

```python
page = nearby_matches.fetch_page(limit=24, offset=0)
preview_refs = [row.ref for row in page]
preview_embeddings = image_embeddings.load_many(preview_refs, batch_size=24)
```

Again: explicit payload loading only.

---

# 15. What the coding agent should implement first

Implementation priority order:

1. `ObservationRef`, `ObservationMeta`, `ObservationRow`, `Lineage`
2. `DB`, `Session`, `Stream`
3. `Query` with time filters and `.fetch_set()`
4. lazy `ObservationSet` with predicate backing
5. explicit payload loading methods
6. text search
7. ref-table-backed observation sets
8. embedding search
9. `project_to()`
10. `derive()`
11. correlation
12. introspection/stats

---

# 16. Minimal acceptance examples

These examples should work.

## 16.1 Re-query narrowed data

```python
recent = images.query().filter_time(t1, t2).fetch_set()
recent2 = recent.query().filter_near(pose, radius=2.0).fetch_set()
assert recent2.count() <= recent.count()
```

## 16.2 Fetch set does not load payloads

```python
matches = images.query().filter_time(t1, t2).limit(1000).fetch_set()
# should be cheap even for large image payloads
rows = matches.fetch_page(limit=10)
assert len(rows) == 10
```

## 16.3 Derived stream from narrowed set

```python
subset = images.query().filter_time(t1, t2).limit(100).fetch_set()
captions = subset.derive(
    name="captions_test",
    transform=caption_model,
    retention="derived",
    payload_type=str,
)
assert captions.count() == subset.count()
```

## 16.4 Projection from embeddings to images

```python
emb_matches = image_embeddings.query().search_embedding(qvec, candidate_k=100).fetch_set()
frame_matches = emb_matches.project_to(images)
rows = frame_matches.fetch_page(limit=5)
frames = images.load_many([r.ref for r in rows], batch_size=5)
assert len(frames) == 5
```

---

# 17. Summary

Memory2 should expose:

- appendable `Stream`
- lazy read-only `ObservationSet`
- composable `Query`
- explicit payload loading
- derived stream generation
- re-queryable narrowed results
- stable observation refs
- backend-backed candidate sets instead of eager payload lists

The most important implementation rule is this:

> `fetch_set()` returns a lazy queryable view over observations, not a Python list of decoded payloads.
