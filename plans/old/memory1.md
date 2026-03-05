# DB → Session → Store: DimOS Memory2

## Context

PR #1080 introduced `TimeSeriesStore[T]` with pluggable backends. Paul's review identified it mixes DB lifecycle, connection, and query concerns. Additionally, `memory.md` describes a system where all sensor data is stored as temporal streams with 4D spatial-temporal indexing, cross-stream correlation is the primary operation, and search (text/embedding) must work across streams. This plan builds a clean 3-layer architecture from scratch in `dimos/memory2/`, SQLite-first, with R\*Tree indexing for spatial-temporal queries.

## Architecture

```
SqliteDB (config + factory + WAL + sqlite-vec + R*Tree)
  └─ Session (connection, thread-bound)
       ├─ .timeseries(table, type) → TimeSeries[T]    (temporal store + optional 4D spatial index)
       ├─ .embeddings(table, dim)     → EmbeddingStore       (KNN search store + optional spatial index)
       ├─ .at(t, *stores)          → tuple             (multi-stream temporal lookup)
       ├─ .between(t1, t2, *stores)→ Iterator[tuple]   (batch temporal join)
       └─ .execute(sql, params)    → rows              (raw SQL escape hatch)
```

Every stream gets an R\*Tree (4D: time + xyz). Spatial info is optional per-row — rows without spatial data are indexed by time only (x/y/z set to NaN sentinels or excluded). This eliminates the need for cross-stream pose joins: each datapoint carries its own spatial context at write time.

## API Examples

```python
db = SqliteDB("run_001.db")

with db.session() as s:
    images = s.timeseries("color_images", Image)
    poses  = s.timeseries("poses", PoseStamped)
    lidar  = s.timeseries("lidar", PointCloud)
    img_emb = s.embeddings("image_embeddings", dim=512)

    # --- Save with optional spatial context ---
    images.save(frame)                          # temporal only
    images.save(frame, pose=robot_pose)         # temporal + spatial (baked in)

    # --- Temporal queries (chainable) ---
    hit = images.at(now).one()                  # closest to now → Hit | None
    hit = images.at(now, tolerance=0.1).one()   # within 100ms or None
    hit = images.before(now).one()              # last item before now
    hit = images.last()                         # most recent (shortcut)

    # Lazy fetch actual data from Hit
    image = images.load(hit.ts)                 # → Image

    # --- Spatial queries (R*Tree, chainable) ---
    hits = images.near(Point(1, 2, 3), radius=0.5).fetch()
    hits = images.near(robot_pose, radius=2.0).between(t1, t2).fetch()

    # Each hit has pose (full 6DOF) for reconstruction
    for hit in hits:
        print(f"Seen at {hit.pose}, dist={hit.spatial_distance}m")

    # --- Embedding search (chainable) ---
    query_vec = clip.encode_text("a shoe")

    # Embedding only
    hits = img_emb.search(query_vec, k=20).fetch()

    # Embedding + spatial
    hits = img_emb.search(query_vec, k=10).near(robot_pose, radius=3.0).fetch()

    # Embedding + temporal
    hits = img_emb.search(query_vec, k=10).between(t1, t2).fetch()

    # All three: embedding + spatial + temporal
    hits = (img_emb.search(query_vec, k=10)
                .near(robot_pose, radius=5.0)
                .between(now - 3600, now)
                .fetch())

    for hit in hits:
        hit.ts                                   # when
        hit.pose                                 # where + orientation (6DOF)
        hit.embedding_distance                   # similarity score
        hit.spatial_distance                     # meters from query point
        image = images.at(hit.ts).one()          # correlate to image stream
        vec = img_emb.load_embedding(hit.id)     # lazy fetch embedding

    # --- Cross-stream temporal lookup ---
    pose_hit = poses.at(hit.ts).one()

    # --- Raw SQL escape hatch ---
    rows = s.execute("SELECT ... FROM ... JOIN ...", params)
```

## File Structure

```
dimos/memory2/
    __init__.py          # public exports
    _sql.py              # _validate_identifier(), shared SQL helpers
    db.py                # DB ABC + SqliteDB
    session.py           # Session ABC + SqliteSession
    hit.py               # Hit hierarchy (7 classes: Hit, Temporal, Spatial, Embedding, combos)
    query.py             # Query hierarchy (7 classes: matching Hit types, chainable)
    timeseries.py        # TimeSeries[T] ABC + SqliteTimeSeries
    embeddings.py        # EmbeddingStore ABC + SqliteEmbeddingStore
    test_memory2.py      # tests
```

## Interfaces

### DB (`db.py`)

```python
class DB(Resource, ABC):
    def session(self) -> Session: ...
    def close(self) -> None: ...       # closes all tracked sessions
    # Resource protocol
    def start(self) -> None: pass      # usable after __init__
    def stop(self) -> None: self.close()
```

`SqliteDB`:
- Stores file path, creates parent dirs on first connect
- `_connect()`: `sqlite3.connect()`, enables WAL mode, loads sqlite-vec
- Tracks sessions via `WeakSet` for cleanup
- `:memory:` uses `file::memory:?cache=shared` URI so sessions share data

### Session (`session.py`)

```python
class Session(ABC):
    def timeseries(self, table: str, type: type[T]) -> TimeSeries[T]: ...
    def embeddings(self, table: str, dim: int) -> EmbeddingStore: ...
    def execute(self, sql: str, params=()) -> list: ...
    def close(self) -> None: ...
    def __enter__ / __exit__           # context manager
```

`SqliteSession`:
- Holds one `sqlite3.Connection`
- `timeseries()` / `embeddings()` validate table name, create store, cache it
- `execute()`: raw SQL passthrough
- Cross-stream correlation done via Query builder (e.g. `poses.at(hit.ts).one()`)

### TimeSeries (`timeseries.py`)

```python
from dimos.msgs.geometry_msgs.Pose import Pose, PoseLike
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Point import Point

# --- Hit type hierarchy (type-state, 7 classes) ---

@dataclass
class Hit:
    """Base result: just ts + optional pose. All data lazy-fetched."""
    ts: float
    pose: Pose | None = None

@dataclass
class TemporalHit(Hit):
    temporal_distance: float = 0.0       # |query_time - ts|

@dataclass
class SpatialHit(Hit):
    spatial_distance: float = 0.0        # meters from query point
    pose: Pose = field(default=...)      # guaranteed present for spatial hits

@dataclass
class EmbeddingHit(Hit):
    embedding_distance: float = 0.0      # cosine/L2 in embedding space
    id: str = ""
    metadata: dict | None = None

# Combinations (multiple inheritance)
@dataclass
class TemporalSpatialHit(TemporalHit, SpatialHit): ...

@dataclass
class TemporalEmbeddingHit(TemporalHit, EmbeddingHit): ...

@dataclass
class SpatialEmbeddingHit(SpatialHit, EmbeddingHit): ...

@dataclass
class FullHit(TemporalHit, SpatialHit, EmbeddingHit): ...

# --- Query type-state hierarchy (7 classes, narrows on chain) ---

class Query:
    """Base query builder. Accumulates filters, executes on .fetch()."""
    def fetch(self, limit: int | None = None) -> list[Hit]: ...
    def one(self) -> Hit | None: ...
    def count(self) -> int: ...

class TemporalQuery(Query):
    def near(self, point: Point | PoseLike | PoseStamped,
             radius: float) -> TemporalSpatialQuery: ...
    def fetch(self, limit=None) -> list[TemporalHit]: ...
    def one(self) -> TemporalHit | None: ...

class SpatialQuery(Query):
    def at(self, t: float, tolerance: float | None = None) -> TemporalSpatialQuery: ...
    def before(self, t: float) -> TemporalSpatialQuery: ...
    def after(self, t: float) -> TemporalSpatialQuery: ...
    def between(self, t1: float, t2: float) -> TemporalSpatialQuery: ...
    def fetch(self, limit=None) -> list[SpatialHit]: ...
    def one(self) -> SpatialHit | None: ...

class EmbeddingQuery(Query):
    def near(self, ...) -> SpatialEmbeddingQuery: ...
    def at(self, ...) -> TemporalEmbeddingQuery: ...
    def between(self, ...) -> TemporalEmbeddingQuery: ...
    def fetch(self, limit=None) -> list[EmbeddingHit]: ...

class TemporalSpatialQuery(Query):
    def fetch(self, limit=None) -> list[TemporalSpatialHit]: ...

class TemporalEmbeddingQuery(Query):
    def near(self, ...) -> FullQuery: ...
    def fetch(self, limit=None) -> list[TemporalEmbeddingHit]: ...

class SpatialEmbeddingQuery(Query):
    def at(self, ...) -> FullQuery: ...
    def between(self, ...) -> FullQuery: ...
    def fetch(self, limit=None) -> list[SpatialEmbeddingHit]: ...

class FullQuery(Query):
    def fetch(self, limit=None) -> list[FullHit]: ...

# All query logic (SQL generation) lives in base Query.
# Subclasses only override type signatures — no duplicated logic.

# --- TimeSeries ---

class TimeSeries(Generic[T], ABC):
    # Write
    def save(self, *items: T, pose: PoseLike | PoseStamped | None = None) -> None: ...

    # Start a query chain (returns typed Query)
    def at(self, t: float, tolerance: float | None = None) -> TemporalQuery: ...
    def before(self, t: float) -> TemporalQuery: ...
    def after(self, t: float) -> TemporalQuery: ...
    def between(self, t1: float, t2: float) -> TemporalQuery: ...
    def near(self, point: Point | PoseLike | PoseStamped,
             radius: float) -> SpatialQuery: ...

    # Convenience terminals (no chain needed)
    def last(self) -> TemporalHit | None: ...
    def first(self) -> TemporalHit | None: ...

    # Lazy data fetch (from Hit.ts)
    def load(self, ts: float) -> T | None: ...

    def delete(self, t: float) -> bool: ...
    def count(self) -> int: ...
```

All spatial parameters accept DimOS types with `.x`, `.y`, `.z` — `Point`, `Pose`, `PoseStamped`, `PoseLike`. Full pose (with orientation) stored per row for post-filter reconstruction.

`SqliteTimeSeries`:
- Data table: `CREATE TABLE {table} (rowid INTEGER PRIMARY KEY, timestamp REAL NOT NULL, data BLOB NOT NULL)`
- R\*Tree: `CREATE VIRTUAL TABLE {table}_rtree USING rtree(id, min_t, max_t, min_x, max_x, min_y, max_y, min_z, max_z)`
- R\*Tree `id` matches `rowid` in data table
- `save(item, pose=p)`: inserts data row + R\*Tree entry with `(ts, ts, x, x, y, y, z, z)` (point)
- `save(item)` without pose: inserts data row + R\*Tree entry with time only (x/y/z set to ±inf to match any spatial query)
- `at()`: `SELECT data FROM {table} ORDER BY ABS(timestamp - ?) LIMIT 1`
- `between()`: `SELECT data FROM {table} WHERE timestamp >= ? AND timestamp < ? ORDER BY timestamp`
- `near()`: `SELECT d.data FROM {table} d JOIN {table}_rtree r ON d.rowid = r.id WHERE r.min_t >= ? AND r.max_t <= ? AND r.min_x >= ? AND r.max_x <= ? ...`
- Lazy table creation on first operation

### EmbeddingStore (`embeddings.py`)

```python
class EmbeddingStore(ABC):
    def save(self, id: str, vector: np.ndarray, timestamp: float,
             pose: PoseLike | PoseStamped | None = None,
             metadata: dict | None = None) -> None: ...

    # Start a query chain with embedding search (returns typed Query)
    def search(self, query: np.ndarray, k: int = 10) -> EmbeddingQuery: ...

    # Chain: .search(vec, 10).near(p, 3.0).between(t1, t2).fetch() → list[FullHit]

    # Lazy fetch
    def load_embedding(self, id: str) -> np.ndarray | None: ...

    def delete(self, id: str) -> bool: ...
    def count(self) -> int: ...
```

Uses the same `Query` builder and `Hit` result type as TimeSeries. `.search()` returns a Query with embedding filter set; chain `.near()`, `.between()`, etc. to add spatial/temporal constraints.

`SqliteEmbeddingStore`:
- Three tables: `{table}_vec` (sqlite-vec virtual, `float[dim]`), `{table}_meta` (rowid, id, timestamp, x, y, z, metadata JSON), `{table}_rtree` (R\*Tree for spatial-temporal filtering)
- `search()`: KNN via `{table}_vec MATCH ?`, joined with meta for time/spatial filters
- `near=` param: pre-filters candidates via R\*Tree before KNN
- Each `SearchHit` carries position (x, y, z) directly — no pose join needed

## SQLite Details

- **WAL mode**: enabled on first connection per DB file. Allows concurrent readers + one writer across threads.
- **R\*Tree**: built into SQLite (compile-time option, enabled by default). Every stream gets a 4D R\*Tree (time + xyz). No extra extension needed.
- **sqlite-vec**: loaded via `conn.load_extension()`. Required for EmbeddingStore. TimeSeries works without it.
- **Thread safety**: each session = one connection = one thread. No `check_same_thread=False`.
- **Pickle BLOBs**: same serialization as current SqliteTSStore. Works with any `Timestamped` subclass.
- **Spatial data without pose**: rows saved without `pose=` get R\*Tree entry with x/y/z bounds set to ±1e38 (effectively unbounded), so they match any spatial query but don't constrain results.

## Implementation Order

1. `_sql.py` — `_validate_identifier()`
2. `hit.py` — `Hit` dataclass (unified result type)
3. `query.py` — `Query` builder (accumulates filters, generates SQL, returns `list[Hit]`)
4. `timeseries.py` — `TimeSeries[T]` ABC + `SqliteTimeSeries` (chain methods return Query)
5. `embeddings.py` — `EmbeddingStore` ABC + `SqliteEmbeddingStore` (.search() returns Query)
6. `session.py` — `Session` ABC + `SqliteSession`
7. `db.py` — `DB` ABC + `SqliteDB` (config, connect, WAL, sqlite-vec, Resource)
8. `__init__.py` — public exports
9. `test_memory2.py` — tests: lifecycle, temporal/spatial/embedding queries, combined chains, lazy fetch
10. `pyproject.toml` — add `sqlite-vec` dependency

## Verification

1. `uv run pytest dimos/memory2/test_memory2.py -v` — all new tests pass
2. `uv run mypy dimos/memory2/` — type checks clean
3. Existing `dimos/memory/timeseries/test_base.py` still passes (untouched)
