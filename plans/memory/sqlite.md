# SQLite Implementation

Implementation spec for `dimos/memory/impl/sqlite/`. A coding agent should be able to implement the full SQLite backend from this document + `api.md`.

## File Structure

```
dimos/memory/
    __init__.py              # public exports: Observation, EmbeddingObservation,
                             # Stream, EmbeddingStream, TextStream, Transformer,
                             # EmbeddingTransformer, PerItemTransformer, Session, Store
    types.py                 # Observation, EmbeddingObservation, StreamInfo
    stream.py                # Stream, EmbeddingStream, TextStream (base classes)
    transformer.py           # Transformer ABC, EmbeddingTransformer, PerItemTransformer
    store.py                 # Store ABC
    session.py               # Session ABC

    impl/
        sqlite/
            __init__.py      # exports SqliteStore
            store.py         # SqliteStore
            session.py       # SqliteSession
            stream.py        # SqliteStream, SqliteEmbeddingStream, SqliteTextStream
            query.py         # FilterChain — accumulates predicates, generates SQL
            _sql.py          # SQL helpers, identifier validation, pose helpers, serialization
```

## Dependencies

- `sqlite3` (stdlib)
- `sqlite-vec` — vector similarity search via vec0 virtual table. Optional — `search_embedding` raises if unavailable.
- FTS5 — built into SQLite by default on most platforms.
- R*Tree — built into SQLite by default.
- `reactivex` — for `.appended` observable (already a DimOS dependency).

## Connection Management

### SqliteStore

```python
class SqliteStore(Store):
    def __init__(self, path: str):
        self.path = path  # or ":memory:"

    def session(self) -> SqliteSession:
        conn = self._connect()
        return SqliteSession(conn)

    def _connect(self) -> sqlite3.Connection:
        if self.path == ":memory:":
            uri = "file::memory:?cache=shared"
            conn = sqlite3.connect(uri, uri=True)
        else:
            Path(self.path).parent.mkdir(parents=True, exist_ok=True)
            conn = sqlite3.connect(self.path)

        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=NORMAL")
        conn.execute("PRAGMA foreign_keys=ON")

        # Try loading sqlite-vec
        try:
            conn.enable_load_extension(True)
            conn.load_extension("vec0")  # or find via sqlite_vec.loadable_path()
            conn.enable_load_extension(False)
        except Exception:
            pass  # vec0 unavailable — search_embedding will raise

        return conn

    def close(self) -> None: ...
```

### SqliteSession

```python
class SqliteSession(Session):
    def __init__(self, conn: sqlite3.Connection):
        self._conn = conn
        self._streams: dict[str, SqliteStream] = {}  # cache by name
        self._ensure_registry()

    def _ensure_registry(self):
        """Create _streams table if not exists."""
        self._conn.execute("""
            CREATE TABLE IF NOT EXISTS _streams (
                rowid INTEGER PRIMARY KEY,
                name TEXT UNIQUE NOT NULL,
                type TEXT NOT NULL,
                payload_type TEXT,
                parent_stream TEXT,
                embedding_dim INTEGER
            )
        """)

    def stream(self, name, payload_type=None, *, pose_provider=None) -> SqliteStream:
        if name in self._streams:
            return self._streams[name]
        self._register_stream(name, "blob", payload_type)
        self._create_stream_tables(name, stream_type="blob")
        s = SqliteStream(name, self._conn, payload_type, pose_provider)
        self._streams[name] = s
        return s

    def text_stream(self, name, payload_type=None, *, tokenizer="unicode61",
                    pose_provider=None) -> SqliteTextStream:
        # Similar — creates FTS tables too
        ...

    def list_streams(self) -> list[StreamInfo]: ...
    def close(self) -> None: self._conn.close()
```

## Schema

All table names are prefixed with the stream name. Stream names are validated: `[a-zA-Z_][a-zA-Z0-9_]*`, max 64 chars.

### `_streams` — Global registry

```sql
CREATE TABLE _streams (
    rowid INTEGER PRIMARY KEY,
    name TEXT UNIQUE NOT NULL,
    type TEXT NOT NULL,              -- 'blob', 'embedding', 'text'
    payload_type TEXT,               -- e.g. 'dimos.msgs.sensor_msgs.Image'
    parent_stream TEXT,              -- FK name of parent stream (lineage)
    embedding_dim INTEGER            -- only for type='embedding'
);
```

### `{name}_meta` — Observation metadata (all stream types)

```sql
CREATE TABLE {name}_meta (
    rowid INTEGER PRIMARY KEY,       -- = Observation.id
    ts REAL,
    pose_x REAL, pose_y REAL, pose_z REAL,
    pose_qx REAL, pose_qy REAL, pose_qz REAL, pose_qw REAL,
    tags TEXT,                        -- JSON dict, NULL if empty
    parent_rowid INTEGER              -- lineage: rowid in parent stream's _meta
);
CREATE INDEX idx_{name}_ts ON {name}_meta(ts);
```

### `{name}_payload` — Blob/Text payload (not EmbeddingStream)

```sql
CREATE TABLE {name}_payload (
    rowid INTEGER PRIMARY KEY,       -- matches _meta.rowid
    data BLOB NOT NULL               -- TextStream: TEXT instead of BLOB
);
```

Separated from `_meta` so metadata queries never page in multi-MB blobs.

### `{name}_rtree` — Spatial index (all stream types)

```sql
CREATE VIRTUAL TABLE {name}_rtree USING rtree(
    rowid,                            -- matches _meta.rowid
    min_x, max_x,
    min_y, max_y,
    min_z, max_z
);
```

Only rows with pose are inserted into R*Tree. Rows without pose are excluded from `.near()` results.

### `{name}_fts` — Full-text search (TextStream only)

```sql
CREATE VIRTUAL TABLE {name}_fts USING fts5(
    content,
    tokenize='{tokenizer}'
);
```

Standalone FTS table (not content-synced). Rowids match `_meta.rowid`.

### `{name}_vec` — Vector index (EmbeddingStream only)

```sql
CREATE VIRTUAL TABLE {name}_vec USING vec0(
    embedding float[{dim}]
);
```

Rowids match `_meta.rowid`. Dimension inferred from first embedding inserted, or from `EmbeddingModel.embed()` output.

## Stream Implementation

### SqliteStream (implements Stream[T])

Internally, a stream object can be in different modes:

```python
@dataclass
class StoredBacking:
    """Root DB-backed stream. Created by session.stream()."""
    name: str

@dataclass
class FilteredBacking:
    """Lazy predicate chain. Created by .after(), .near(), etc."""
    parent: StreamBacking  # recursive — can chain filters
    predicates: list[Predicate]
    ordering: list[OrderClause]
    limit_val: int | None
    offset_val: int | None

@dataclass
class TransformBacking:
    """Unevaluated transform. Created by .transform()."""
    source: StreamBacking
    transformer: Transformer
    live: bool
    backfill_only: bool

Backing = StoredBacking | FilteredBacking | TransformBacking
```

The stream carries its backing and resolves it at terminal time.

### append()

Only valid on `StoredBacking`. Otherwise raises `TypeError`.

```python
def append(self, payload, *, ts=None, pose=None, tags=None):
    if not isinstance(self._backing, StoredBacking):
        raise TypeError("append() only valid on stored streams")

    ts = ts or time.time()
    pose = pose or (self._pose_provider() if self._pose_provider else None)

    # 1. Insert into _meta
    meta_rowid = self._insert_meta(ts, pose, tags, parent_rowid=None)

    # 2. Insert into _payload
    blob = serialize(payload)  # see Serialization section
    self._conn.execute(
        f"INSERT INTO {name}_payload(rowid, data) VALUES (?, ?)",
        (meta_rowid, blob)
    )

    # 3. Insert into _rtree (if pose)
    if pose:
        x, y, z = extract_position(pose)
        self._conn.execute(
            f"INSERT INTO {name}_rtree(rowid, min_x, max_x, min_y, max_y, min_z, max_z) "
            "VALUES (?, ?, ?, ?, ?, ?, ?)",
            (meta_rowid, x, x, y, y, z, z)
        )

    self._conn.commit()

    # 4. Build Observation and emit
    obs = Observation(id=meta_rowid, ts=ts, pose=pose, tags=tags or {})
    obs._data = payload  # pre-populated
    self._appended_subject.on_next(obs)
    return obs
```

### EmbeddingStream.append()

Same as above but inserts into `_vec` instead of `_payload`:

```python
# Insert embedding vector
vec_data = embedding.to_numpy().tobytes()
self._conn.execute(
    f"INSERT INTO {name}_vec(rowid, embedding) VALUES (?, ?)",
    (meta_rowid, vec_data)
)
```

### TextStream.append()

Inserts into both `_payload` (TEXT) and `_fts`:

```python
self._conn.execute(
    f"INSERT INTO {name}_payload(rowid, data) VALUES (?, ?)",
    (meta_rowid, text_content)
)
self._conn.execute(
    f"INSERT INTO {name}_fts(rowid, content) VALUES (?, ?)",
    (meta_rowid, text_content)
)
```

## Filter → SQL Generation

Each filter method returns a new stream with a `FilteredBacking` wrapping the current backing. At terminal time, the filter chain is compiled to SQL.

### Predicate types

```python
@dataclass
class AfterPred:
    t: float
    # → WHERE ts > ?

@dataclass
class BeforePred:
    t: float
    # → WHERE ts < ?

@dataclass
class TimeRangePred:
    t1: float
    t2: float
    # → WHERE ts BETWEEN ? AND ?

@dataclass
class AtPred:
    t: float
    tolerance: float
    # → WHERE ts BETWEEN ? AND ? ORDER BY ABS(ts - ?) LIMIT 1

@dataclass
class NearPred:
    x: float
    y: float
    z: float
    radius: float
    # → JOIN with _rtree bounding box query

@dataclass
class TagsPred:
    tags: dict[str, Any]
    # → WHERE json_extract(tags, '$.key') = ?

@dataclass
class TextSearchPred:
    text: str
    k: int | None
    # → JOIN with _fts MATCH

@dataclass
class EmbeddingSearchPred:
    vector: list[float]
    k: int
    # → query _vec for top-k, then filter
```

### SQL compilation

Walk the backing chain to the root `StoredBacking`, collect all predicates, then generate SQL:

```python
def _compile(self) -> tuple[str, list[Any]]:
    """Walk backing chain, return (sql, params)."""
    root_name = self._find_root_name()
    predicates = self._collect_predicates()
    ordering = self._collect_ordering()
    limit = self._collect_limit()
    offset = self._collect_offset()

    # Start with base SELECT
    sql = f"SELECT rowid, ts, pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw, tags FROM {root_name}_meta"
    params = []
    joins = []
    wheres = []

    for pred in predicates:
        if isinstance(pred, AfterPred):
            wheres.append("ts > ?")
            params.append(pred.t)
        elif isinstance(pred, NearPred):
            joins.append(
                f"JOIN {root_name}_rtree r ON r.rowid = {root_name}_meta.rowid"
            )
            wheres.append(
                "r.min_x >= ? AND r.max_x <= ? AND "
                "r.min_y >= ? AND r.max_y <= ? AND "
                "r.min_z >= ? AND r.max_z <= ?"
            )
            params.extend([
                pred.x - pred.radius, pred.x + pred.radius,
                pred.y - pred.radius, pred.y + pred.radius,
                pred.z - pred.radius, pred.z + pred.radius,
            ])
        elif isinstance(pred, TagsPred):
            for key, val in pred.tags.items():
                wheres.append(f"json_extract(tags, '$.{key}') = ?")
                params.append(val)
        # ... etc

    sql += " " + " ".join(joins)
    if wheres:
        sql += " WHERE " + " AND ".join(wheres)
    if ordering:
        sql += " ORDER BY " + ", ".join(ordering)
    if limit is not None:
        sql += " LIMIT ?"
        params.append(limit)
    if offset is not None:
        sql += " OFFSET ?"
        params.append(offset)

    return sql, params
```

### search_embedding (vec0)

```sql
-- Top-k vector search
SELECT rowid, distance
FROM {name}_vec
WHERE embedding MATCH ?
  AND k = ?
ORDER BY distance
```

Returns rowids, which are then used to filter `_meta`. This is a two-step process:
1. Get top-k rowids from vec0
2. Fetch metadata for those rowids

### search_text (FTS5)

```sql
SELECT rowid, rank
FROM {name}_fts
WHERE {name}_fts MATCH ?
ORDER BY rank
```

Same two-step: get rowids from FTS5, then fetch metadata.

## Terminal Execution

### __iter__() — lazy iteration

`Stream` is directly iterable. Pages internally via `fetch_pages`, yielding one `Observation` at a time:

```python
def __iter__(self) -> Iterator[Observation]:
    for page in self.fetch_pages():
        yield from page
```

### fetch()

```python
def fetch(self) -> list[Observation]:
    sql, params = self._compile()
    rows = self._conn.execute(sql, params).fetchall()
    return [self._row_to_observation(row) for row in rows]
```

### fetch_pages()

```python
def fetch_pages(self, batch_size=128) -> Iterator[list[Observation]]:
    sql, params = self._compile()
    # Add LIMIT/OFFSET pagination
    offset = 0
    while True:
        page_sql = sql + f" LIMIT {batch_size} OFFSET {offset}"
        rows = self._conn.execute(page_sql, params).fetchall()
        if not rows:
            break
        yield [self._row_to_observation(row) for row in rows]
        offset += batch_size
```

### count()

```python
def count(self) -> int:
    sql, params = self._compile()
    count_sql = f"SELECT COUNT(*) FROM ({sql})"
    return self._conn.execute(count_sql, params).fetchone()[0]
```

### one() / last()

- `one()` → adds `LIMIT 1` to the query
- `last()` → adds `ORDER BY ts DESC LIMIT 1`

## Lazy Data Loading

`Observation.data` uses lazy loading. The implementation:

```python
@dataclass
class Observation:
    id: int
    ts: float | None = None
    pose: PoseStamped | None = None
    tags: dict[str, Any] = field(default_factory=dict)
    _data: Any = field(default=_SENTINEL, repr=False)
    _load: Callable[[], Any] | None = field(default=None, repr=False)

    @property
    def data(self) -> Any:
        if self._data is _SENTINEL and self._load is not None:
            self._data = self._load()
        return self._data
```

When building observations from query results:

```python
def _row_to_observation(self, row) -> Observation:
    rowid = row[0]
    obs = Observation(
        id=rowid,
        ts=row[1],
        pose=reconstruct_pose(row[2:9]),
        tags=json.loads(row[9]) if row[9] else {},
    )
    name = self._root_name()
    conn = self._conn
    obs._load = lambda: deserialize(
        conn.execute(f"SELECT data FROM {name}_payload WHERE rowid = ?", (rowid,)).fetchone()[0]
    )
    return obs
```

### EmbeddingObservation

For `EmbeddingStream`, terminals return `EmbeddingObservation` which auto-projects `.data` to the source stream:

```python
def _row_to_embedding_observation(self, row) -> EmbeddingObservation:
    rowid = row[0]
    parent_stream = self._get_parent_stream_name()
    obs = EmbeddingObservation(id=rowid, ts=row[1], ...)

    # .data loads from PARENT stream (auto-projection)
    obs._load = lambda: deserialize(
        conn.execute(
            f"SELECT data FROM {parent_stream}_payload WHERE rowid = ?",
            (conn.execute(
                f"SELECT parent_rowid FROM {self._name}_meta WHERE rowid = ?",
                (rowid,)
            ).fetchone()[0],)
        ).fetchone()[0]
    )

    # .embedding loads from _vec
    obs._embedding_load = lambda: Embedding(
        np.frombuffer(
            conn.execute(
                f"SELECT embedding FROM {self._name}_vec WHERE rowid = ?",
                (rowid,)
            ).fetchone()[0],
            dtype=np.float32
        )
    )
    return obs
```

## Lineage & join

### Storing lineage

When a Transformer appends to a target stream, `parent_rowid` links back to the source:

```python
# Inside Transformer execution
target.append(result, ts=source_obs.ts, pose=source_obs.pose,
              _parent_rowid=source_obs.id)  # internal param
```

The `_streams` registry tracks stream-level lineage:
```python
# When .store() creates from a transform
INSERT INTO _streams (name, type, payload_type, parent_stream)
VALUES ('detections', 'blob', '...', 'images')
```

### join()

Returns tuples of `(self_obs, target_obs)` linked by lineage:

```sql
-- Join self with target via parent_rowid
SELECT
    c.rowid, c.ts, c.pose_x, ...,   -- self (e.g., detections)
    p.rowid, p.ts, p.pose_x, ...    -- target (e.g., images)
FROM {self}_meta c
JOIN {target}_meta p ON c.parent_rowid = p.rowid
WHERE c.rowid IN (/* current filtered set */)
```

Iteration yields `tuple[Observation, Observation]` — both sides have lazy `.data`.

## Transform Execution

### .transform() — returns lazy stream

`.transform(xf)` doesn't execute immediately. It returns a new stream with `TransformBacking`. Execution happens at terminal time or `.store()`.

### .store() — materializes

When `.store(name)` is called on a transform-backed stream:

1. Register target stream in `_streams` (with `parent_stream` set)
2. Create target tables (`_meta`, `_payload`, etc.)
3. If not `live` mode: run `xf.process(source_stream, target_stream)` (backfill)
4. If not `backfill_only`: subscribe to source's `.appended` observable, call `xf.on_append()` for each new item
5. Return the stored stream (now `StoredBacking`)

```python
def store(self, name):
    if not isinstance(self._backing, TransformBacking):
        # Already stored or predicate-backed — different path
        ...

    tb = self._backing
    # Create target stream
    target = self._session._create_stream(name, ...)

    # Register lineage
    self._session._register_lineage(name, parent_stream=source_name)

    # Backfill
    if not tb.live and tb.transformer.supports_backfill:
        source_stream = self._resolve_source()
        tb.transformer.process(source_stream, target)

    # Live subscription
    if not tb.backfill_only and tb.transformer.supports_live:
        source_stream = self._resolve_source()
        source_stream.appended.subscribe(
            lambda obs: tb.transformer.on_append(obs, target)
        )

    return target
```

### Incremental backfill

When re-opening a previously stored transform, check what's already been processed:

```python
# Find max parent_rowid already processed
max_parent = conn.execute(
    f"SELECT MAX(parent_rowid) FROM {target_name}_meta"
).fetchone()[0]

# Only process source rows after that
if max_parent is not None:
    source = source.after_id(max_parent)  # internal method
```

### .fetch() on transform-backed stream (no .store())

If `.fetch()` is called on a transform-backed stream without `.store()`, execute the transform in-memory:

1. Fetch source observations
2. Apply transformer's `process()` with an in-memory target
3. Return results without persisting

This is useful for one-off transforms but can cause memory pressure with large datasets.

## Reactive (.appended)

Each stored stream has a `ReplaySubject` (or `Subject`) from reactivex:

```python
class SqliteStream:
    def __init__(self, ...):
        self._appended_subject = Subject()

    @property
    def appended(self) -> Observable[Observation]:
        return self._appended_subject.pipe(...)
```

`append()` emits to the subject after the DB write succeeds.

For filtered streams (`.after(t).near(pose, 5.0).appended`), the observable filters events through the predicate chain in Python:

```python
@property
def appended(self):
    root = self._find_root_stream()
    predicates = self._collect_predicates()
    return root.appended.pipe(
        ops.filter(lambda obs: all(p.matches(obs) for p in predicates))
    )
```

Each predicate type implements `matches(obs) -> bool` for Python-side filtering.

## Serialization

### Payload serialization

Use Python `pickle` for general types, with an optimization path for known DimOS types (LCM-encoded messages):

```python
def serialize(payload: Any) -> bytes:
    # LCM types: use lcm_encode for compact binary
    if hasattr(payload, '_get_packed_fingerprint'):
        return lcm_encode(payload)
    # Fallback: pickle
    return pickle.dumps(payload)

def deserialize(blob: bytes, payload_type: type | None = None) -> Any:
    if payload_type and hasattr(payload_type, '_get_packed_fingerprint'):
        return lcm_decode(blob, payload_type)
    return pickle.loads(blob)
```

### Pose helpers

```python
def extract_position(pose: PoseLike) -> tuple[float, float, float]:
    """Extract (x, y, z) from any PoseLike."""
    if isinstance(pose, PoseStamped):
        p = pose.pose.position
        return (p.x, p.y, p.z)
    # ... handle Pose, Point, PointStamped

def extract_orientation(pose: PoseLike) -> tuple[float, float, float, float] | None:
    """Extract (qx, qy, qz, qw) if available."""
    ...

def reconstruct_pose(row_slice) -> PoseStamped | None:
    """Rebuild PoseStamped from (x, y, z, qx, qy, qz, qw) columns."""
    x, y, z, qx, qy, qz, qw = row_slice
    if x is None:
        return None
    ...
```

### Tag serialization

Tags are stored as JSON text. `None`/empty dict → `NULL` in the column.

```python
tags_json = json.dumps(tags) if tags else None
```

## SQL Safety

- **Identifier validation**: stream names must match `^[a-zA-Z_][a-zA-Z0-9_]{0,63}$`. Reject anything else with `ValueError`.
- **Parameterized queries**: all user values go through `?` params, never string interpolation.
- **Table names**: constructed from validated stream names, so they're safe for SQL interpolation (e.g., `f"{name}_meta"`).

## Thread Safety

- Each `Session` owns one `sqlite3.Connection` — not shared across threads.
- Multiple sessions can exist on the same file (WAL mode allows concurrent reads + one writer).
- The `appended` subject emits on the thread that called `append()`.

## Error Handling

- `append()` on non-stored stream → `TypeError`
- `search_embedding()` on non-embedding stream → `TypeError`
- `search_text()` on non-text stream → `TypeError`
- `search_embedding()` when sqlite-vec not loaded → `RuntimeError`
- Invalid stream name → `ValueError`
- `one()` with no results → `LookupError`

## Testing

Tests go in `dimos/memory/tests/test_sqlite.py`. Use `:memory:` store for speed.

Key test scenarios:
1. Create stream, append, fetch — verify data round-trips
2. Temporal filters (after, before, time_range, at)
3. Spatial filter (near) — with and without pose
4. Tag filtering
5. EmbeddingStream — store embeddings, search_embedding, verify EmbeddingObservation auto-projects .data
6. TextStream — store text, search_text
7. Transform with lambda — verify lineage
8. Transform with Transformer class — verify process() called
9. Chained filters — verify SQL composition
10. join — verify cross-stream lineage returns tuples
11. fetch_pages — verify pagination
12. Lazy data loading — verify .data only hits DB on access
13. .appended observable — verify reactive emission
14. Incremental backfill — verify resume from last processed
15. Multiple sessions on same file
