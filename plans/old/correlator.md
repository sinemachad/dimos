# Correlator

Cross-stream temporal+spatial join for Memory2.

## Motivation

5 of 14 usage questions (Q5, Q6, Q7, Q10, Q14) require the same pattern:

```python
for anchor in stream_a.query().fetch():
    matches = (stream_b.query()
        .filter_time(anchor.ts_start - tol, anchor.ts_end + tol)
        .filter_near(anchor.pose, radius=r)
        .fetch())
    # do something with (anchor, matches)
```

This is a nested-loop join — N queries, one per anchor observation. Correlator replaces it with a single batch operation.

## API

Method on Session:

```python
class Session:
    def correlate(
        self,
        anchors: Stream | ObservationSet,
        targets: Stream | ObservationSet,
        *,
        time_tolerance: float | None = None,  # symmetric: sets both before and after
        time_before: float | None = None,      # asymmetric: window before anchor ts_start
        time_after: float | None = None,       # asymmetric: window after anchor ts_end
        spatial_radius: float | None = None,
    ) -> CorrelationResult: ...
```

Accepts Stream (correlate everything) or ObservationSet (correlate a filtered subset).

**Time window per anchor**: `[ts_start - time_before, ts_end + time_after]`. If `ts_end` is None, uses `ts_start` for both. `time_tolerance` is shorthand for `time_before = time_after = time_tolerance`. Explicit `time_before`/`time_after` override `time_tolerance`.

### CorrelationResult

```python
@dataclass
class CorrelationPair:
    anchor: ObservationRow
    matches: list[ObservationRow]

class CorrelationResult:
    def __iter__(self) -> Iterator[CorrelationPair]: ...
    def __len__(self) -> int: ...

    # Filter by match cardinality
    def unambiguous(self) -> list[CorrelationPair]:
        """Pairs where exactly one target matched."""
        ...

    def with_matches(self) -> list[CorrelationPair]:
        """Pairs where at least one target matched."""
        ...

    def unmatched(self) -> list[ObservationRow]:
        """Anchor observations with zero matches."""
        ...
```

### Usage

**Q5 — Who opened the door?**
```python
door_events = events.query().filter_tags(type="door_open").fetch_set()

pairs = s.correlate(door_events, faces, time_tolerance=5.0, spatial_radius=3.0)
for p in pairs.with_matches():
    identity = identify_face(faces.vector(p.matches[0].ref))
    print(f"Door opened at {p.anchor.ts_start} — {identity}")
```

**Q7 — Voice ↔ face (unambiguous only)**
```python
pairs = s.correlate(voices, faces, time_tolerance=0.5)
for p in pairs.unambiguous():
    voice_vec = voices.vector(p.anchor.ref)
    face_vec = faces.vector(p.matches[0].ref)
    pairings.append((voice_vec, face_vec))
```

**Q10 — Pre-event timeline (30s before, nothing after)**
```python
vase_event = events.query().search_text("vase fell").fetch_set()

timeline = {}
for info in s.list_streams():
    stream = s.stream(info.name, info.payload_type,
                      embedding=info.embedding, text=info.text)
    result = s.correlate(vase_event, stream, time_before=30.0, time_after=0.0)
    timeline[info.name] = result
```

**Q14 — Distance while carrying**
```python
carrying = detections.query().filter_tags(action="carrying").fetch_set()

pairs = s.correlate(carrying, poses, time_tolerance=0.0)
total_distance = 0.0
for p in pairs:
    sorted_poses = sorted(p.matches, key=lambda r: r.ts_start)
    for i in range(1, len(sorted_poses)):
        total_distance += distance(sorted_poses[i-1].pose, sorted_poses[i].pose)
```

## Implementation

### SQL batch join (single query instead of N)

```sql
-- 1. Materialize anchors into temp table
CREATE TEMP TABLE _corr_anchors (
    anchor_id TEXT,
    ts_lo REAL,       -- ts_start - time_before
    ts_hi REAL,       -- (ts_end or ts_start) + time_after
    pose_x REAL,
    pose_y REAL,
    pose_z REAL
);

-- 2. Join to target stream's _meta on time overlap
SELECT a.anchor_id, t.*
FROM _corr_anchors a
JOIN {target}_meta t
  ON t.ts_start >= a.ts_lo AND t.ts_start <= a.ts_hi
ORDER BY a.anchor_id, t.ts_start;
```

With spatial constraint, add R*Tree join:

```sql
SELECT a.anchor_id, t.*
FROM _corr_anchors a
JOIN {target}_rtree r
  ON r.min_x >= a.pose_x - :radius AND r.max_x <= a.pose_x + :radius
 AND r.min_y >= a.pose_y - :radius AND r.max_y <= a.pose_y + :radius
 AND r.min_z >= a.pose_z - :radius AND r.max_z <= a.pose_z + :radius
 AND r.min_t >= a.ts_lo AND r.max_t <= a.ts_hi
JOIN {target}_meta t ON t.rowid = r.rowid
ORDER BY a.anchor_id, t.ts_start;
```

### Grouping

The SQL returns flat rows sorted by `anchor_id`. Group in Python into `CorrelationPair`s:

```python
pairs = []
current_anchor = None
current_matches = []
for row in cursor:
    anchor_id = row["anchor_id"]
    if anchor_id != current_anchor:
        if current_anchor is not None:
            pairs.append(CorrelationPair(anchor=..., matches=current_matches))
        current_anchor = anchor_id
        current_matches = []
    current_matches.append(to_observation_row(row))
```

### Performance

- Temp table insert: O(A) where A = anchor count
- Join: SQLite uses the ts_start index (or R*Tree) on the target → O(A × log(T)) where T = target count
- vs naive loop: O(A × T) without indexes, O(A × log(T)) with indexes but A round-trips

The batch approach saves round-trip overhead and lets SQLite optimize the join plan. For A=1000 anchors, that's 1 query vs 1000.

## Types

```python
# In types.py
@dataclass
class CorrelationPair:
    anchor: ObservationRow
    matches: list[ObservationRow]
```

```python
# In correlation.py
class CorrelationResult:
    _pairs: list[CorrelationPair]

    def __iter__(self): return iter(self._pairs)
    def __len__(self): return len(self._pairs)

    def unambiguous(self) -> list[CorrelationPair]:
        return [p for p in self._pairs if len(p.matches) == 1]

    def with_matches(self) -> list[CorrelationPair]:
        return [p for p in self._pairs if p.matches]

    def unmatched(self) -> list[ObservationRow]:
        return [p.anchor for p in self._pairs if not p.matches]
```

## File structure update

```
dimos/memory2/
    ...
    correlation.py           # CorrelationResult, correlate() implementation
```

## Phase

This can be Phase 2b — after Stream + Query + ObservationSet are working. It depends on:
- ObservationSet (anchors can be a set)
- Stream._meta table schema (join target)
- Session.execute() (raw SQL for the batch join)

No dependency on derive(), CompositeBacking, or retention.

## Not in scope

- **Continuous/streaming correlation** — this is one-shot batch. Live correlation (new anchor arrives → auto-query targets) is a different abstraction.
- **Multi-stream correlation** — correlate(A, [B, C, D]) returning aligned tuples. Call correlate() multiple times instead.
- **Embedding cross-match** — correlation is time+space only. "Find similar embeddings across streams" is a different operation (use search_embedding on each stream).
