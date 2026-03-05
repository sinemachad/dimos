# Analysis Utilities

Application-level analysis on Memory2 query results. NOT part of memory2 core — operates on fetched `ObservationRow` lists, no SQLite dependency.

Location: `dimos/memory2/analysis.py`

Dependencies: only `dimos/memory2/types.py` (ObservationRow, ObservationRef). No numpy, no sklearn.

---

## 1. `cluster_observations()`

The most common post-query pattern across Q2, Q4, Q5, Q9, Q11, Q12, Q14.

```python
@dataclass
class Cluster:
    rows: list[ObservationRow]
    representative: ObservationRow   # best by rank_key

    @property
    def t_start(self) -> float:
        return self.rows[0].ts_start

    @property
    def t_end(self) -> float:
        return self.rows[-1].ts_start

    @property
    def duration(self) -> float:
        return self.t_end - self.t_start

    @property
    def center_pose(self) -> PoseLike | None:
        """Average position of all localized rows."""
        ...


def cluster_observations(
    rows: list[ObservationRow],
    *,
    time_scale: float | None = None,
    space_scale: float | None = None,
    threshold: float = 1.0,
    rank_key: Callable[[ObservationRow], float] | None = None,
) -> list[Cluster]:
    """Greedy sequential clustering over time and/or space.

    Distance between consecutive rows (must be sorted by ts_start):

        d = sqrt((dt/time_scale)^2 + (ds/space_scale)^2)

    New cluster starts when d > threshold.

    Args:
        rows: ObservationRows, sorted by ts_start.
        time_scale: Normalize temporal gap (seconds). None = ignore time.
        space_scale: Normalize spatial distance (meters). None = ignore space.
        threshold: Combined normalized distance to split clusters.
        rank_key: Scoring function for representative selection.
                  Default: embedding score, then recency.

    Returns:
        List of Cluster objects, each with .rows and .representative.
    """
```

### Modes

```python
# Temporal only: split if gap > 10s
clusters = cluster_observations(rows, time_scale=10.0)

# Spatial only: split if > 3m apart
clusters = cluster_observations(rows, space_scale=3.0)

# Combined: either 10s gap OR 3m apart triggers split
clusters = cluster_observations(rows, time_scale=10.0, space_scale=3.0)

# Bias toward spatial (space matters more):
clusters = cluster_observations(rows, time_scale=30.0, space_scale=2.0)
```

### Representative selection

Default `rank_key`: `lambda r: r.scores.get("embedding", 0)` — picks the most relevant frame after a search. Override for quality-based selection:

```python
# Quality-biased: prefer sharp, well-exposed frames
clusters = cluster_observations(rows,
    time_scale=10.0,
    rank_key=lambda r: (
        r.scores.get("embedding", 0) * 0.4 +
        r.tags.get("sharpness", 0.5) * 0.4 +
        r.tags.get("exposure", 0.5) * 0.2
    ),
)

# Recency-biased: prefer the latest frame in each cluster
clusters = cluster_observations(rows,
    time_scale=10.0,
    rank_key=lambda r: r.ts_start,
)
```

### Which questions use this

| Question | Mode | Purpose |
|----------|------|---------|
| Q2 — red socks viewing sessions | temporal | Group continuous sightings, VLM one per cluster |
| Q4 — where were red socks | spatial | Group nearby sightings into distinct locations |
| Q5 — door open events | temporal | Group rapid-fire "door open" detections into single events |
| Q9 — cat trail | spatial | Group into distinct locations the cat visited |
| Q11 — cat absence | temporal | (indirect — use `find_gaps` on clusters) |
| Q12 — mailman schedule | temporal | Group same-visit detections into single arrival events |
| Q14 — carrying intervals | temporal | Group "carrying" detections into continuous intervals |

---

## 2. `find_gaps()`

Find periods where observations are absent. Used in Q11 (cat absence) and Q14 (carrying interval boundaries).

```python
@dataclass
class Gap:
    t_start: float    # timestamp of last observation before the gap
    t_end: float      # timestamp of first observation after the gap
    duration: float   # t_end - t_start


def find_gaps(
    rows: list[ObservationRow],
    *,
    min_gap: float,
) -> list[Gap]:
    """Find temporal gaps in a sorted observation list.

    Args:
        rows: ObservationRows, sorted by ts_start.
        min_gap: Minimum gap duration (seconds) to report.

    Returns:
        List of Gap objects, sorted by time.
    """
```

Usage:

```python
# Q11: When was the cat last NOT seen?
cat_seen = detections.query().filter_tags(class_name="cat").order_by("ts_start").fetch()
gaps = find_gaps(cat_seen, min_gap=60.0)
if gaps:
    print(f"Last absence: {gaps[-1].t_start} to {gaps[-1].t_end}")
```

Works on clusters too — find gaps between cluster end and next cluster start:

```python
# Gaps between sighting sessions (not between individual frames)
clusters = cluster_observations(cat_seen, time_scale=10.0)
# Synthesize one row per cluster (the representative) for gap analysis
cluster_reps = [c.representative for c in clusters]
session_gaps = find_gaps(cluster_reps, min_gap=300.0)
```

---

## 3. `compute_path_distance()`

Sum of Euclidean distances along a pose trail. Used in Q9 (cat trail length) and Q14 (distance while carrying).

```python
def compute_path_distance(
    rows: list[ObservationRow],
) -> float:
    """Total Euclidean path distance from consecutive poses.

    Args:
        rows: ObservationRows with poses, sorted by ts_start.
              Rows without pose are skipped.

    Returns:
        Total distance in meters.
    """
```

Usage:

```python
# Q14: How far did I travel while carrying?
for cluster in carrying_clusters:
    pose_rows = poses.query().filter_time(cluster.t_start, cluster.t_end).order_by("ts_start").fetch()
    dist = compute_path_distance(pose_rows)
    print(f"Carried for {cluster.duration:.0f}s, traveled {dist:.1f}m")
```

---

## 4. `extract_time_pattern()`

Extract time-of-day statistics from observations spread across multiple days. Used in Q12 (mailman schedule).

```python
@dataclass
class TimePattern:
    mean_hour: float          # e.g. 10.5 = 10:30 AM
    std_minutes: float        # standard deviation in minutes
    count: int                # number of observations
    times: list[float]        # individual hours (for histogram)

    def __str__(self) -> str:
        h = int(self.mean_hour)
        m = int((self.mean_hour % 1) * 60)
        return f"{h}:{m:02d} +/- {self.std_minutes:.0f}min (n={self.count})"


def extract_time_pattern(
    rows: list[ObservationRow],
    *,
    tz: timezone | None = None,
) -> TimePattern:
    """Extract time-of-day pattern from observations across multiple days.

    Best used on cluster representatives (one per event) rather than raw rows,
    to avoid dense clusters biasing the average.

    Args:
        rows: ObservationRows with ts_start.
        tz: Timezone for time-of-day extraction. Default: UTC.

    Returns:
        TimePattern with mean, std, and individual times.
    """
```

Usage:

```python
# Q12: When does the mailman usually come?
sightings = faces.query().search_embedding(mailman_emb, candidate_k=100).fetch()
sightings = [r for r in sightings if r.scores.get("embedding", 0) > 0.8]

# Cluster into individual visits (one per day)
visits = cluster_observations(sightings, time_scale=300.0)
pattern = extract_time_pattern([v.representative for v in visits])
print(f"Mailman comes at {pattern}")  # "10:30 +/- 12min (n=23)"
```

---

## 5. `match_viewpoints()`

Match observations from two sets by embedding similarity — find corresponding views across time. Used in Q8 (room diff: today vs yesterday).

```python
@dataclass
class ViewpointMatch:
    current: ObservationRow
    reference: ObservationRow
    similarity: float


def match_viewpoints(
    current: list[ObservationRow],
    reference: list[ObservationRow],
    vectors_current: list[list[float]],
    vectors_reference: list[list[float]],
    *,
    min_similarity: float = 0.85,
) -> list[ViewpointMatch]:
    """Match observations by embedding similarity (cosine via dot product).

    Assumes vectors are L2-normalized (as CLIP embeddings are).

    Pure Python — no numpy required (but callers may use numpy for
    batch vector retrieval before calling this).

    Args:
        current: ObservationRows from the "current" time window.
        reference: ObservationRows from the "reference" time window.
        vectors_current: Embedding vectors for current rows.
        vectors_reference: Embedding vectors for reference rows.
        min_similarity: Minimum cosine similarity for a valid match.

    Returns:
        List of ViewpointMatch objects, one per matched current row.
        Unmatched rows are excluded.
    """
```

Usage:

```python
# Q8: What changed in this room vs yesterday?
current_imgs = images.query().filter_time(now - 300, now).filter_near(pose, radius=5.0).fetch()
yesterday_imgs = images.query().filter_time(yest - 300, yest + 300).filter_near(pose, radius=5.0).fetch()

current_vecs = [images.vector(r.ref) for r in current_imgs]
yesterday_vecs = [images.vector(r.ref) for r in yesterday_imgs]

matches = match_viewpoints(current_imgs, yesterday_imgs, current_vecs, yesterday_vecs)
for m in matches:
    diff = vlm.ask([images.load(m.current.ref), images.load(m.reference.ref)],
                   "What changed between these two views?")
```

Note: this is O(n*m) dot products. Fine for typical sizes (tens to low hundreds of images per spatial query). For very large sets, callers can use numpy directly.

---

## 6. `diff_observation_sets()`

Find observations in set A that have no similar match in set B. Used in Q13 (cross-robot diff).

```python
@dataclass
class UnmatchedObservation:
    row: ObservationRow
    best_similarity: float    # highest similarity to anything in the other set


def diff_observation_sets(
    source: list[ObservationRow],
    reference: list[ObservationRow],
    vectors_source: list[list[float]],
    vectors_reference: list[list[float]],
    *,
    similarity_threshold: float = 0.7,
) -> list[UnmatchedObservation]:
    """Find observations in source that have no close match in reference.

    Args:
        source: Observations to check ("what did robot-2 see?")
        reference: Observations to compare against ("what did I see?")
        vectors_source: Embeddings for source rows.
        vectors_reference: Embeddings for reference rows.
        similarity_threshold: Below this = "unmatched" = novel observation.

    Returns:
        List of UnmatchedObservation from source with no reference match.
    """
```

Usage:

```python
# Q13: What did robot-2 see that I missed?
r2 = detections.query().filter_tags(robot_id="robot-2").filter_near(warehouse, radius=20).fetch()
me = detections.query().filter_tags(robot_id="robot-1").filter_near(warehouse, radius=20).fetch()
r2_vecs = [detections.vector(r.ref) for r in r2]
me_vecs = [detections.vector(r.ref) for r in me]

missed = diff_observation_sets(r2, me, r2_vecs, me_vecs)
for m in missed:
    print(f"Missed: {m.row.tags.get('class_name')} at {m.row.pose}")
```

---

## Quality Conventions

Image quality metrics are stored in tags at ingest time by the pipeline. The analysis utilities don't compute quality — they consume it via `rank_key`.

### Recommended tag keys

| Tag | Type | Description | Range |
|-----|------|-------------|-------|
| `sharpness` | float | Laplacian variance of grayscale image | 0.0–1.0 (normalized) |
| `blur` | float | Inverse of sharpness (lower = sharper) | 0.0–1.0 |
| `exposure` | float | How well-exposed (0 = dark/blown out, 1 = good) | 0.0–1.0 |
| `occlusion` | float | Fraction of frame occluded | 0.0–1.0 |

### Pipeline example

```python
def compute_quality(frame) -> dict:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    sharpness = min(lap_var / 500.0, 1.0)  # normalize
    mean_brightness = gray.mean() / 255.0
    exposure = 1.0 - abs(mean_brightness - 0.45) * 2  # penalize too dark/bright
    return {"sharpness": round(sharpness, 3), "exposure": round(max(exposure, 0), 3)}

images.append(frame,
    pose=robot_pose,
    tags=compute_quality(frame),
)
```

Then in analysis:

```python
clusters = cluster_observations(candidates,
    time_scale=10.0,
    rank_key=lambda r: (
        r.scores.get("embedding", 0) * 0.5 +
        r.tags.get("sharpness", 0.5) * 0.3 +
        r.tags.get("exposure", 0.5) * 0.2
    ),
)
```

---

## The "Embed → Cluster → VLM" Pipeline

The dominant analysis pattern across Q2, Q4, Q5, Q8. Not a function — a recipe.

```
1. Embedding search     → candidate_k rows (cheap, fast, noisy)
2. Score filter          → discard low-similarity noise
3. cluster_observations  → group into distinct events/locations
4. Representative pick   → best frame per cluster (by quality + relevance)
5. VLM verify           → confirm/describe each representative (expensive, precise)
6. Expand               → confirmed representative → entire cluster is valid
```

This is NOT worth wrapping in a single function because:
- The VLM prompt varies per question
- The cluster parameters vary per domain
- The expand step varies (sometimes you want all rows, sometimes just the cluster metadata)
- Steps 1-4 compose naturally with existing tools

But documenting it as a pattern means every new question follows the same structure.

### Example: complete pipeline for Q2

```python
# 1. Embedding search
candidates = images.query().search_embedding(clip_text_encode("red socks"), candidate_k=1000).order_by("ts_start").fetch()

# 2. Score filter
candidates = [r for r in candidates if r.scores.get("embedding", 0) > 0.7]

# 3. Cluster (temporal — group continuous viewing)
clusters = cluster_observations(candidates,
    time_scale=10.0,
    rank_key=lambda r: (
        r.scores.get("embedding", 0) * 0.5 +
        r.tags.get("sharpness", 0.5) * 0.5
    ),
)

# 4. VLM verify representatives only
confirmed = []
for c in clusters:
    img = images.load(c.representative.ref)
    if vlm.ask(img, "Are there red socks in this image? yes/no") == "yes":
        confirmed.append(c)

# 5. Use results
print(f"Currently watching for {confirmed[-1].duration:.0f}s")
print(f"Seen {len(confirmed) - 1} time(s) before")
```

---

## Summary

| Utility | Pure Python | Used in | Core purpose |
|---------|-------------|---------|-------------|
| `cluster_observations` | yes | Q2,Q4,Q5,Q9,Q11,Q12,Q14 | Group by time/space, pick representative |
| `find_gaps` | yes | Q11 | Detect absence periods |
| `compute_path_distance` | yes | Q9,Q14 | Trajectory length |
| `extract_time_pattern` | yes | Q12 | Time-of-day statistics |
| `match_viewpoints` | yes | Q8 | Cross-temporal view matching |
| `diff_observation_sets` | yes | Q13 | Set difference by embedding similarity |

All utilities are stateless functions on `list[ObservationRow]`. No DB access, no numpy dependency (callers use numpy for batch vector ops if they want). Quality metrics live in tags, set by the ingest pipeline.

### Not included (stays in application code)

- **Identity clustering** (Q3, Q6, Q7): Requires DBSCAN/sklearn + domain-specific parameters. Too varied for a generic utility.
- **State transition detection** (Q5): "door went from closed→open" needs domain knowledge about what states exist.
- **Absence reasoning** (Q11): Distinguishing "cat not here" from "robot not looking" requires cross-referencing robot coverage — application context.
- **VLM prompting**: Every question has different prompts and response parsing.
