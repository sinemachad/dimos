# Answers — with Correlator

Side-by-side: how the cross-stream questions change with `s.correlate()`.
Only covering questions where correlator applies (Q1, Q5, Q6, Q7, Q10, Q14).

---

## Q1. "Where was I when this log line was added?"

**Before** (using `.at()`):
```python
log_hit = logs.query().search_text("motor fault detected").one()
pose_hit = poses.query().at(log_hit.ts_start, tolerance=0.5).one()
```

**With correlator**:
```python
log_set = logs.query().search_text("motor fault detected").fetch_set()
result = s.correlate(log_set, poses, time_tolerance=0.5)
pose = result.unambiguous()[0].matches[0].pose
```

**Verdict**: `.at()` is better here. Correlator adds ceremony for a single-observation lookup. Correlator wins when you have many anchors — e.g., "where was I for ALL error log lines?":

```python
errors = logs.query().search_text("error").fetch_set()
result = s.correlate(errors, poses, time_tolerance=0.5)
for p in result.with_matches():
    error_text = logs.load(p.anchor.ref).text
    print(f"{error_text} → {p.matches[0].pose}")
```

That replaces a loop of N `.at()` calls with one batch query.

---

## Q5. "Did anyone open this door? Who?"

**Before** (manual loop):
```python
door_events = events.query().filter_tags(type="door_open").order_by("ts_start").fetch()

for event in door_events:
    nearby = (faces.query()
        .filter_time(event.ts_start - 5.0, event.ts_start + 2.0)
        .filter_near(event.pose, radius=3.0)
        .fetch())
    if nearby:
        vec = faces.vector(nearby[0].ref)
        identity = lookup_identity(vec)
        print(f"Door opened at {event.ts_start} by {identity}")
```

**With correlator**:
```python
door_events = events.query().filter_tags(type="door_open").fetch_set()

pairs = s.correlate(door_events, faces,
                    time_before=5.0, time_after=2.0,
                    spatial_radius=3.0)

for p in pairs.with_matches():
    vec = faces.vector(p.matches[0].ref)
    identity = lookup_identity(vec)
    print(f"Door opened at {p.anchor.ts_start} by {identity}")

# Bonus: which door openings had nobody nearby?
for anchor in pairs.unmatched():
    print(f"Door opened at {anchor.ts_start} — nobody detected")
```

**What changed**:
- Loop of N queries → 1 batch query
- `.unmatched()` is free — no extra work to find events with zero matches
- Asymmetric window (`time_before=5.0, time_after=2.0`) expresses "who was there just before and shortly after" naturally

---

## Q6. "STT + voice embeddings — who is saying what?"

**Before** (manual loop):
```python
for tx_row in transcripts.query().order_by("ts_start").fetch():
    voice = (voice_embs.query()
        .filter_time(tx_row.ts_start, tx_row.ts_end)
        .one())

    speaker_vec = voice_embs.vector(voice.ref)
    transcript_text = transcripts.load(tx_row.ref).text
    print(f"[{speaker_vec_to_name(speaker_vec)}]: {transcript_text}")
```

**With correlator**:
```python
pairs = s.correlate(transcripts, voice_embs, time_tolerance=0.0)

for p in pairs.with_matches():
    speaker_vec = voice_embs.vector(p.matches[0].ref)
    transcript_text = transcripts.load(p.anchor.ref).text
    print(f"[{speaker_vec_to_name(speaker_vec)}]: {transcript_text}")

# Transcripts with no matching voice segment (e.g., gap in audio)
for anchor in pairs.unmatched():
    print(f"[unknown]: {transcripts.load(anchor.ref).text}")
```

**What changed**:
- `time_tolerance=0.0` means: target's `ts_start` must fall within anchor's `[ts_start, ts_end]` window. Since transcripts have both `ts_start`/`ts_end`, this matches voice segments that overlap with the spoken words.
- `.unmatched()` catches transcripts where audio processing failed or had gaps — previously silently lost in a `.one()` that would throw.

---

## Q7. "Voice ↔ face correlation (partial overlap)"

**Before** (manual loop + filtering):
```python
pairings = []
for v_row in voices.query().order_by("ts_start").fetch():
    visible_faces = (faces.query()
        .filter_time(v_row.ts_start, v_row.ts_end)
        .fetch())

    if len(visible_faces) == 1:
        voice_vec = voices.vector(v_row.ref)
        face_vec = faces.vector(visible_faces[0].ref)
        pairings.append((voice_vec, face_vec))
```

**With correlator**:
```python
pairs = s.correlate(voices, faces, time_tolerance=0.0)

# Unambiguous pairings: exactly one face visible during voice segment
pairings = []
for p in pairs.unambiguous():
    voice_vec = voices.vector(p.anchor.ref)
    face_vec = faces.vector(p.matches[0].ref)
    pairings.append((voice_vec, face_vec))

# Stats for free
total = len(pairs)
matched = len(pairs.with_matches())
unambiguous = len(pairs.unambiguous())
unmatched = len(pairs.unmatched())
print(f"{total} voice segments: {unambiguous} unambiguous, "
      f"{matched - unambiguous} ambiguous, {unmatched} no face visible")
```

**What changed**:
- `.unambiguous()` replaces the `if len(...) == 1` check
- Statistics about match quality are trivial to compute
- The "I don't see all people speaking at all times" constraint is directly visible in `.unmatched()` count

---

## Q10. "What happened in the 30 seconds before the vase fell?"

**Before** (loop over streams):
```python
vase_event = events.query().search_text("vase fell").one()
t = vase_event.ts_start

timeline = {}
for info in s.list_streams():
    stream = s.stream(info.name, info.payload_type,
                      embedding=info.embedding, text=info.text)
    window = (stream.query()
        .filter_time(t - 30.0, t)
        .order_by("ts_start")
        .fetch())
    timeline[info.name] = window
```

**With correlator**:
```python
vase_set = events.query().search_text("vase fell").fetch_set()

timeline = {}
for info in s.list_streams():
    stream = s.stream(info.name, info.payload_type,
                      embedding=info.embedding, text=info.text)
    result = s.correlate(vase_set, stream, time_before=30.0, time_after=0.0)
    timeline[info.name] = result
```

**What changed**:
- `time_before=30.0, time_after=0.0` — asymmetric window expresses "30s before, nothing after" directly. No manual `t - 30.0, t` arithmetic.
- Still loops over streams (correlator is pairwise). But each iteration is cleaner.
- If `vase_set` had multiple events (vase fell twice), you'd get per-event windows for free. The manual version would need a nested loop.

**Honest assessment**: Marginal improvement for Q10 since the anchor is typically one event. The correlator shines more when you have many anchors.

---

## Q14. "How far did I travel while carrying an object?"

**Before** (segment + loop):
```python
# Step 1: Segment carrying detections into intervals
carrying = (detections.query()
    .filter_tags(action="carrying")
    .order_by("ts_start")
    .fetch())

intervals = []
seg_start = carrying[0].ts_start
prev_t = carrying[0].ts_start
for r in carrying[1:]:
    if r.ts_start - prev_t > 2.0:
        intervals.append((seg_start, prev_t))
        seg_start = r.ts_start
    prev_t = r.ts_start
intervals.append((seg_start, prev_t))

# Step 2: For each interval, get poses and sum distance
total_distance = 0.0
for t_start, t_end in intervals:
    pose_rows = (poses.query()
        .filter_time(t_start, t_end)
        .order_by("ts_start")
        .fetch())
    for i in range(1, len(pose_rows)):
        total_distance += distance(pose_rows[i-1].pose, pose_rows[i].pose)
```

**With correlator**:
```python
carrying = detections.query().filter_tags(action="carrying").fetch_set()

pairs = s.correlate(carrying, poses, time_tolerance=0.1)

# Each carrying detection gets matched to nearby poses
# Deduplicate: collect all unique matched pose refs, sorted by time
seen_pose_refs = set()
all_poses = []
for p in pairs:
    for m in p.matches:
        if m.ref.id not in seen_pose_refs:
            seen_pose_refs.add(m.ref.id)
            all_poses.append(m)

all_poses.sort(key=lambda r: r.ts_start)

total_distance = 0.0
for i in range(1, len(all_poses)):
    total_distance += distance(all_poses[i-1].pose, all_poses[i].pose)
```

**Honest assessment**: The correlator version is *not* cleaner here. The problem is that carrying detections are per-frame (one every 0.2s at 5Hz), so you get thousands of overlapping CorrelationPairs that all match the same poses. You need deduplication, which is awkward.

The original approach is actually better: segment into intervals first (app logic), then do one time-range query per interval. Correlator is designed for "match discrete events to another stream", not "define continuous intervals and query within them."

**When correlator WOULD help for Q14**: If carrying detections had `ts_start`/`ts_end` representing the full carry interval (not per-frame), then:

```python
# If each carrying observation spans the full interval
carry_intervals = detections.query().filter_tags(action="carrying").fetch_set()

pairs = s.correlate(carry_intervals, poses, time_tolerance=0.0)
total_distance = 0.0
for p in pairs:
    sorted_poses = sorted(p.matches, key=lambda r: r.ts_start)
    for i in range(1, len(sorted_poses)):
        total_distance += distance(sorted_poses[i-1].pose, sorted_poses[i].pose)
```

Clean — but requires interval-shaped observations. The segmentation from point detections to intervals is still app logic.

---

## Summary

| Q | Before | With Correlator | Improvement |
|---|--------|----------------|-------------|
| Q1 | `.at()` — 1 query | overkill for single lookup | None (`.at()` is better) |
| Q1 batch | N `.at()` calls | 1 batch query | Yes — N→1 queries |
| Q5 | N queries in loop | 1 batch + `.with_matches()` / `.unmatched()` | Yes — cleaner + free stats |
| Q6 | N queries in loop | 1 batch + `.unmatched()` catches gaps | Yes — cleaner + error visibility |
| Q7 | N queries + `if len==1` | 1 batch + `.unambiguous()` | Yes — most natural fit |
| Q10 | N queries (1 per stream) | N correlate calls, asymmetric window | Marginal — still loops over streams |
| Q14 | segment + N queries | messy dedup of overlapping pairs | No — manual approach is better for continuous intervals |

**Key insight**: Correlator is best for **discrete events correlated against another stream** (Q5, Q6, Q7). It's less useful for continuous intervals (Q14) or single-observation lookups (Q1). The sweet spot is "I have 50-5000 anchors and want matches from another stream for each."

**API validated**: `time_before`/`time_after` asymmetric windows are needed (Q5, Q10). `.unambiguous()` and `.unmatched()` are the most-used convenience methods.
