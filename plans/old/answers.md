# Answers

API reference: `memory3.md` (current)

---

## 1. "Where was I, when this log line was added?" + "Where do motor faults keep happening?"

**Streams**: `logs` (text-capable), `poses` (robot localization at high frequency)

**Single log line**:

```python
s = db.session()
logs = s.stream("logs", LogMsg, text=TextConfig())
poses = s.stream("poses", PoseStamped)

# Find the log entry by text
log_hit = logs.query().search_text("motor fault detected").one()

# Look up pose at that time — .at() finds nearest within tolerance
pose_hit = poses.query().at(log_hit.ts_start, tolerance=0.5).one()
print(pose_hit.pose)  # Pose(x=1.2, y=3.4, z=0.5)
```

**Multiple log lines → spatial map of faults**:

```python
fault_logs = logs.query().search_text("motor fault").order_by("ts_start").fetch()

# Correlate each to a pose
fault_locations = []
for log_row in fault_logs:
    pose_row = poses.query().at(log_row.ts_start, tolerance=0.5).fetch()
    if pose_row:
        fault_locations.append((log_row, pose_row[0]))

# Cluster by location — "where do faults keep happening?"
from dimos.memory2.analysis import cluster_observations
location_clusters = cluster_observations(
    [pose for _, pose in fault_locations],
    space_scale=2.0,  # within 2m = same spot
)

for c in location_clusters:
    print(f"{len(c.rows)} faults near {c.center_pose} "
          f"({c.t_start} to {c.t_end})")
    # → "12 faults near Pose(x=3.1, y=7.2) over the last 3 days"

# Render on costmap
for c in location_clusters:
    costmap.mark(pose=c.center_pose, label=f"motor faults ({len(c.rows)}x)")
```

**What works**: `.search_text()` finds all matching logs, `.at()` correlates each to a pose, `cluster_observations(space_scale=)` groups faults by location. The result is a heatmap of where the robot has trouble.

**Cross-stream join**: The for-loop is the same nested-loop join pattern as Q5/Q7/Q14. `Correlator` (Phase 3) would batch this:
```python
fault_poses = s.correlate(fault_logs_set, poses, time_tolerance=0.5)
```

---

## 2. "How long have I been observing the red socks in view currently?" + "How many times did I see them before?"

**Streams**: `images` (camera frames with CLIP embeddings and poses)

No detection pipeline — we search raw images by embedding similarity, then VLM-verify.

**Stage 1 — Embedding candidate retrieval**:

```python
s = db.session()
images = s.stream("images", Image, embedding=EmbeddingConfig(dim=512))

socks_embedding = clip_text_encode("red socks")

# Find all frames that might contain red socks
candidates = (images.query()
    .search_embedding(socks_embedding, candidate_k=1000)
    .order_by("ts_start")
    .fetch())

# Post-filter by similarity score to discard weak matches
candidates = [h for h in candidates if h.scores.get("embedding", 0) > 0.7]
```

**Stage 2 — Diverse sampling before VLM** (don't waste VLM on 200 frames of the robot staring at socks):

The embedding top-k will cluster heavily around moments of prolonged viewing. We need to spread VLM budget across time and space to discover all distinct sighting sessions.

```python
# Cluster candidates into temporal segments (frames within 10s = same cluster)
# Then pick one representative per cluster for VLM
candidates.sort(key=lambda r: r.ts_start)

clusters = []  # list of lists
for row in candidates:
    if not clusters or row.ts_start - clusters[-1][-1].ts_start > 10.0:
        clusters.append([row])
    else:
        clusters[-1].append(row)

# Pick the highest-scoring representative from each cluster
representatives = []
for cluster in clusters:
    best = max(cluster, key=lambda r: r.scores.get("embedding", 0))
    representatives.append((best, cluster))
```

Now VLM verifies only the representatives — one call per temporal cluster, not per frame:

```python
confirmed_segments = []
for rep, cluster in representatives:
    img = images.load(rep.ref)
    if vlm.ask(img, "Are there red socks visible in this image? yes/no") == "yes":
        # Entire cluster counts as a sighting session
        confirmed_segments.append((cluster[0].ts_start, cluster[-1].ts_start))
```

If the robot saw socks 5 different times across the day but stared for minutes each time, this makes ~5 VLM calls instead of 200+.

**Stage 3 — Answer the question**:

```python
now = time.time()

# Current viewing session = last confirmed segment
if confirmed_segments:
    current_duration = now - confirmed_segments[-1][0]
    print(f"Watching red socks for {current_duration:.1f}s")
    print(f"Seen them {len(confirmed_segments) - 1} time(s) before")
```

**What works**: Embedding search is the broad net (cheap, fast), temporal clustering deduplicates the "staring" problem, VLM confirms only one frame per cluster. Scales to long sessions without blowing VLM budget.

**What's application logic**: Cluster gap threshold (10s), VLM prompt, what counts as "same sighting" — all domain-specific.

**Limitation**: `candidate_k=1000` is a guess. sqlite-vec is KNN-only — no "all vectors above threshold" query. Workaround: use a large candidate_k and post-filter by score.

**Extension — spatial diversity**: If the robot revisits the same spot repeatedly, add pose-based deduplication within temporal clusters. But temporal clustering alone handles the dominant case (continuous staring).

---

## 3. "How many people did I see during last week?"

**Pipeline**:
```
camera frames → face detector → face crops → embedding model → face embeddings
                                    ↓
                    faces stream (each row = one detected face with identity embedding)
```

Yes — the `faces` stream stores detected face crops. Each append includes the face embedding. Searching over this stream by embedding finds the same face across time.

**Streams**: `faces` (face crops with identity embeddings)

```python
s = db.session()
faces = s.stream("faces", FaceCrop, embedding=EmbeddingConfig(dim=512))

one_week_ago = time.time() - 7 * 86400
week_faces = (faces.query()
    .filter_after(one_week_ago)
    .order_by("ts_start")
    .fetch())

# Get all embedding vectors for clustering
vectors = []
for row in week_faces:
    vec = faces.vector(row.ref)  # retrieve stored embedding
    vectors.append(vec)

# Cluster to find unique identities
import numpy as np
from sklearn.cluster import DBSCAN

X = np.array(vectors)
clustering = DBSCAN(eps=0.6, min_samples=2, metric="cosine").fit(X)
n_people = len(set(clustering.labels_)) - (1 if -1 in clustering.labels_ else 0)
print(f"Saw {n_people} unique people last week")
```

**What works**: `filter_after` for time range, `faces.vector(ref)` to retrieve stored embeddings for clustering.

**What's application logic**: Identity clustering (DBSCAN, threshold tuning) is domain-specific — different robots may have different accuracy needs.

**With derive() (Phase 3)**: Could automate the dedup into a persistent `people` stream, then it's just `.count()`.

---

## 4. "Where did you see red socks during last week?"

**Streams**: `images` (camera frames with CLIP embeddings and poses)

**Stage 1 — Embedding candidate retrieval**:

```python
s = db.session()
images = s.stream("images", Image, embedding=EmbeddingConfig(dim=512))

one_week_ago = time.time() - 7 * 86400
socks_embedding = clip_text_encode("red socks")

candidates = (images.query()
    .search_embedding(socks_embedding, candidate_k=200)
    .filter_after(one_week_ago)
    .limit(50)
    .fetch_set())
```

**Stage 2 — VLM verification**:

```python
verified_refs = []
for row in candidates.rows():
    img = candidates.load(row.ref)
    if vlm.ask(img, "Are there red socks in this image? yes/no") == "yes":
        verified_refs.append(row.ref)

# Wrap verified results back into an ObservationSet
verified = images.query().filter_refs(verified_refs).fetch_set()
```

`filter_refs()` gives us an ObservationSet of just the verified images — ephemeral, session-scoped.

To persist: write to a new stream with lineage back to the originals:

```python
red_socks = s.stream("red_socks", Image)
for ref in verified_refs:
    src = images.meta(ref)
    red_socks.append(
        images.load(ref),
        pose=src.pose, ts_start=src.ts_start,
        tags={"query": "red socks"},
        parent_stream="images", parent_id=ref.id,
    )
```

**Stage 3 — Costmap**:

```python
for row in verified.rows():
    costmap.mark(pose=row.pose, label="red socks", time=row.ts_start)
```

Every verified observation carries the robot's pose from the original image stream → direct costmap placement.

---

## 5. "Did anyone ever open this door? At what times? Who opened it?"

**Streams**: `detections` (object detections with tags), `faces` (face crops with identity embeddings)

**Sub-question 1 & 2 — When was the door open?**

Depends on the detection pipeline. If the detector tags door state:

```python
detections = s.stream("detections", Detection, embedding=EmbeddingConfig(dim=512))

door_open = (detections.query()
    .filter_tags(class_name="door", state="open")
    .order_by("ts_start")
    .fetch())

for row in door_open:
    print(f"Door open at {row.ts_start}")
```

If the detector doesn't tag state — embedding search + VLM verify (same pattern as Q4):

```python
open_door_emb = clip_text_encode("open door")
candidates = (images.query()
    .search_embedding(open_door_emb, candidate_k=100)
    .filter_near(door_location, radius=3.0)  # only images near the door
    .fetch())

# VLM verify each candidate
open_times = [r for r in candidates
              if vlm.ask(images.load(r.ref), "Is this door open?") == "yes"]
```

**Sub-question 3 — Who opened it?**

Cross-stream temporal+spatial correlation: for each door-open event, find faces nearby at that time.

```python
faces = s.stream("faces", FaceCrop, embedding=EmbeddingConfig(dim=512))

for event in open_times:
    # Find faces near the door around the time it opened
    nearby = (faces.query()
        .filter_time(event.ts_start - 5.0, event.ts_start + 2.0)
        .filter_near(event.pose, radius=3.0)
        .fetch())

    if nearby:
        # Identify the person via face embedding
        vec = faces.vector(nearby[0].ref)
        identity = lookup_identity(vec)  # match against known faces DB
        print(f"Door opened at {event.ts_start} by {identity}")
```

**What works**: `filter_time` + `filter_near` compose naturally for "who was here when this happened". The R*Tree + ts_start index handle this efficiently.

**What's manual**: The for-loop is a nested-loop join. `Correlator` (Phase 3) would batch this:
```python
# Phase 3:
s.correlate(door_events, faces, time_tolerance=5.0, spatial_radius=3.0)
```

**State transition detection** ("door went from closed→open") is application logic. The memory system stores observations, not state machines. You'd either store explicit events in a `door_events` stream, or detect transitions by comparing consecutive detections.

---

## 6. "I have a transcription log (STT) and voice embeddings — how do I figure out who is saying what?"

**Streams**: `transcripts` (STT output, text-capable), `voice_embeddings` (speaker embeddings per audio segment)

Two separate streams because they come from different models: STT gives you text, a speaker encoder gives you a voice identity vector.

```python
s = db.session()
transcripts = s.stream("transcripts", Transcript, text=TextConfig())
voice_embs = s.stream("voice_segments", VoiceSegment, embedding=EmbeddingConfig(dim=192))
```

**Step 1 — Align transcripts to voice segments by time**:

Each transcript has `ts_start`/`ts_end` (when the words were spoken). Each voice segment has a speaker embedding for that time window.

```python
for tx_row in transcripts.query().order_by("ts_start").fetch():
    # Find the voice segment that overlaps this transcript
    voice = (voice_embs.query()
        .filter_time(tx_row.ts_start, tx_row.ts_end)
        .one())

    # voice.ref → voice_embs.vector(voice.ref) gives us the speaker embedding
    speaker_vec = voice_embs.vector(voice.ref)
    transcript_text = transcripts.load(tx_row.ref).text

    print(f"[{speaker_vec_to_name(speaker_vec)}]: {transcript_text}")
```

**Step 2 — Build speaker identity mapping**:

Cluster all voice embeddings to find distinct speakers, then label:

```python
all_voices = voice_embs.query().order_by("ts_start").fetch()
vectors = [voice_embs.vector(r.ref) for r in all_voices]

# Cluster into distinct speakers
clustering = DBSCAN(eps=0.3, min_samples=3, metric="cosine").fit(np.array(vectors))
# label_id → speaker name mapping (manual or via face correlation — see Q7)
```

**What works**: `filter_time` on voice stream using transcript's time window is the natural join key. `.vector()` retrieves stored embeddings for clustering.

**Key insight**: The two streams are aligned by time, not by embedding similarity. We don't search by embedding across streams — we use temporal co-occurrence to pair them, then use the voice embedding for speaker identity.

---

## 7. "I have parallel voice and facial recognition streams — how do I correlate voice to people? (I don't see all people speaking at all times)"

**Streams**: `voices` (speaker embeddings per audio segment), `faces` (face identity embeddings per detection)

The constraint "I don't see all people speaking at all times" means:
- Sometimes a person is speaking but out of camera view → voice segment exists, no face match
- Sometimes multiple people are visible but only one is speaking
- The correlation must be probabilistic, accumulated over time

```python
s = db.session()
voices = s.stream("voices", VoiceSegment, embedding=EmbeddingConfig(dim=192))
faces = s.stream("faces", FaceCrop, embedding=EmbeddingConfig(dim=512))
```

**Step 1 — Collect unambiguous pairings** (only one face visible while voice active):

```python
pairings = []  # (voice_embedding, face_embedding) pairs

for v_row in voices.query().order_by("ts_start").fetch():
    # Find faces visible during this voice segment
    visible_faces = (faces.query()
        .filter_time(v_row.ts_start, v_row.ts_end)
        .fetch())

    if len(visible_faces) == 1:
        # Unambiguous: only one person visible → must be the speaker
        voice_vec = voices.vector(v_row.ref)
        face_vec = faces.vector(visible_faces[0].ref)
        pairings.append((voice_vec, face_vec))
```

**Step 2 — Build cross-modal identity mapping**:

```python
# Cluster voice embeddings → speaker IDs
voice_vecs = np.array([p[0] for p in pairings])
voice_clusters = DBSCAN(eps=0.3, min_samples=2, metric="cosine").fit(voice_vecs)

# For each voice cluster, find the most common face cluster
# This gives us: voice_speaker_id → face_identity
speaker_to_face = {}
for cluster_id in set(voice_clusters.labels_):
    if cluster_id == -1:
        continue
    cluster_face_vecs = [p[1] for i, p in enumerate(pairings)
                         if voice_clusters.labels_[i] == cluster_id]
    # Majority vote on face identity
    face_identity = identify_majority(cluster_face_vecs)
    speaker_to_face[cluster_id] = face_identity
```

**Step 3 — Label all voice segments** (including ambiguous ones):

```python
for v_row in voices.query().order_by("ts_start").fetch():
    voice_vec = voices.vector(v_row.ref)
    # Find nearest voice cluster → mapped face identity
    speaker_id = predict_cluster(voice_vec, voice_clusters)
    person = speaker_to_face.get(speaker_id, "unknown")
    print(f"[{person}] spoke at {v_row.ts_start}")
```

**What works**:
- `filter_time` on faces using voice segment's time window — natural temporal join
- `.vector()` on both streams for cross-modal clustering
- The API provides the building blocks; the correlation logic (accumulate unambiguous pairings → build mapping → apply to ambiguous cases) is correctly application-level

**What the constraint exposes**: "I don't see all people speaking at all times" means we can't rely on a single observation to establish identity. We need statistical accumulation — many unambiguous pairings build confidence. This is fundamentally a learning problem, not a query problem. The memory system's job is to make the data accessible; the correlation intelligence lives above.

**With Correlator (Phase 3)**: The inner loop (for each voice segment, query faces) would become:
```python
pairs = s.correlate(voices, faces, time_tolerance=0.5)
```
But the clustering/identity-mapping step still lives in application code.

---

## 8. "What's different in this room compared to yesterday?"

**What we need**: Compare object detections from "now" vs "yesterday" at the same location, find what changed.

**Streams**: `images` (camera frames with CLIP embeddings and poses)

We can't rely on a precomputed detection stream — object detection for a fixed set is expensive and not run in realtime. Instead, store raw images and diff at query time using embeddings + VLM.

```python
s = db.session()
images = s.stream("images", Image, embedding=EmbeddingConfig(dim=512))

now = time.time()
yesterday = now - 86400
robot_pose = get_current_pose()

# Two queries: images from this room now vs yesterday
current_imgs = (images.query()
    .filter_time(now - 300, now)
    .filter_near(robot_pose, radius=5.0)
    .fetch())

yesterday_imgs = (images.query()
    .filter_time(yesterday - 300, yesterday + 300)
    .filter_near(robot_pose, radius=5.0)
    .fetch())

# Match viewpoints by embedding similarity (numpy, no extra queries)
current_vecs = np.array([images.vector(r.ref) for r in current_imgs])
yesterday_vecs = np.array([images.vector(r.ref) for r in yesterday_imgs])
similarity = current_vecs @ yesterday_vecs.T

# Pair each current image with its closest yesterday viewpoint
pairs = []
for i, row in enumerate(current_imgs):
    j = similarity[i].argmax()
    if similarity[i, j] > 0.85:  # same viewpoint
        pairs.append((row, yesterday_imgs[j]))

# VLM diffs only matched viewpoint pairs
for curr, yest in pairs:
    diff = vlm.ask(
        [images.load(curr.ref), images.load(yest.ref)],
        "What changed between these two views?")
    if diff != "nothing":
        print(f"At {curr.pose}: {diff}")
```

**What works**: Two queries retrieve the two temporal snapshots scoped to this room. Embedding similarity in numpy matches viewpoints without extra DB queries. VLM provides open-vocabulary scene comparison — no fixed object set needed.

**What's application logic**: Viewpoint matching threshold, VLM prompting, what counts as a meaningful change. The memory system provides spatial+temporal retrieval; the VLM provides the intelligence.

**Cost structure**: 2 DB queries + N `.vector()` reads (small, fast) + numpy matmul + M VLM calls (expensive, but only on matched pairs).

---

## 9. "Show me everywhere the cat went today"

**What we need**: Retrieve all cat detections from today, extract the pose trail, render as a path on the costmap.

**Streams**: `detections` (object detections with poses)

```python
s = db.session()
detections = s.stream("detections", Detection, embedding=EmbeddingConfig(dim=512))

today_start = start_of_day()

# All cat detections today, ordered by time
cat_trail = (detections.query()
    .filter_tags(class_name="cat")
    .filter_after(today_start)
    .order_by("ts_start")
    .fetch())

# Extract the pose path
path = [(row.ts_start, row.pose) for row in cat_trail if row.pose]

# Render on costmap
for ts, pose in path:
    costmap.add_point(pose=pose, time=ts, label="cat")
costmap.draw_path([pose for _, pose in path])
```

**What works**: `filter_tags(class_name="cat")` + `filter_after()` + `order_by("ts_start")` is clean and direct. Every detection carries the robot's pose → we know where the robot saw the cat, which approximates where the cat was.

**Subtlety**: `row.pose` is the *robot's* pose when it detected the cat, not the cat's position in world frame. If you need the cat's actual position, you'd need the detection bounding box + depth + robot pose to project into world coordinates. That projection would happen in the detection pipeline before appending to the stream:

```python
# In the detection pipeline:
cat_world_pose = project_to_world(bbox, depth_frame, robot_pose)
detections.append(detection, pose=cat_world_pose, tags={"class_name": "cat"})
```

If stored this way, `row.pose` *is* the cat's world position, and the path is accurate.

**Dense vs sparse**: If the detector runs at 5Hz and the cat is visible for an hour, that's 18,000 rows. `order_by("ts_start")` + the ts_start index handles this efficiently. For rendering, you might want to downsample:

```python
# Fetch pages to avoid loading all 18k rows at once
for page in range(0, cat_trail_count, 100):
    rows = (detections.query()
        .filter_tags(class_name="cat")
        .filter_after(today_start)
        .order_by("ts_start")
        .limit(100)  # TODO: need offset on limit, or use fetch_set + fetch_page
        .fetch())
```

**Gap exposed**: `Query.limit(k)` has no `offset`. For pagination, you'd need `fetch_set()` then `fetch_page(limit=100, offset=N)`. This works but means you can't paginate purely at the query level.

---

## 10. "What happened in the 30 seconds before the vase fell?"

**What we need**: Detect the "vase fell" event, then slice ALL streams in a 30s window before it.

**Streams**: `events` (detected events with tags), plus any number of other streams: `images`, `audio`, `detections`, `poses`, etc.

```python
s = db.session()
events = s.stream("events", Event, text=TextConfig())

# Find the vase-fall event
vase_event = events.query().search_text("vase fell").one()
t_event = vase_event.ts_start

# Now query every stream for the 30s window before the event
# list_streams() returns StreamInfo with payload_type, configs, count
timeline = {}
for info in s.list_streams():
    stream = s.stream(info.name, info.payload_type,
                      embedding=info.embedding, text=info.text)
    window = (stream.query()
        .filter_time(t_event - 30.0, t_event)
        .order_by("ts_start")
        .fetch())
    timeline[info.name] = window
```

**What works**: `list_streams()` returns `StreamInfo` with everything needed to reconstruct stream handles — no hardcoding payload types. `filter_time(t - 30, t)` on each stream gives the pre-event window.

---

## 11. "When was the last time I did NOT see the cat in the apartment?"

**What we need**: Find gaps in the cat detection stream — periods where no cat was detected.

**Streams**: `detections` (object detections with tags)

```python
s = db.session()
detections = s.stream("detections", Detection, embedding=EmbeddingConfig(dim=512))

# Get all cat detections, ordered by time
cat_seen = (detections.query()
    .filter_tags(class_name="cat")
    .order_by("ts_start")
    .fetch())

# Find gaps — periods where the cat wasn't detected
# Gap = time between consecutive cat detections longer than some threshold
gap_threshold = 60.0  # 1 minute without seeing the cat = "not seen"

timestamps = [r.ts_start for r in cat_seen]
gaps = []
for i in range(1, len(timestamps)):
    gap = timestamps[i] - timestamps[i - 1]
    if gap > gap_threshold:
        gaps.append((timestamps[i - 1], timestamps[i], gap))

if gaps:
    # Most recent gap = last time the cat wasn't seen
    last_gap = gaps[-1]
    print(f"Last not seen: {last_gap[0]} to {last_gap[1]} ({last_gap[2]:.0f}s)")
else:
    print("Cat has been visible continuously")
```

**What works**: `filter_tags` + `order_by` gives us the detection timeline. Gap analysis in Python is straightforward.

**What the API can't do natively**: Negation queries ("when did X NOT happen") aren't expressible in the query builder. You can only query for what exists, then find gaps in Python. This is fundamentally correct — the memory system stores positive observations, not the absence of observations. Detecting absence requires knowledge of when the sensor *could* have observed (was the robot even in the apartment? was the camera on?) — that's application context.

**Edge case**: The robot wasn't always in the apartment. A "gap" might be because the robot was in another room, not because the cat wasn't there. You'd need to cross-reference with the robot's own position to distinguish "didn't see cat because cat was absent" from "didn't see cat because robot was elsewhere."

---

## 12. "What time does the mailman usually come?"

**What we need**: We don't know who the mailman is. We need to discover them first, then find all their appearances, then extract the schedule.

**Streams**: `images` (camera frames with CLIP embeddings and poses), `faces` (face crops with identity embeddings)

**Stage 1 — Find the mailman via VLM** (retroactive identification):

We know the mailman comes to the front door. Use spatial + embedding search to find candidates, VLM to confirm.

```python
s = db.session()
images = s.stream("images", Image, embedding=EmbeddingConfig(dim=512))
faces = s.stream("faces", FaceCrop, embedding=EmbeddingConfig(dim=512))

mailman_emb = clip_text_encode("person delivering mail at front door")

# Search images near the front door
candidates = (images.query()
    .search_embedding(mailman_emb, candidate_k=200)
    .filter_near(front_door_pose, radius=5.0)
    .fetch())

# Cluster temporally (don't VLM 200 frames of same delivery)
clusters = cluster_observations(candidates, time_scale=60.0)

# VLM verify representatives
mailman_times = []
for c in clusters:
    img = images.load(c.representative.ref)
    if vlm.ask(img, "Is there a person delivering mail or packages? yes/no") == "yes":
        mailman_times.append(c)
```

**Stage 2 — Extract mailman embedding** (from confirmed sightings):

Now we know *when* the mailman was there. Find their face embedding from those time windows.

```python
# For each confirmed mailman visit, find faces near the door at that time
mailman_face_vecs = []
for c in mailman_times:
    nearby_faces = (faces.query()
        .filter_time(c.t_start - 5.0, c.t_end + 5.0)
        .filter_near(front_door_pose, radius=3.0)
        .fetch())
    for f in nearby_faces:
        mailman_face_vecs.append(faces.vector(f.ref))

# Average the face embeddings → stable mailman identity vector
import numpy as np
mailman_identity = np.mean(mailman_face_vecs, axis=0).tolist()
```

**Stage 3 — Search broadly with the discovered embedding**:

Now we have a face embedding. Search ALL face data, not just near the door — catches sightings we might have missed with the spatial filter.

```python
all_sightings = (faces.query()
    .search_embedding(mailman_identity, candidate_k=200)
    .fetch())
sightings = [r for r in all_sightings if r.scores.get("embedding", 0) > 0.8]

# Cluster into individual visits
visits = cluster_observations(sightings, time_scale=300.0)
```

**Stage 4 — Extract schedule**:

```python
pattern = extract_time_pattern([v.representative for v in visits])
print(f"Mailman comes at {pattern}")  # "10:30 +/- 12min (n=23)"
```

**The general pattern — retroactive identification**:
1. **Describe** → CLIP text embedding + spatial constraint to narrow candidates
2. **VLM confirm** → identify positive examples (expensive, but on clustered representatives only)
3. **Extract identity embedding** → from confirmed examples, average face/object embeddings
4. **Search broadly** → use discovered embedding to find all appearances across time
5. **Analyze** → cluster, extract patterns

This is the inverse of the usual flow (have embedding → search). Here you don't know what you're looking for until you find it via VLM, then bootstrap an embedding for broader retrieval.

**Cross-session note**: This only works if the DB persists across days (`retention` != `"run"`). For long-term pattern analysis, use a persistent retention policy.

---

## 13. "What did robot-2 observe in the warehouse that I missed?"

**What we need**: Compare observations between two robots at the same location, find what robot-2 saw that robot-1 (me) didn't.

**Streams**: Both robots write to the same DB (or DBs are merged). Observations carry `robot_id` in tags.

```python
s = db.session()
detections = s.stream("detections", Detection, embedding=EmbeddingConfig(dim=512))

# What robot-2 saw in the warehouse
robot2_saw = (detections.query()
    .filter_tags(robot_id="robot-2")
    .filter_near(warehouse_pose, radius=20.0)  # warehouse area
    .fetch())

# What I saw in the same area
my_saw = (detections.query()
    .filter_tags(robot_id="robot-1")
    .filter_near(warehouse_pose, radius=20.0)
    .fetch())

# Diff: find objects robot-2 detected that I didn't
# By embedding — for each of robot-2's detections, check if I have a similar one
my_vecs = [detections.vector(r.ref) for r in my_saw]

missed = []
for r2_row in robot2_saw:
    r2_vec = detections.vector(r2_row.ref)
    # Check if any of my detections are similar
    similarities = [cosine_sim(r2_vec, mv) for mv in my_vecs]
    if not similarities or max(similarities) < 0.7:
        missed.append(r2_row)

print(f"Robot-2 saw {len(missed)} things you missed:")
for m in missed:
    print(f"  {m.tags.get('class_name')} at {m.pose}")
```

**What works**: `filter_tags(robot_id=...)` scopes to a specific robot — `robot_id` lives in the tags JSON, queried via `filter_tags`. `filter_near` scopes to a location. `.vector()` enables cross-robot embedding comparison. No special `filter_robot()` needed.

---

## 14. "How far did I travel while carrying an object?"

**What we need**: Compute path distance from the pose stream, but only during time intervals when a parallel detection stream shows "carrying object."

**Streams**: `poses` (robot poses at high frequency), `detections` (with "carrying" state)

```python
s = db.session()
poses = s.stream("poses", PoseStamped)
detections = s.stream("detections", Detection, embedding=EmbeddingConfig(dim=512))

# Step 1: Find all time intervals where the robot was carrying an object
carrying = (detections.query()
    .filter_tags(action="carrying")
    .order_by("ts_start")
    .fetch())

# Segment into continuous carrying intervals
intervals = []
if carrying:
    seg_start = carrying[0].ts_start
    prev_t = carrying[0].ts_start
    for r in carrying[1:]:
        if r.ts_start - prev_t > 2.0:  # gap = stopped carrying
            intervals.append((seg_start, prev_t))
            seg_start = r.ts_start
        prev_t = r.ts_start
    intervals.append((seg_start, prev_t))

# Step 2: For each carrying interval, get poses and compute distance
import math
total_distance = 0.0

for t_start, t_end in intervals:
    pose_rows = (poses.query()
        .filter_time(t_start, t_end)
        .order_by("ts_start")
        .fetch())

    for i in range(1, len(pose_rows)):
        p1 = pose_rows[i - 1].pose
        p2 = pose_rows[i].pose
        dx = p2.position.x - p1.position.x
        dy = p2.position.y - p1.position.y
        dz = p2.position.z - p1.position.z
        total_distance += math.sqrt(dx * dx + dy * dy + dz * dz)

print(f"Traveled {total_distance:.2f}m while carrying objects")
```

**What works**: `filter_tags` identifies "carrying" intervals. `filter_time` + `order_by` retrieves the pose trail for each interval. Distance computation is simple Euclidean accumulation.

**This is the cross-stream conditional join pattern**: Query stream A (detections) for intervals, then query stream B (poses) within those intervals. Same nested-loop pattern as Q5/Q6/Q7.

**What would be cleaner with Correlator (Phase 3)**:
```python
# Get pose observations that overlap with carrying detections
carrying_poses = s.correlate(carrying_set, poses, time_tolerance=0.0)
```

---

## Summary

| Question                      | Key API features used                                                   | Works well?                          |
|-------------------------------|-------------------------------------------------------------------------|--------------------------------------|
| Q1 — pose at log time         | `.search_text()` + `.at()`                                              | Yes                                  |
| Q2 — continuous observation   | `.search_embedding()` + VLM verify + `.order_by()` + segmentation       | Yes                                  |
| Q3 — count unique people      | `.filter_after()` + `.vector()` + DBSCAN                                | Yes                                  |
| Q4 — map red socks            | `.search_embedding()` + VLM + `.filter_refs()` + costmap                | Yes                                  |
| Q5 — door opener              | `.filter_tags()` + `.filter_time()` + `.filter_near()`                  | Yes, cross-stream loop               |
| Q6 — STT + voice identity     | `.filter_time()` + `.vector()`                                          | Yes                                  |
| Q7 — voice ↔ face             | `.filter_time()` + `.vector()` + accumulation                           | Yes                                  |
| Q8 — room diff                | `.filter_time()` + `.filter_near()` + `.vector()` diff                  | Yes, diffing is app logic            |
| Q9 — cat trail                | `.filter_tags()` + `.order_by()` + pose path                            | Yes                                  |
| Q10 — pre-event timeline      | `.filter_time()` + `list_streams()` → `StreamInfo`                      | Yes                                  |
| Q11 — absence detection       | `.filter_tags()` + `.order_by()` + gap analysis                         | Yes, negation is app logic           |
| Q12 — mailman schedule        | `.search_embedding()` or `.filter_tags()` + time stats                  | Yes, pattern extraction is app logic |
| Q13 — cross-robot diff        | `.filter_tags(robot_id=)` + `.filter_near()` + `.vector()`              | Yes                                  |
| Q14 — distance while carrying | `.filter_tags()` + `.filter_time()` + `.order_by()` + pose accumulation | Yes, cross-stream conditional join   |

**API gaps exposed by Q8-Q14**:

**Remaining gap**:

| Gap | Affects | Suggestion |
|-----|---------|------------|
| Cross-stream conditional join is always a manual loop | Q5,Q7,Q10,Q14 | Phase 3 `Correlator` — the most motivated feature |
