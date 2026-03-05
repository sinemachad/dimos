```python
# Filter → transform → store

  images.after(one_hour_ago) \
      .near(kitchen_pose, 5.0) \
      .transform(CLIPModel()) \
      .store("kitchen_embeddings")

  # Filter → transform → fetch (in-memory, not persisted)
  results = images.after(one_hour_ago) \
      .near(kitchen_pose, 5.0) \
      .transform(CLIPModel()) \
      .fetch()

  # Filter → transform → transform → store
  images.near(kitchen_pose, 5.0) \
      .transform(CLIPModel()) \
      .transform(CigaretteDetector(vlm)) \
      .store("kitchen_cigarette_detections")

```
