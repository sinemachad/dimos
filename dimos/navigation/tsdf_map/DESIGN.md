# TSDFMap: Dynamic Global Mapping via Truncated Signed Distance Fields

## Overview

`TSDFMap` provides a dynamic 3-D global map for the Go2 quadruped (and any robot with
a LiDAR + odometry stream).  It fuses incoming point clouds into a sparse TSDF volume and
publishes the resulting occupied-voxel point cloud at a configurable rate.

Unlike occupancy-based mappers (OctoMap, `VoxelGridMapper`), TSDF naturally handles
**dynamic obstacles** without explicit object tracking.

---

## TSDF vs OctoMap

| Property | OctoMap (log-odds occupancy) | TSDF |
|---|---|---|
| Per-voxel value | log-odds of occupancy probability | Signed distance to nearest surface |
| Free-space update | Decreases log-odds | Large positive SDF overwrites stale occupied |
| Dynamic obstacle clearing | Requires explicit ray-casting & log-odds decay | Implicit - ray sweeps write positive SDF values |
| Surface representation | Binary occupied/free | Continuous zero-crossing -> smooth mesh extraction |
| Memory | Octree (O(n log n)) | Sparse hash-map (O(n)) |
| Typical use | Navigation costmaps | Dense reconstruction, manipulation, SLAM |

In TSDF:
- **Positive** values -> voxel is in *free space* ahead of the observed surface
- **Negative** values -> voxel is *behind* (inside) the observed surface
- **Zero-crossing** -> the surface itself

When a moving obstacle vacates a region, subsequent LiDAR rays sweep through it with
positive TSDF updates (free space), gradually overwriting the stale negative values.
The `sdf_trunc` parameter controls how aggressively this clearing propagates.

---

## VDBFusion

### Paper
> Vizzo, I., Guadagnino, T., Behley, J., & Stachniss, C. (2022).
> **VDBFusion: Flexible and Efficient TSDF Integration of Range Sensor Data**.
> *Sensors*, 22(3), 1296. https://doi.org/10.3390/s22031296

VDBFusion wraps the [OpenVDB](https://www.openvdb.org/) sparse volumetric data structure
(originally developed at DreamWorks Animation) with a fast TSDF integration kernel.

### Python API used by TSDFMap
```python
from vdbfusion import VDBVolume

vol = VDBVolume(voxel_size=0.15, sdf_trunc=0.3, space_carving=True)
vol.integrate(points_Nx3, sensor_origin)   # fuse one scan
vertices, sdf_vals = vol.extract_vdb_grids(0.0)  # extract near-surface voxels
```

`space_carving=True` enables the free-space ray-casting that clears dynamic obstacles.

### Python 3.12 note
As of early 2026 the PyPI wheels for `vdbfusion` target Python <= 3.10.
DimOS 3.12 deployments therefore fall back to the pure-Python TSDF backend in
`dimos/navigation/tsdf_map/tsdf_volume.py`, which implements the same integration
algorithm using a `dict`-backed sparse voxel grid.  The fallback is functionally
equivalent but slower on dense scans.  When vdbfusion ships Python 3.12
wheels the code will automatically prefer the C++ backend.

---

## Module Architecture

```
LiDAR (PointCloud2)  -> registered_scan ->+
                                           |  TSDFMap
Robot Odometry (PoseStamped) -> raw_odom ->|     |
                                           |  integrate()
                                           |     |
                                           +-> tsdf_volume
                                                  |
                                            get_occupied_points()
                                                  |
                                           global_map (PointCloud2)
                                           odom (PoseStamped) [pass-through]
```

### Config parameters

| Name | Default | Description |
|---|---|---|
| `voxel_size` | 0.15 m | Voxel edge length |
| `sdf_trunc` | 0.30 m | TSDF truncation distance (approx 2x voxel_size) |
| `max_range` | 15.0 m | Points further than this are discarded |
| `map_publish_rate` | 0.5 Hz | How often `global_map` is emitted |
| `max_weight` | 64.0 | Maximum per-voxel integration weight |

---

## Blueprint

`unitree_go2_tsdf_nav` extends `unitree_go2` with TSDFMap:

```python
unitree_go2_tsdf_nav = (
    autoconnect(
        unitree_go2,
        TSDFMap.blueprint(voxel_size=0.15, sdf_trunc=0.3),
    )
    .remappings([
        (TSDFMap, "registered_scan", "lidar"),
        (TSDFMap, "raw_odom", "go2_odom"),
    ])
    .global_config(n_workers=8, robot_model="unitree_go2")
)
```

---

## References

- Vizzo et al. (2022) - VDBFusion paper: https://www.mdpi.com/1424-8220/22/3/1296
- OpenVDB: https://www.openvdb.org/
- VDBFusion GitHub: https://github.com/PRBonn/vdbfusion
- Curless & Levoy (1996) - original TSDF formulation: https://dl.acm.org/doi/10.1145/237170.237269
