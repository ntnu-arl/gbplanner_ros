# Atomic map snapshot and its provenance. The original voxel_map topic remains
# available as PointCloud2 with the same point layout.
sensor_msgs/PointCloud2 map
# Unique mapper process instance. Stable across map resets and imports.
string source_id
# Starts at one and increases on reset, load and replacement imports. Delayed
# messages from an older epoch must not replace a newer service/topic snapshot.
uint64 map_epoch
# Strictly increasing for every published snapshot within this source_id.
# Never resets when map_epoch increases.
uint64 update_id
# Counts published batches containing newly integrated physical sensor scans
# in this map epoch. Several scans may share one batch. Synthetic operations
# advance it only when their snapshot also includes pending physical scans.
uint64 observation_id
# Acquisition stamp of the most recently integrated physical sensor scan.
# Zero when this epoch has not integrated a physical scan.
builtin_interfaces/Time observation_stamp
# observation, freespace, augmentation, import, load, reset or snapshot.
string change_kind
