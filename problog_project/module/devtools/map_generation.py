#!/usr/bin/env python3
"""
module/devtools/map_generation.py

Standalone dev tool, NOT part of the live main.py pipeline. Generates a
ROS map_server-compatible map (map.pgm + map.yaml) describing a
28 m x 17 m environment containing circular obstacles of 0.5 m diameter.
The .pgm/.yaml pair is the standard on-disk representation that ROS's
map_server / nav2 map_server loads and republishes as a nav_msgs/OccupancyGrid
on the /map topic. Cell values follow the map_server convention:
    254 -> free      (occupancy 0)
    0   -> occupied   (occupancy 100)
    205 -> unknown    (occupancy -1)  [not used here, whole map is known]

To use the result for a real problem, run this from (or copy its output
into) that problem's own directory, e.g. problems/<name>/map.yaml +
map.pgm -- main.py's own pre-flight translator
(module/translators/occgrid_to_problog.py) reads map.yaml from there.
"""
import os
import numpy as np
from PIL import Image

# Save all outputs to a fixed ./maps/ directory, relative to the current
# working directory (i.e. wherever you invoke the script from), not to
# the script's own location.
OUTPUT_DIR = os.path.join(os.getcwd(), "maps")
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ---- Map parameters ----------------------------------------------------
RESOLUTION = 0.05          # meters/cell
WIDTH_M = 18.0              # meters, x
HEIGHT_M = 17.0             # meters, y
OBSTACLE_DIAMETER = 0.5     # meters
OBSTACLE_RADIUS = OBSTACLE_DIAMETER / 2.0
OBSTACLES_XY = [
    (4, 4), (9, 4), (14, 4),
    (4, 7), (9, 7), (14, 7),
    (4, 10), (9, 10), (14, 10),
    (4, 13), (9, 13), (14, 13),
]
WIDTH_CELLS = int(round(WIDTH_M / RESOLUTION))
HEIGHT_CELLS = int(round(HEIGHT_M / RESOLUTION))
# occupancy grid in "map frame" indexing: occ[row, col], row 0 = y=0 (bottom)
# this matches the nav_msgs/OccupancyGrid.data row-major, origin-at-bottom-left convention
occ = np.zeros((HEIGHT_CELLS, WIDTH_CELLS), dtype=np.uint8)  # 0 = free
rows, cols = np.indices(occ.shape)
cell_x = (cols + 0.5) * RESOLUTION
cell_y = (rows + 0.5) * RESOLUTION
for (ox, oy) in OBSTACLES_XY:
    dist2 = (cell_x - ox) ** 2 + (cell_y - oy) ** 2
    occ[dist2 <= OBSTACLE_RADIUS ** 2] = 100  # 100 = occupied (OccupancyGrid convention)
# ---- Save raw occupancy data (row 0 = y=0), useful for direct OccupancyGrid publishing
np.save(os.path.join(OUTPUT_DIR, "occupancy_data.npy"), occ.astype(np.int8))
# ---- Build PGM image for map_server -------------------------------------
# map_server PGM convention: row 0 of the IMAGE is the TOP of the map (max y),
# and pixel value 254=free, 0=occupied, 205=unknown.
pgm = np.full(occ.shape, 254, dtype=np.uint8)
pgm[occ == 100] = 0
pgm_image = np.flipud(pgm)  # flip so image row 0 = y_max (top), matching map_server convention
img = Image.fromarray(pgm_image, mode="L")
img.save(os.path.join(OUTPUT_DIR, "map.pgm"))
# ---- Write map.yaml -------------------------------------------------------
# NOTE: the "image" field is relative to the yaml file's own directory
# (./maps/), so it must stay as the bare filename "map.pgm", not a path
# that includes "maps/" again.
yaml_content = f"""image: map.pgm
resolution: {RESOLUTION}
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
with open(os.path.join(OUTPUT_DIR, "map.yaml"), "w") as f:
    f.write(yaml_content)
print(f"Map generated: {WIDTH_CELLS} x {HEIGHT_CELLS} cells @ {RESOLUTION} m/cell")
print(f"Obstacles placed: {len(OBSTACLES_XY)}")
print(f"Output directory: {OUTPUT_DIR}")