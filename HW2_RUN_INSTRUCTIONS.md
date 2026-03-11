## HW 2: Object Detection Events (ROS 2 + Zenoh + PostgreSQL)

This file describes how to build and run the full pipeline you now have in this workspace.

### 1. Prerequisites

- **ROS 2 Humble** (sourced)
- **Python 3.10+**
- **TurtleBot3 simulation** working (you should already be able to launch the maze/house world and see `/camera/image_raw`, `/odom`, `/tf`).
- **Zenoh** router (`zenohd`) installed and in `PATH`.
- **PostgreSQL** server running locally.

#### Python packages

Install the following in the same environment where you run the nodes:

```bash
pip install ultralytics opencv-python pillow zenoh psycopg2-binary cv_bridge
```

> Adjust `cv_bridge` installation if you already use the ROS Debian package version.

### 2. Build the workspace (once, after adding the `maze_events` package)

From the workspace root:

```bash
cd /home/dmin/turtlebot_ws
colcon build
source install/setup.bash
```

You must source `install/setup.bash` in every new terminal before running any nodes in this workspace.

### 3. Set up PostgreSQL schema

1. Ensure PostgreSQL is running and that you have a database (example below uses `maze_events`, user `maze`, password `maze`):

```bash
sudo -u postgres createuser -P maze   # set password 'maze' when prompted
sudo -u postgres createdb -O maze maze_events
```

2. Apply the schema from this repo:

```bash
cd /home/dmin/turtlebot_ws
psql "host=localhost port=5432 dbname=maze_events user=maze password=maze" -f maze_events_schema.sql
```

3. Optionally verify:

```bash
psql "host=localhost port=5432 dbname=maze_events user=maze password=maze" -c "\dt"
```

### 4. Start Zenoh router

In a new terminal:

```bash
zenohd
```

By default the code assumes a TCP locator `tcp/127.0.0.1:7447`. If your router uses a different port or locator, adjust the `zenoh_locator` parameter or `zenoh` config.

### 5. Launch TurtleBot simulation

Use your course-provided maze world launch, or for the `turtlebot3_behavior_demos` house world:

```bash
source /home/dmin/turtlebot_ws/install/setup.bash
ros2 launch tb_worlds tb_demo_world.launch.py
```

Ensure you see topics:

```bash
ros2 topic list | grep -E "/camera/image_raw|/odom"
```

### 6. Run the detection + Zenoh publisher node

In another terminal:

```bash
cd /home/dmin/turtlebot_ws
source install/setup.bash

ros2 run maze_events detector_node \
  --ros-args \
  -p camera_topic:=/camera/image_raw \
  -p odom_topic:=/odom \
  -p base_frame:=base_footprint \
  -p camera_frame:=camera_link \
  -p robot_id:=tb3_sim \
  -p run_id:=$(uuidgen) \
  -p yolo_model:=yolov8n.pt \
  -p zenoh_locator:=tcp/127.0.0.1:7447
```

Notes:

- `run_id` should be a new UUID for each experiment; `uuidgen` is convenient.
- `yolo_model` should point to a local YOLO weights file (`yolov8n.pt` is a small model; download with Ultralytics if needed).

You should see log messages about YOLO loading and Zenoh connecting. The node will:

- Subscribe to `/camera/image_raw` and `/odom`.
- Look up TF `base_footprint -> camera_link`.
- Publish detection events as JSON to keys:
  - `maze/<robot_id>/<run_id>/detections/v1/<event_id>`
  - `maze/<robot_id>/<run_id>/runmeta/v1` (once per run)

### 7. Run the Zenoh ingest worker (PostgreSQL writer)

In another terminal:

```bash
cd /home/dmin/turtlebot_ws
source install/setup.bash

export MAZE_EVENTS_PG_DSN="host=localhost port=5432 dbname=maze_events user=maze password=maze"

ros2 run maze_events zenoh_ingest_worker
```

This process will:

- Subscribe to `maze/**/detections/v1/*` over Zenoh.
- Parse each JSON event.
- Insert into `detection_events` and `detections` inside a single transaction.
- Rely on the `PRIMARY KEY` and `UNIQUE` constraints for **idempotent** inserts.

### 8. Running an experiment

1. Start (in order, each in its own terminal):
   - PostgreSQL (service)
   - `zenohd`
   - TurtleBot simulation (`ros2 launch ...`)
   - Detection node (`ros2 run maze_events detector_node ...`)
   - Ingest worker (`ros2 run maze_events zenoh_ingest_worker`)
2. Drive the robot (teleop or autonomy) so it views benches and objects.
3. Let the system run until you have collected enough detections (e.g., 1–2 minutes of motion).
4. Stop nodes with `Ctrl+C` in reverse order.

### 9. Simple run report queries

From `psql`:

- **Total detection events**:

  ```sql
  SELECT COUNT(*) FROM detection_events;
  ```

- **Total detections**:

  ```sql
  SELECT COUNT(*) FROM detections;
  ```

- **Events per run**:

  ```sql
  SELECT run_id, COUNT(*) AS num_events
  FROM detection_events
  GROUP BY run_id
  ORDER BY num_events DESC;
  ```

- **Class histogram**:

  ```sql
  SELECT class_name, COUNT(*) AS count
  FROM detections
  GROUP BY class_name
  ORDER BY count DESC;
  ```

Use the results of these queries to fill in your HW2 **run report**, including:

- Number of events ingested.
- Number of detections.
- Histogram of classes seen (top-N classes with counts).

