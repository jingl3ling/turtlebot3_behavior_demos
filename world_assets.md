## Maze World Assets (Benches + Objects)

**Note**: This file is a template/checklist for the HW2 world configuration. You should fill in the concrete numbers after placing benches and objects in your maze world in Gazebo (or the provided TurtleBot3 maze world).

### Bench Layout

- **Bench IDs**: `bench_1`, `bench_2`, `bench_3`, `bench_4`
- **Requirements**:
  - Benches must touch maze walls.
  - Bench height must allow the robot camera to see all objects when the robot is in front of the bench.
  - Each bench must use **only COCO classes** (e.g., cup, bottle, book, laptop, mouse, keyboard, cell phone, etc.).
  - Each bench must have a **different object set**.

Fill in the approximate poses (in the world frame) once you place them. Use any consistent pose convention \((x, y, yaw)\) that matches your simulator.

#### Bench 1 — `bench_1`

- **Approx pose (world frame)**:
  - `x ≈ ___`
  - `y ≈ ___`
  - `yaw ≈ ___` (radians)
- **Objects on bench** (COCO classes only, 3–5 items):
  - `object_1` (e.g., `cup`)
  - `object_2`
  - `object_3`
  - `object_4` (optional)
  - `object_5` (optional)
- **Screenshots (robot camera view)**:
  - Save 2 images from the robot camera when it is in front of `bench_1` and looking at the objects.
  - Recommended filenames:
    - `media/bench_1_view_1.png`
    - `media/bench_1_view_2.png`

#### Bench 2 — `bench_2`

- **Approx pose (world frame)**:
  - `x ≈ ___`
  - `y ≈ ___`
  - `yaw ≈ ___` (radians)
- **Objects on bench** (COCO classes only, 3–5 items):
  - `object_1`
  - `object_2`
  - `object_3`
  - `object_4` (optional)
  - `object_5` (optional)
- **Screenshots (robot camera view)**:
  - `media/bench_2_view_1.png`
  - `media/bench_2_view_2.png`

#### Bench 3 — `bench_3`

- **Approx pose (world frame)**:
  - `x ≈ ___`
  - `y ≈ ___`
  - `yaw ≈ ___` (radians)
- **Objects on bench** (COCO classes only, 3–5 items):
  - `object_1`
  - `object_2`
  - `object_3`
  - `object_4` (optional)
  - `object_5` (optional)
- **Screenshots (robot camera view)**:
  - `media/bench_3_view_1.png`
  - `media/bench_3_view_2.png`

#### Bench 4 — `bench_4` (optional if using only 3 benches)

- **Approx pose (world frame)**:
  - `x ≈ ___`
  - `y ≈ ___`
  - `yaw ≈ ___` (radians)
- **Objects on bench** (COCO classes only, 3–5 items):
  - `object_1`
  - `object_2`
  - `object_3`
  - `object_4` (optional)
  - `object_5` (optional)
- **Screenshots (robot camera view)**:
  - `media/bench_4_view_1.png`
  - `media/bench_4_view_2.png`

### How to Capture Bench Screenshots

1. **Launch the maze world** in Gazebo with your TurtleBot3 simulation.
2. Drive or command the robot so it is positioned in front of each bench, looking at the objects.
3. Open an `rqt_image_view` window (or the Gazebo camera view) on the camera topic (e.g., `/camera/image_raw`).
4. For each bench:
   - Take **two screenshots** of the camera view from slightly different distances or headings.
   - Save them under the `media/` folder relative to this workspace.
5. List the final filenames for each bench in the sections above.

