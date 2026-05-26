# SCARA_projekt

## Final Sorting Scene

This project simulates a SCARA robot that uses a table camera detector to
detect objects on a table and sort them into separate collection bins.

The scene contains two 5 cm dynamic cubes in the center input area:

- `wood_cube_5cm`
- `steel_cube_5cm`

The target areas are two low open collection bins on the table:

- wood bin on the left
- steel bin on the right

| Object class | Start pose `(x y z r p y)` | Target bin | Purpose |
| --- | --- | --- | --- |
| `wood_cube_5cm` | `0.00 0.18 1.015 0 0 0` | `wood_collection_bin` | First neural-network detection target |
| `steel_cube_5cm` | `0.12 0.18 1.015 0 0 0` | `steel_collection_bin` | Future second object class for sorting |

The table camera provides the compressed image stream for the detector node,
which publishes object class, confidence, and image position for the movement
algorithm.

### Run The Simulation

```bash
cd /home/veszpo/projekt_ws/SCARA_projekt
colcon build --packages-select projekt
source install/setup.bash
ros2 launch projekt spawn_robot.launch.py
```

### Verify The Camera

```bash
ros2 topic hz /table_camera/image
ros2 topic echo /table_camera/camera_info --once
```

### Documentation Screenshot

For the final report, launch the simulation and save a Gazebo screenshot showing:

- wood cube in the center input area
- steel cube in the center input area
- wood bin on the left
- steel bin on the right

Recommended screenshot path:

```text
docs/images/final_sorting_scene.png
```

## Collect Training Images

The first AI milestone is a camera image collection node. It subscribes to the
table camera image and saves labeled frames for neural network training.

Start the simulation in one terminal:

```bash
cd /home/veszpo/projekt_ws/SCARA_projekt
source install/setup.bash
ros2 launch projekt spawn_robot.launch.py
```

Start the collector in another terminal:

```bash
cd /home/veszpo/projekt_ws/SCARA_projekt
source install/setup.bash
ros2 run projekt save_training_images.py
```

Keyboard labels in the OpenCV window:

- drag a box around the cube
- `w`: save the box as `wood_cube`
- `s`: save the box as `steel_cube`
- `n`: save the frame as background
- `t` / `v`: select train or validation split
- `q`: quit

Existing class-folder images are stored under:

```text
projekt/training_images/
```

YOLO training datasets are stored under:

```text
projekt/datasets/
```

YOLO training runs are stored under:

```text
projekt/runs/
```

The sorting launch uses the trained detector at
`projekt/runs/detect/train-3/weights/best.onnx` by default.
