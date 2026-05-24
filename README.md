# SCARA_projekt

## Final Sorting Scene

This project simulates a SCARA robot that will use an AI-assisted camera to
detect objects on a table and sort them into separate collection bins. The
current milestone prepares the simulation scene before the neural network is
implemented.

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

The table camera stays unchanged for this milestone. It will later provide the
input image for the AI detector node, which should publish the object class,
confidence, and image position for the robot movement algorithm.

### Run The Simulation

```bash
cd /home/telegdicsongor/scara_ws/SCARA_projekt
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
