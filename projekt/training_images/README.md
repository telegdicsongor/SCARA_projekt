# Training Images

This folder is for labeled camera images used to train the cube classifier.

Run the image collector from the package source folder:

```bash
cd /home/telegdicsongor/scara_ws/SCARA_projekt/projekt
source ../install/setup.bash
ros2 run projekt save_training_images.py
```

Keyboard labels:

- `w`: save the current frame as `wood_cube`
- `s`: save the current frame as `steel_cube`
- `n`: save the current frame as `not_cube`
- `q`: quit the OpenCV window

Recommended first dataset:

| Folder | Meaning | First target |
| --- | --- | --- |
| `wood_cube/` | Wood cube visible | 30 images |
| `steel_cube/` | Steel cube visible | 30 images |
| `not_cube/` | Empty table, bins, robot, or unclear view | 30 images |

For a better model, collect at least 100 images per class. Move the objects to
different places on the table and include slightly different camera views if
possible.
