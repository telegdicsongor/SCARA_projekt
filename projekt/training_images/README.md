# Cube Detector Training Images

This folder contains the class-folder source images for the SCARA cube detector:

```text
training_images/
  wood_cube/
  steel_cube/
  not_cube/
```

These images can be reused for YOLO training, but `wood_cube` and `steel_cube`
frames still need bounding boxes before training.

## Annotate Existing Images

From the workspace folder:

```bash
cd /home/veszpo/projekt_ws
python3 SCARA_projekt/projekt/scripts/save_training_images.py \
  --source-root SCARA_projekt/projekt/training_images
```

Controls:

- drag with the left mouse button: select the cube bounding box
- Enter or Space: save the current image
- `n`: save the current image as background/no cube label
- `t`: save future samples to `train`
- `v`: save future samples to `val`
- `s`: skip current image
- `c` or right-click: clear the current box
- `q`: quit

The YOLO dataset is written to:

```text
/home/veszpo/projekt_ws/SCARA_projekt/projekt/datasets/scara_cubes/
```

## Train And Export

From `/home/veszpo/projekt_ws`, save future training runs inside the project:

```bash
yolo detect train model=SCARA_projekt/projekt/yolov8n.pt data=SCARA_projekt/projekt/datasets/scara_cubes/data.yaml epochs=80 imgsz=640 project=SCARA_projekt/projekt/runs/detect name=train
yolo export model=SCARA_projekt/projekt/runs/detect/train-3/weights/best.pt format=onnx imgsz=640 opset=12 simplify=False
```

The default sorting launch now uses:

```text
SCARA_projekt/projekt/runs/detect/train-3/weights/best.onnx
```
