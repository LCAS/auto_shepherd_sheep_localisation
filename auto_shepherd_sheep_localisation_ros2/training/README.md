# YOLO training dataset

Place the local training data in `dataset/` using this layout:

```text
dataset/
├── sheep.yaml
├── images/
│   ├── train/
│   ├── val/
│   └── test/
└── labels/
    ├── train/
    ├── val/
    └── test/
```

Images and labels are ignored by Git because they can be large. The folder
markers, `sheep.yaml`, and this README are tracked so the structure is shared.

The COCO-to-YOLO importer is available at
`scripts/prepare_yolo_dataset.py`. It maps the Roboflow and South Africa
`train`, `valid`/`val`, and `test` splits to the corresponding YOLO folders
and keeps sheep-labelled categories. South Africa image data is prefixed with
`south_africa_` to prevent filename collisions.

To regenerate the dataset from the local Roboflow export:

```bash
python3 scripts/prepare_yolo_dataset.py
```

That imports the Roboflow data and the South Africa COCO image data. The
current combined dataset contains 383 train, 74 validation, and 64 test
images, with 16,306 sheep boxes. The Roboflow converter combines both COCO
categories named `sheep` into class `0` and ignores `human` and `sheepdog`.

The older South Africa `.txt` files use pixel `x1,y1,x2,y2,confidence`
groups rather than YOLO labels. To additionally import those legacy boxes:

```bash
python3 scripts/prepare_yolo_dataset.py --include-south-africa
```

Only use that option after checking that those confidence-bearing boxes are
ground-truth annotations. They are not standard COCO or YOLO labels.

Example training command inside the Docker container, using GPU 0:

```bash
cd /home/ros/base_ws/src/auto_shepherd_sheep_localisation_ros2
yolo detect train \
  model=/home/ros/base_ws/src/auto_shepherd_sheep_localisation_ros2/auto_shepherd_sheep_localisation_ros2/detection_process/models/samples/sample_model/Aerial-Auth-Asfenah-DeepBack-PrabsUoL3-9.pt \
  data=training/dataset/sheep.yaml \
  epochs=100 imgsz=640 batch=4 device=0
```

Each YOLO label file must have one row per sheep:

```text
class_id x_center_normalised y_center_normalised width_normalised height_normalised
```
