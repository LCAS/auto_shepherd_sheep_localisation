#!/usr/bin/env python3
"""Prepare the local sheep annotations for Ultralytics YOLO training.

The Roboflow and South Africa exports are COCO JSON. This script converts
their sheep categories to one YOLO class and copies the train/valid/test
images into the repository's training/dataset layout. Legacy South Africa
text files are supported separately because they are pixel
x1,y1,x2,y2,confidence groups rather than YOLO labels.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path


IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".JPG", ".JPEG", ".PNG")


def copy_image(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if source.stat().st_size == destination.stat().st_size:
            return
        raise FileExistsError(f"Refusing to overwrite different file: {destination}")
    shutil.copy2(source, destination)


def write_labels(destination: Path, rows: list[str]) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("\n".join(rows) + ("\n" if rows else ""))


def convert_box(bbox, width: float, height: float) -> str | None:
    if len(bbox) != 4 or width <= 0 or height <= 0:
        return None
    x, y, box_width, box_height = (float(value) for value in bbox)
    x1 = max(0.0, min(width, x))
    y1 = max(0.0, min(height, y))
    x2 = max(0.0, min(width, x + box_width))
    y2 = max(0.0, min(height, y + box_height))
    if x2 <= x1 or y2 <= y1:
        return None
    center_x = ((x1 + x2) / 2.0) / width
    center_y = ((y1 + y2) / 2.0) / height
    normalized_width = (x2 - x1) / width
    normalized_height = (y2 - y1) / height
    return f"0 {center_x:.8f} {center_y:.8f} {normalized_width:.8f} {normalized_height:.8f}"


def convert_coco_split(source_root: Path, destination_root: Path, source_split: str, target_split: str) -> tuple[int, int]:
    split_root = source_root / source_split
    annotation_path = split_root / "_annotations.coco.json"
    if not annotation_path.is_file():
        raise FileNotFoundError(annotation_path)
    data = json.loads(annotation_path.read_text())
    sheep_category_ids = {
        category["id"]
        for category in data.get("categories", [])
        if str(category.get("name", "")).strip().lower() == "sheep"
    }
    annotations_by_image: dict[int, list[dict]] = {}
    for annotation in data.get("annotations", []):
        if annotation.get("category_id") in sheep_category_ids:
            annotations_by_image.setdefault(annotation["image_id"], []).append(annotation)

    copied = 0
    boxes = 0
    for image in data.get("images", []):
        source_image = split_root / image["file_name"]
        if not source_image.is_file():
            raise FileNotFoundError(source_image)
        destination_image = destination_root / "images" / target_split / source_image.name
        destination_label = destination_root / "labels" / target_split / f"{source_image.stem}.txt"
        copy_image(source_image, destination_image)
        rows = []
        for annotation in annotations_by_image.get(image["id"], []):
            row = convert_box(annotation.get("bbox", []), image["width"], image["height"])
            if row is not None:
                rows.append(row)
        write_labels(destination_label, rows)
        copied += 1
        boxes += len(rows)
    return copied, boxes


def convert_south_africa_coco(source_root: Path, destination_root: Path) -> tuple[int, int]:
    """Import the South Africa Image Data COCO train/val/test export."""
    image_root = source_root / "Train and Validation Images"
    if not image_root.is_dir():
        return 0, 0

    copied = 0
    boxes = 0
    for source_split, target_split in (("train", "train"), ("val", "val"), ("test", "test")):
        annotation_path = source_root / f"{source_split}.json"
        if not annotation_path.is_file():
            continue
        data = json.loads(annotation_path.read_text())
        sheep_category_ids = {
            category["id"]
            for category in data.get("categories", [])
            if "sheep" in str(category.get("name", "")).strip().lower()
        }
        annotations_by_image: dict[int, list[dict]] = {}
        for annotation in data.get("annotations", []):
            if annotation.get("category_id") in sheep_category_ids:
                annotations_by_image.setdefault(annotation["image_id"], []).append(annotation)

        for image in data.get("images", []):
            source_image = image_root / image.get("file_name", image.get("path", ""))
            if not source_image.is_file():
                raise FileNotFoundError(source_image)
            # Prefix these files so a same-named image from another source
            # cannot overwrite or collide with its label.
            destination_name = f"south_africa_{source_image.name}"
            destination_image = destination_root / "images" / target_split / destination_name
            destination_label = destination_root / "labels" / target_split / f"{Path(destination_name).stem}.txt"
            copy_image(source_image, destination_image)
            rows = []
            for annotation in annotations_by_image.get(image["id"], []):
                row = convert_box(annotation.get("bbox", []), image["width"], image["height"])
                if row is not None:
                    rows.append(row)
            write_labels(destination_label, rows)
            copied += 1
            boxes += len(rows)
    return copied, boxes


def image_for_label(label_path: Path) -> Path | None:
    for extension in IMAGE_EXTENSIONS:
        candidate = label_path.with_suffix(extension)
        if candidate.is_file():
            return candidate
    return None


def image_size(path: Path) -> tuple[int, int]:
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("OpenCV is required for --include-south-africa") from exc
    image = cv2.imread(str(path))
    if image is None:
        raise ValueError(f"Could not read image: {path}")
    height, width = image.shape[:2]
    return width, height


def convert_south_africa(source_root: Path, destination_root: Path) -> tuple[int, int]:
    copied = 0
    boxes = 0
    for label_path in sorted(source_root.rglob("*.txt")):
        source_image = image_for_label(label_path)
        if source_image is None:
            continue
        values = [float(value) for value in label_path.read_text().split()]
        if len(values) % 5 != 0:
            raise ValueError(f"Expected x1,y1,x2,y2,confidence groups in {label_path}")
        width, height = image_size(source_image)
        rows = []
        for index in range(0, len(values), 5):
            x1, y1, x2, y2, _confidence = values[index:index + 5]
            row = convert_box([x1, y1, x2 - x1, y2 - y1], width, height)
            if row is not None:
                rows.append(row)
        destination_image = destination_root / "images" / "train" / source_image.name
        destination_label = destination_root / "labels" / "train" / f"{source_image.stem}.txt"
        copy_image(source_image, destination_image)
        write_labels(destination_label, rows)
        copied += 1
        boxes += len(rows)
    return copied, boxes


def main() -> None:
    repository_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--coco-root",
        type=Path,
        default=repository_root / "auto_shepherd_sheep_localisation_ros2" / "training" / "SLAMB.v4-08-10-2026-396-images-no-augmentation-no-tile.coco",
    )
    parser.add_argument(
        "--south-africa-root",
        type=Path,
        default=repository_root / "auto_shepherd_sheep_localisation_ros2" / "training" / "south_africa" / "Video Data",
    )
    parser.add_argument(
        "--south-africa-image-data-root",
        type=Path,
        default=repository_root / "auto_shepherd_sheep_localisation_ros2" / "training" / "south_africa" / "Image Data",
    )
    parser.add_argument(
        "--destination",
        type=Path,
        default=repository_root / "auto_shepherd_sheep_localisation_ros2" / "training" / "dataset",
    )
    parser.add_argument(
        "--include-south-africa",
        action="store_true",
        help="also import the legacy pixel-box text files as sheep labels",
    )
    args = parser.parse_args()
    args.destination.mkdir(parents=True, exist_ok=True)

    totals = {"images": 0, "boxes": 0}
    for source_split, target_split in (("train", "train"), ("valid", "val"), ("test", "test")):
        images, boxes = convert_coco_split(args.coco_root, args.destination, source_split, target_split)
        totals["images"] += images
        totals["boxes"] += boxes
        print(f"COCO {source_split:5s} -> {target_split:5s}: {images} images, {boxes} sheep boxes")

    images, boxes = convert_south_africa_coco(args.south_africa_image_data_root, args.destination)
    totals["images"] += images
    totals["boxes"] += boxes
    if images:
        print(f"South Africa COCO -> train/val/test: {images} images, {boxes} sheep boxes")

    if args.include_south_africa:
        images, boxes = convert_south_africa(args.south_africa_root, args.destination)
        totals["images"] += images
        totals["boxes"] += boxes
        print(f"South Africa -> train: {images} images, {boxes} sheep boxes")

    print(f"Total: {totals['images']} images, {totals['boxes']} sheep boxes")
    print(f"Dataset YAML: {args.destination / 'sheep.yaml'}")


if __name__ == "__main__":
    main()
