"""
YOLO detection on drone videos.

This module wraps the Ultralytics YOLO API for detecting sheep in drone footage.
It supports:
- Loading custom YOLO weights
- Filtering detections to 'sheep' class only
- Optional tracking with ByteTrack
"""
import logging
from typing import List, Dict, Any, Optional, Tuple
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from sheepgeo.config import DEFAULT_CONFIDENCE, DEFAULT_IOU_THRESHOLD


logger = logging.getLogger(__name__)


class SheepDetector:
    """
    YOLO-based sheep detector for drone videos.
    
    This class wraps the Ultralytics YOLO model and provides a simple interface
    for detecting sheep in video frames.
    """
    
    def __init__(
        self,
        weights_path: str = "yolov8n.pt",
        confidence: float = DEFAULT_CONFIDENCE,
        iou_threshold: float = DEFAULT_IOU_THRESHOLD,
        track: bool = False,
        tracker_config: Optional[str] = None
    ):
        """
        Initialize the sheep detector.
        
        Args:
            weights_path: Path to YOLO weights file
            confidence: Confidence threshold for detections
            iou_threshold: IoU threshold for NMS
            track: Enable tracking with ByteTrack
            tracker_config: Path to tracker configuration file
        """
        self.weights_path = weights_path
        self.confidence = confidence
        self.iou_threshold = iou_threshold
        self.track_enabled = track
        self.tracker_config = tracker_config
        
        # Load YOLO model
        logger.info(f"Loading YOLO model from {weights_path}")
        self.model = YOLO(weights_path)
        
        logger.info(f"Detector initialized: conf={confidence}, iou={iou_threshold}, "
                   f"track={track}")
    
    def detect_frame(
        self,
        frame: np.ndarray,
        frame_index: int = 0
    ) -> List[Dict[str, Any]]:
        """
        Detect sheep in a single frame.
        
        Args:
            frame: OpenCV BGR image (H x W x 3)
            frame_index: Frame index (for logging)
            
        Returns:
            List of detections, each a dict with:
                - bbox_xyxy: [x1, y1, x2, y2]
                - confidence: detection confidence
                - class_id: class ID (should be SHEEP_CLASS_ID)
                - track_id: track ID (if tracking enabled, else None)
        """
        detections = []
        
        # Run inference
        if self.track_enabled:
            results = self.model.track(
                frame,
                conf=self.confidence,
                iou=self.iou_threshold,
                persist=True,
                tracker=self.tracker_config,
                verbose=False
            )
        else:
            results = self.model.predict(
                frame,
                conf=self.confidence,
                iou=self.iou_threshold,
                verbose=False
            )
        
        # Extract detections
        for result in results:
            if result.boxes is None or len(result.boxes) == 0:
                continue
            
            boxes = result.boxes
            
            for i in range(len(boxes)):
                box = boxes[i]
                
                # Get class ID
                class_id = int(box.cls.item())
                
                # Note: We accept all detections from the model here.
                # The CLI provides --filter-class option to filter by class name (e.g., 'sheep')
                # rather than hard-coding class IDs, since different models may use different
                # class IDs. COCO models: sheep=19, dog=18, but custom livestock models vary.
                
                # Get bounding box coordinates
                bbox_xyxy = box.xyxy[0].cpu().numpy().tolist()
                
                # Get confidence
                confidence = float(box.conf.item())
                
                # Get track ID if available
                track_id = None
                if self.track_enabled and box.id is not None:
                    track_id = int(box.id.item())
                
                detection = {
                    'bbox_xyxy': bbox_xyxy,
                    'confidence': confidence,
                    'class_id': class_id,
                    'track_id': track_id
                }
                
                detections.append(detection)
        
        logger.debug(f"Frame {frame_index}: Detected {len(detections)} objects")
        
        return detections
    
    def detect_video(
        self,
        video_path: Path,
        max_frames: Optional[int] = None
    ) -> List[Tuple[int, np.ndarray, List[Dict[str, Any]]]]:
        """
        Detect sheep in a video file.
        
        Args:
            video_path: Path to video file
            max_frames: Maximum number of frames to process (None = all)
            
        Returns:
            List of tuples: (frame_index, frame, detections)
        """
        video_path = Path(video_path)
        
        if not video_path.exists():
            raise FileNotFoundError(f"Video not found: {video_path}")
        
        logger.info(f"Processing video: {video_path}")
        
        cap = cv2.VideoCapture(str(video_path))
        
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open video: {video_path}")
        
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        
        logger.info(f"Video: {total_frames} frames @ {fps:.2f} FPS")
        
        results = []
        frame_index = 0
        
        while True:
            ret, frame = cap.read()
            
            if not ret:
                break
            
            # Run detection
            detections = self.detect_frame(frame, frame_index)
            
            results.append((frame_index, frame, detections))
            
            frame_index += 1
            
            # Check max frames limit
            if max_frames is not None and frame_index >= max_frames:
                logger.info(f"Reached max_frames limit: {max_frames}")
                break
            
            # Progress logging
            if frame_index % 100 == 0:
                logger.info(f"Processed {frame_index}/{total_frames} frames")
        
        cap.release()
        
        logger.info(f"Detection complete: {frame_index} frames processed")
        
        return results


def filter_detections_by_class(
    detections: List[Dict[str, Any]],
    class_names: List[str],
    model: YOLO
) -> List[Dict[str, Any]]:
    """
    Filter detections by class name.
    
    Args:
        detections: List of detection dicts
        class_names: List of class names to keep (e.g., ['sheep', 'dog'])
        model: YOLO model (to access class names)
        
    Returns:
        Filtered list of detections
    """
    # Get model class names
    model_classes = model.names  # Dict: {class_id: class_name}
    
    # Build set of class IDs to keep
    keep_class_ids = set()
    for class_id, class_name in model_classes.items():
        if class_name.lower() in [cn.lower() for cn in class_names]:
            keep_class_ids.add(class_id)
    
    # Filter detections
    filtered = [d for d in detections if d['class_id'] in keep_class_ids]
    
    logger.debug(f"Filtered {len(detections)} → {len(filtered)} detections "
                f"(classes: {class_names})")
    
    return filtered
