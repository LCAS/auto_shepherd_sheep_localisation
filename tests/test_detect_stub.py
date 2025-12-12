"""
Smoke tests for YOLO detector.

These tests verify that the detector can be initialized and run basic inference,
but don't require actual video files or trained models.
"""
import pytest
import numpy as np

from sheepgeo.yolo.detect import SheepDetector, filter_detections_by_class
from sheepgeo.config import DEFAULT_CONFIDENCE, DEFAULT_IOU_THRESHOLD


def test_detector_initialization():
    """Test that detector can be initialized."""
    # This will download yolov8n.pt if not present
    detector = SheepDetector(
        weights_path="yolov8n.pt",
        confidence=DEFAULT_CONFIDENCE,
        iou_threshold=DEFAULT_IOU_THRESHOLD
    )
    
    assert detector is not None
    assert detector.model is not None
    assert detector.confidence == DEFAULT_CONFIDENCE


def test_detector_on_blank_frame():
    """Test detection on a blank frame."""
    detector = SheepDetector(
        weights_path="yolov8n.pt",
        confidence=0.5
    )
    
    # Create a blank 640x480 BGR image
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Run detection (should not crash)
    detections = detector.detect_frame(frame, frame_index=0)
    
    # Should return a list (possibly empty)
    assert isinstance(detections, list)


def test_detector_on_random_frame():
    """Test detection on a random noise frame."""
    detector = SheepDetector(
        weights_path="yolov8n.pt",
        confidence=0.8  # high threshold to avoid random detections
    )
    
    # Create a random noise image
    frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    # Run detection
    detections = detector.detect_frame(frame, frame_index=0)
    
    # Should return a list
    assert isinstance(detections, list)
    
    # Each detection should have required fields
    for det in detections:
        assert 'bbox_xyxy' in det
        assert 'confidence' in det
        assert 'class_id' in det
        assert 'track_id' in det


def test_detector_with_tracking():
    """Test detector with tracking enabled."""
    detector = SheepDetector(
        weights_path="yolov8n.pt",
        track=True
    )
    
    # Process a few frames
    for i in range(3):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        detections = detector.detect_frame(frame, frame_index=i)
        assert isinstance(detections, list)


def test_detection_structure():
    """Test that detection dict has correct structure."""
    detector = SheepDetector(weights_path="yolov8n.pt")
    
    # Create a simple frame
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    
    detections = detector.detect_frame(frame)
    
    # Check structure
    for det in detections:
        assert isinstance(det, dict)
        assert 'bbox_xyxy' in det
        assert 'confidence' in det
        assert 'class_id' in det
        assert 'track_id' in det
        
        assert isinstance(det['bbox_xyxy'], list)
        assert len(det['bbox_xyxy']) == 4
        assert isinstance(det['confidence'], float)
        assert isinstance(det['class_id'], int)


def test_filter_detections_by_class():
    """Test filtering detections by class name."""
    # Create mock detections
    detections = [
        {'class_id': 0, 'confidence': 0.9, 'bbox_xyxy': [0, 0, 10, 10], 'track_id': None},
        {'class_id': 16, 'confidence': 0.8, 'bbox_xyxy': [20, 20, 30, 30], 'track_id': None},
        {'class_id': 18, 'confidence': 0.7, 'bbox_xyxy': [40, 40, 50, 50], 'track_id': None},
    ]
    
    # Load model to get class names
    from ultralytics import YOLO
    model = YOLO("yolov8n.pt")
    
    # Filter to 'dog' class (typically class 16 in COCO)
    filtered = filter_detections_by_class(detections, ['dog'], model)
    
    # Should only keep detections with class_id for 'dog'
    assert all(model.names[d['class_id']] == 'dog' for d in filtered)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
