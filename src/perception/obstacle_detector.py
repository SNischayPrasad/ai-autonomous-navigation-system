"""Computer-vision obstacle detector for the bird's-eye-view camera.

Pipeline (classic OpenCV, runs on CPU in ~1 ms):
    BGR image -> Gaussian blur -> HSV -> colour thresholds -> morphology
    -> contours -> filtered blobs -> world-frame detections

Static structure (walls, shelves, parked cars) and moving actors (people,
forklifts) are separated by colour class. The interface (`detect()` returning
`Detection` objects) is the same one a learned detector such as YOLO would
implement, so the classical detector can be swapped for a neural one later
without touching planning or control.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from src.config import PerceptionConfig
from src.simulation.sensors import CameraFrame

# HSV ranges (OpenCV hue is 0-179). Steel-blue static class, red dynamic class.
STATIC_HSV = ((90, 80, 80), (125, 255, 255))
DYNAMIC_HSV = [((0, 120, 120), (10, 255, 255)), ((170, 120, 120), (179, 255, 255))]


@dataclass
class Detection:
    label: str                 # "static" or "dynamic"
    center: np.ndarray         # world (x, y) [m]
    radius: float              # enclosing radius [m]
    bbox: tuple                # (x, y, w, h) in image pixels
    area_px: int
    confidence: float          # blob fill ratio, 0..1


@dataclass
class DetectionResult:
    detections: list = field(default_factory=list)
    static_mask: np.ndarray | None = None
    dynamic_mask: np.ndarray | None = None

    @property
    def dynamic(self) -> list:
        return [d for d in self.detections if d.label == "dynamic"]

    @property
    def static(self) -> list:
        return [d for d in self.detections if d.label == "static"]


class ObstacleDetector:
    def __init__(self, cfg: PerceptionConfig) -> None:
        self.cfg = cfg
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    def _clean(self, mask: np.ndarray) -> np.ndarray:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)   # remove speckle noise
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)  # fill small holes

    def detect(self, frame: CameraFrame) -> DetectionResult:
        blurred = cv2.GaussianBlur(frame.image, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        static_mask = self._clean(cv2.inRange(hsv, *STATIC_HSV))
        dynamic_mask = np.zeros_like(static_mask)
        for lo, hi in DYNAMIC_HSV:
            dynamic_mask |= cv2.inRange(hsv, lo, hi)
        dynamic_mask = self._clean(dynamic_mask)

        result = DetectionResult(static_mask=static_mask, dynamic_mask=dynamic_mask)
        for label, mask in (("dynamic", dynamic_mask), ("static", static_mask)):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < self.cfg.min_blob_area_px:
                    continue
                (cx, cy), r = cv2.minEnclosingCircle(cnt)
                fill = float(area / (np.pi * r * r + 1e-6))
                result.detections.append(Detection(
                    label=label,
                    center=frame.pixel_to_world(cx, cy),
                    radius=float(r * frame.m_per_px),
                    bbox=cv2.boundingRect(cnt),
                    area_px=int(area),
                    confidence=min(1.0, fill),
                ))
        return result

    @staticmethod
    def annotate(frame: CameraFrame, result: DetectionResult) -> np.ndarray:
        """Draw bounding boxes and labels on a copy of the camera image."""
        img = frame.image.copy()
        for det in result.detections:
            x, y, w, h = det.bbox
            if det.label == "dynamic":
                cv2.rectangle(img, (x - 2, y - 2), (x + w + 2, y + h + 2), (0, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(img, f"MOVING {det.confidence:.2f}", (x, max(10, y - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 255), 1, cv2.LINE_AA)
            else:
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 220, 120), 1, cv2.LINE_AA)
        c = img.shape[0] // 2
        cv2.drawMarker(img, (c, c), (255, 255, 255), cv2.MARKER_CROSS, 10, 1)
        return img
