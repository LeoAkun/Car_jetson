"""Independent frame observations; no navigation or motion commands."""

from .detector import Detector, DetectorConfig, FrameContext, Observation

__all__ = ["Detector", "DetectorConfig", "FrameContext", "Observation"]
