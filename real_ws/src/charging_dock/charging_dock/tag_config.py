# Copyright 2026 akun
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass
import math
from typing import Dict, Iterable, Sequence


class TagConfigError(ValueError):
    """Raised when the configured tag registry is unsafe to use."""


@dataclass(frozen=True)
class TagSpec:
    """Immutable configuration for one physical AprilTag."""

    tag_id: int
    frame: str
    size_m: float


def normalize_family(family: str) -> str:
    """Return the family name used by apriltag_ros."""
    value = family.strip().lower()
    return value[3:] if value.startswith('tag') else value


def build_tag_registry(
    tag_ids: Sequence[int],
    frames: Sequence[str],
    sizes_m: Sequence[float],
) -> Dict[int, TagSpec]:
    """Validate parallel tag arrays and build an ID-indexed registry."""
    if not tag_ids:
        raise TagConfigError('tag.ids must contain at least one tag ID')
    if len(tag_ids) != len(frames) or len(tag_ids) != len(sizes_m):
        raise TagConfigError('tag.ids, tag.frames and tag.sizes must have equal lengths')

    normalized_ids = [int(tag_id) for tag_id in tag_ids]
    normalized_frames = [str(frame).strip() for frame in frames]
    normalized_sizes = [float(size) for size in sizes_m]

    if any(tag_id < 0 for tag_id in normalized_ids):
        raise TagConfigError('tag IDs must be non-negative')
    if len(set(normalized_ids)) != len(normalized_ids):
        raise TagConfigError('tag.ids contains duplicate IDs')
    if any(not frame for frame in normalized_frames):
        raise TagConfigError('tag frame names must not be empty')
    if len(set(normalized_frames)) != len(normalized_frames):
        raise TagConfigError('tag.frames contains duplicate frame names')
    if any(not math.isfinite(size) or size <= 0.0 for size in normalized_sizes):
        raise TagConfigError('tag sizes must be finite and greater than zero')

    return {
        tag_id: TagSpec(tag_id, frame, size)
        for tag_id, frame, size in zip(
            normalized_ids, normalized_frames, normalized_sizes
        )
    }


def duplicate_registered_ids(detections: Iterable, registered_ids: Iterable[int]):
    """Return registered IDs that occur more than once in one image."""
    registered = set(registered_ids)
    counts = {}
    for detection in detections:
        tag_id = int(detection.id)
        if tag_id in registered:
            counts[tag_id] = counts.get(tag_id, 0) + 1
    return {tag_id for tag_id, count in counts.items() if count > 1}
