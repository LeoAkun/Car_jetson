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

from types import SimpleNamespace

import pytest

from charging_dock.tag_config import build_tag_registry
from charging_dock.tag_config import duplicate_registered_ids
from charging_dock.tag_config import normalize_family
from charging_dock.tag_config import TagConfigError


def test_build_tag_registry():
    registry = build_tag_registry([11, 12], ['dock_tag_11', 'dock_tag_12'], [0.16, 0.12])

    assert registry[11].frame == 'dock_tag_11'
    assert registry[12].size_m == pytest.approx(0.12)


@pytest.mark.parametrize(
    'ids,frames,sizes',
    [
        ([], [], []),
        ([11], ['dock_tag_11', 'dock_tag_12'], [0.16]),
        ([11, 11], ['dock_tag_11', 'dock_tag_12'], [0.16, 0.16]),
        ([11, 12], ['dock_tag_11', 'dock_tag_11'], [0.16, 0.16]),
        ([-1], ['dock_tag_invalid'], [0.16]),
        ([11], [''], [0.16]),
        ([11], ['dock_tag_11'], [0.0]),
        ([11], ['dock_tag_11'], [float('nan')]),
    ],
)
def test_invalid_tag_registry_is_rejected(ids, frames, sizes):
    with pytest.raises(TagConfigError):
        build_tag_registry(ids, frames, sizes)


def test_family_normalization():
    assert normalize_family('tag36h11') == '36h11'
    assert normalize_family(' 36H11 ') == '36h11'


def test_duplicate_registered_ids_only_reports_known_tags():
    detections = [
        SimpleNamespace(id=11),
        SimpleNamespace(id=11),
        SimpleNamespace(id=99),
        SimpleNamespace(id=99),
    ]

    assert duplicate_registered_ids(detections, [11, 12]) == {11}
