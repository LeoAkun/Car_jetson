#!/usr/bin/env python3

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

"""Installed entry point for the Stage 3 camera checker."""

from importlib.util import module_from_spec
from importlib.util import spec_from_file_location
from pathlib import Path


def main():
    """Run the source-tree camera checker."""
    source = (
        Path(__file__).resolve().parents[1]
        / 'tools'
        / 'stage3_camera_check.py'
    )
    spec = spec_from_file_location('charging_dock_stage3_camera_check', source)
    if spec is None or spec.loader is None:
        raise RuntimeError('stage3 camera checker source is unavailable')
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.main()
