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

from charging_dock.command_mux import CommandMux, select_source


def test_mux_module_exports_node():
    assert issubclass(CommandMux, object)


def test_dock_timeout_never_falls_back_to_navigation():
    assert select_source(True, False, True) == 'none'


def test_navigation_is_selected_only_after_dock_control_is_released():
    assert select_source(False, False, True) == 'nav'
    assert select_source(True, True, True) == 'dock'
