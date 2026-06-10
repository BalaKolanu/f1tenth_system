#!/usr/bin/env python3
#
# BSD 3-Clause License
#
# Copyright (c) 2026
# All rights reserved.
#

import runpy
import weakref
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


def _install_callback_group_guard():
    original_can_execute = MutuallyExclusiveCallbackGroup.can_execute

    def safe_can_execute(self, entity):
        with self._lock:
            if weakref.ref(entity) not in self.entities:
                return False
            return self._active_entity is None

    if original_can_execute is not safe_can_execute:
        MutuallyExclusiveCallbackGroup.can_execute = safe_can_execute


def _rosbridge_script_path():
    prefix = Path(get_package_prefix("rosbridge_server"))
    script_path = prefix / "lib" / "rosbridge_server" / "rosbridge_websocket"
    if not script_path.exists():
        raise FileNotFoundError(f"Unable to find rosbridge_websocket at {script_path}")
    return script_path


def main():
    _install_callback_group_guard()
    runpy.run_path(str(_rosbridge_script_path()), run_name="__main__")


if __name__ == "__main__":
    main()
