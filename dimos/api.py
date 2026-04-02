# Copyright 2025-2026 Dimensional Inc.
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

"""DimOS Python API — top-level entry point.

Everything flows through the :class:`ModuleCoordinator` returned by
:func:`run`.  Modules expose streams (topics), skills, and RPCs.

Usage::

    from dimos.api import run

    app = run("unitree-go2-basic", simulation=True)

    # Discover what's available
    app.modules                      # → {name: ModuleProxy, ...}
    app.skills                       # → {name: SkillInfo, ...}
    app.topics                       # → {name: (type, [modules]), ...}

    # Access a module and its capabilities
    conn = app["GO2Connection"]
    image = conn.color_image.get_next(timeout=5.0)

    # Invoke a skill
    app["PersonFollowSkillContainer"].follow_person("person in blue")

    # Standalone topic access (tap into already-running system)
    from dimos.api import topic_collect, topic_publish
    msgs = topic_collect("/color_image", duration=3.0)

    app.stop()
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.core.module import SkillInfo
    from dimos.core.module_coordinator import ModuleCoordinator
    from dimos.core.rpc_client import ModuleProxy


class App:
    """Handle to a running DimOS blueprint.

    Wraps :class:`ModuleCoordinator` with name-based module access,
    skill discovery, and topic introspection.
    """

    def __init__(self, coordinator: ModuleCoordinator) -> None:
        self._coordinator = coordinator

    # ── Module access ──

    @property
    def modules(self) -> dict[str, ModuleProxy]:
        """All deployed modules keyed by class name."""
        return {cls.__name__: proxy for cls, proxy in self._coordinator._deployed_modules.items()}

    def __getitem__(self, name: str) -> ModuleProxy:
        """Get a module by class name.  ``app["GO2Connection"]``."""
        for cls, proxy in self._coordinator._deployed_modules.items():
            if cls.__name__ == name:
                return proxy  # type: ignore[return-value]
        raise KeyError(f"No module '{name}'. Available: {list(self.modules)}")

    def __getattr__(self, name: str) -> ModuleProxy:
        """Get a module as attribute.  ``app.GO2Connection``."""
        if name.startswith("_"):
            raise AttributeError(name)
        try:
            return self[name]
        except KeyError:
            raise AttributeError(f"No module '{name}'. Available: {list(self.modules)}") from None

    # ── Skill discovery ──

    @property
    def skills(self) -> dict[str, SkillInfo]:
        """All skills across all deployed modules."""
        result: dict[str, SkillInfo] = {}
        for proxy in self._coordinator._deployed_modules.values():
            try:
                for info in proxy.get_skills():
                    result[info.func_name] = info
            except Exception:
                continue
        return result

    # ── Topic introspection ──

    @property
    def topics(self) -> dict[str, type]:
        """Active topic names and their message types.

        Built from the coordinator's stream wiring — no LCM bus scan needed.
        """
        result: dict[str, type] = {}
        for _cls, proxy in self._coordinator._deployed_modules.items():
            for name, stream in _get_streams(proxy):
                if hasattr(stream, "type"):
                    result[name] = stream.type
        return result

    # ── Lifecycle ──

    def stop(self) -> None:
        """Stop all modules and shut down workers."""
        self._coordinator.stop()


def _get_streams(proxy: Any) -> list[tuple[str, Any]]:
    """Extract stream name/object pairs from a module proxy."""
    from dimos.core.stream import In, Out

    streams = []
    for attr_name in dir(proxy):
        if attr_name.startswith("_"):
            continue
        try:
            attr = getattr(proxy, attr_name)
            if isinstance(attr, (In, Out)):
                streams.append((attr_name, attr))
        except Exception:
            continue
    return streams


def run(target: str, **config: Any) -> App:
    """Start a blueprint by name and return an :class:`App` handle.

    Args:
        target: Blueprint or module name (e.g. ``"unitree-go2-basic"``).
        **config: GlobalConfig overrides (e.g. ``simulation=True``).

    Returns:
        App wrapping the running ModuleCoordinator.
    """
    from dimos.core.blueprints import Blueprint
    from dimos.robot.get_all_blueprints import get_by_name

    if isinstance(target, str):
        blueprint = get_by_name(target)
    elif isinstance(target, Blueprint):
        blueprint = target
    else:
        raise TypeError(f"Expected blueprint name or Blueprint, got {type(target)}")

    coordinator = blueprint.build(cli_config_overrides=config or {})
    return App(coordinator)


def list_blueprints() -> list[str]:
    """List all registered blueprint and module names."""
    from dimos.robot.all_blueprints import all_blueprints, all_modules

    return sorted(set(all_blueprints.keys()) | set(all_modules.keys()))


# ── Standalone topic access (diagnostic / external process) ──

from dimos.robot.cli.topic import topic_collect, topic_publish

__all__ = [
    "App",
    "list_blueprints",
    "run",
    "topic_collect",
    "topic_publish",
]
