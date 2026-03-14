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

"""Mermaid diagram renderer for blueprint visualization.

Generates a Mermaid flowchart with direct labelled edges between modules:

    ModuleA -- "name:Type" --> ModuleB
"""

from __future__ import annotations

from collections import defaultdict
from hashlib import md5

from dimos.core.blueprints import Blueprint
from dimos.core.module import Module

# Colour palettes
_NODE_COLORS = [
    "#2d6a4f", "#1b4965", "#5a189a", "#6d3a0a", "#3d405b",
    "#264653", "#4a3f6b", "#1a535c", "#4e4187", "#2c514c",
]
_EDGE_COLORS = [
    "#4cc9f0",  # sky blue
    "#f77f00",  # orange
    "#80ed99",  # mint green
    "#c77dff",  # lavender
    "#ffd166",  # gold
    "#ef476f",  # coral red
    "#06d6a0",  # teal
    "#3a86ff",  # bright blue
    "#ff9e00",  # amber
    "#e5383b",  # red
    "#2ec4b6",  # cyan-teal
    "#9b5de5",  # purple
    "#00f5d4",  # aquamarine
    "#fee440",  # yellow
    "#f15bb5",  # magenta
    "#00bbf9",  # cerulean
    "#8ac926",  # lime green
    "#ff595e",  # salmon
    "#1982c4",  # steel blue
    "#ffca3a",  # sunflower
]


def _pick_color(palette: list[str], key: str) -> str:
    """Deterministically pick a colour from *palette* based on *key*."""
    idx = int(md5(key.encode()).hexdigest(), 16) % len(palette)
    return palette[idx]


# Connections to ignore (too noisy/common)
DEFAULT_IGNORED_CONNECTIONS = {("odom", "PoseStamped")}

DEFAULT_IGNORED_MODULES = {
    "WebsocketVisModule",
}

_COMPACT_ONLY_IGNORED_MODULES = {
    "WebsocketVisModule",
}


def _mermaid_id(name: str) -> str:
    """Sanitize a string into a valid Mermaid node id."""
    return name.replace(" ", "_").replace("-", "_")


def render(
    blueprint_set: Blueprint,
    *,
    ignored_streams: set[tuple[str, str]] | None = None,
    ignored_modules: set[str] | None = None,
    show_disconnected: bool = False,
) -> tuple[str, dict[str, str]]:
    """Generate a Mermaid flowchart from a Blueprint.

    Returns ``(mermaid_code, label_color_map)`` where *label_color_map* maps
    each edge label string to its hex colour.
    """
    if ignored_streams is None:
        ignored_streams = DEFAULT_IGNORED_CONNECTIONS
    if ignored_modules is None:
        if show_disconnected:
            ignored_modules = DEFAULT_IGNORED_MODULES - _COMPACT_ONLY_IGNORED_MODULES
        else:
            ignored_modules = DEFAULT_IGNORED_MODULES

    # Collect producers/consumers
    producers: dict[tuple[str, type], list[type[Module]]] = defaultdict(list)
    consumers: dict[tuple[str, type], list[type[Module]]] = defaultdict(list)
    module_names: set[str] = set()

    for bp in blueprint_set.blueprints:
        if bp.module.__name__ in ignored_modules:
            continue
        module_names.add(bp.module.__name__)
        for conn in bp.streams:
            remapped_name = blueprint_set.remapping_map.get(
                (bp.module, conn.name), conn.name
            )
            key = (remapped_name, conn.type)
            if conn.direction == "out":
                producers[key].append(bp.module)
            else:
                consumers[key].append(bp.module)

    # Active channels: both producer and consumer exist
    active_keys: list[tuple[str, type]] = []
    for key in producers:
        name, type_ = key
        if key not in consumers:
            continue
        if (name, type_.__name__) in ignored_streams:
            continue
        valid_p = [m for m in producers[key] if m.__name__ not in ignored_modules]
        valid_c = [m for m in consumers[key] if m.__name__ not in ignored_modules]
        if valid_p and valid_c:
            active_keys.append(key)

    # Disconnected channels
    disconnected_keys: list[tuple[str, type]] = []
    if show_disconnected:
        all_keys = set(producers.keys()) | set(consumers.keys())
        for key in all_keys:
            if key in active_keys:
                continue
            name, type_ = key
            if (name, type_.__name__) in ignored_streams:
                continue
            relevant = producers.get(key, []) + consumers.get(key, [])
            if all(m.__name__ in ignored_modules for m in relevant):
                continue
            disconnected_keys.append(key)

    lines = ["graph LR"]

    # Declare module nodes with rounded boxes
    sorted_modules = sorted(module_names)
    for mod_name in sorted_modules:
        mid = _mermaid_id(mod_name)
        lines.append(f"    {mid}([{mod_name}])")

    lines.append("")

    # Active edges (track index for linkStyle)
    edge_idx = 0
    edge_colors: list[str] = []
    label_color_map: dict[str, str] = {}

    for key in sorted(active_keys, key=lambda k: f"{k[0]}:{k[1].__name__}"):
        name, type_ = key
        label = f"{name}:{type_.__name__}"
        color = _pick_color(_EDGE_COLORS, label)
        label_color_map[label] = color
        for prod in producers[key]:
            if prod.__name__ in ignored_modules:
                continue
            for cons in consumers[key]:
                if cons.__name__ in ignored_modules:
                    continue
                pid = _mermaid_id(prod.__name__)
                cid = _mermaid_id(cons.__name__)
                lines.append(f'    {pid} -->|"{label}"| {cid}')
                edge_colors.append(color)
                edge_idx += 1

    # Disconnected edges
    if disconnected_keys:
        lines.append("")
        lines.append("    %% Disconnected streams")
        stub_counter = 0
        for key in sorted(disconnected_keys, key=lambda k: f"{k[0]}:{k[1].__name__}"):
            name, type_ = key
            label = f"{name}:{type_.__name__}"
            color = _pick_color(_EDGE_COLORS, label)
            label_color_map[label] = color
            stub_id = f"stub{stub_counter}"
            stub_counter += 1
            lines.append(f"    {stub_id}(( ))")
            lines.append(f"    style {stub_id} fill:#555,stroke:#888,stroke-width:1px")

            for prod in producers.get(key, []):
                if prod.__name__ in ignored_modules:
                    continue
                pid = _mermaid_id(prod.__name__)
                lines.append(f'    {pid} -.->|"{label}"| {stub_id}')
                edge_colors.append(color)
                edge_idx += 1
            for cons in consumers.get(key, []):
                if cons.__name__ in ignored_modules:
                    continue
                cid = _mermaid_id(cons.__name__)
                lines.append(f'    {stub_id} -.->|"{label}"| {cid}')
                edge_colors.append(color)
                edge_idx += 1

    # Node styles
    lines.append("")
    for mod_name in sorted_modules:
        mid = _mermaid_id(mod_name)
        c = _pick_color(_NODE_COLORS, mod_name)
        lines.append(
            f"    style {mid} fill:{c},stroke:{c},color:#eee,stroke-width:2px"
        )

    # Edge styles (one linkStyle per edge index)
    if edge_colors:
        lines.append("")
        for i, c in enumerate(edge_colors):
            lines.append(f"    linkStyle {i} stroke:{c},stroke-width:2px")

    return "\n".join(lines), label_color_map
