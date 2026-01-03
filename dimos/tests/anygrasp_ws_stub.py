# Copyright 2025 Dimensional Inc.
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

import argparse
import asyncio
import json

import numpy as np
import websockets


def make_grasps(points: np.ndarray, num: int = 8) -> list[dict]:
    """
    Very small stub that returns top-down grasps around the point-cloud centroid.
    Returns a list of dictionaries matching HostedGraspGenerator expected schema.
    """
    if points.size == 0:
        return []

    centroid = np.mean(points, axis=0)
    grasps = []

    # Base rotation: top-down (Z up), yaw variations
    for k in range(num):
        yaw = (2.0 * np.pi * k) / max(1, num)
        c, s = float(np.cos(yaw)), float(np.sin(yaw))
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        grasps.append(
            {
                "translation": centroid.tolist(),
                "rotation_matrix": Rz.tolist(),
                "width": 0.05,
                "height": 0.02,
                "depth": 0.02,
                "score": float(0.9 - 0.02 * k),
            }
        )
    return grasps


async def handler(websocket):
    async for message in websocket:
        try:
            req = json.loads(message)
            pts = np.asarray(req.get("points", []), dtype=float)
            if pts.ndim != 2 or pts.shape[1] != 3:
                await websocket.send(json.dumps({"error": "invalid points array"}))
                continue

            grasps = make_grasps(pts, num=12)
            await websocket.send(json.dumps(grasps))
        except Exception as e:
            await websocket.send(json.dumps({"error": str(e)}))


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=str, default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()

    async with websockets.serve(handler, args.host, args.port):
        print(f"AnyGrasp stub listening on ws://{args.host}:{args.port}")
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    asyncio.run(main())
