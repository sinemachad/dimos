#!/usr/bin/env python3

import time

from dimos import core
from dimos.msgs.sensor_msgs import JointState


def on_joint_state(msg: JointState) -> None:
    names = ", ".join(msg.name) if msg.name else "n/a"
    positions = ", ".join(f"{v:.4f}" for v in msg.position)
    print(f"names=[{names}] positions=[{positions}]")


def main() -> None:
    transport = core.LCMTransport("/xarm/joint_states", JointState)
    print("Transport created")
    transport.subscribe(on_joint_state)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        transport.stop()


if __name__ == "__main__":
    main()
