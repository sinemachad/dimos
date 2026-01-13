#!/usr/bin/env python3

import time

from dimos import core
from dimos.msgs.sensor_msgs import JointCommand, JointState

latest_state: JointState | None = None


def on_joint_state(msg: JointState) -> None:
    global latest_state
    latest_state = msg


def wait_for_state(timeout_s: float = 5.0) -> JointState | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if latest_state is not None:
            return latest_state
        time.sleep(0.05)
    return None


def main() -> None:
    state_sub = core.LCMTransport("/xarm/joint_states", JointState)
    cmd_pub = core.LCMTransport("/xarm/joint_position_command", JointCommand)
    state_sub.subscribe(on_joint_state)

    print("Waiting for /xarm/joint_states...")
    state = wait_for_state()
    if state is None:
        print("No joint state received; exiting.")
        state_sub.stop()
        return

    target = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    if len(state.position) != len(target):
        print(
            f"Expected {len(target)} joints, got {len(state.position)}. "
            "Update target size to match your arm."
        )
        state_sub.stop()
        cmd_pub.stop()
        return

    cmd_pub.broadcast(None, JointCommand(positions=target))
    print(f"sent /xarm/joint_position_command: {target}")

    timeout_s = 10.0
    threshold = 0.02
    deadline = time.monotonic() + timeout_s
    last_print = 0.0
    while time.monotonic() < deadline:
        state = latest_state
        if state is None:
            time.sleep(0.05)
            continue
        errors = [abs(a - b) for a, b in zip(state.position, target)]
        max_err = max(errors) if errors else float("inf")
        now = time.monotonic()
        if now - last_print >= 0.5:
            positions = ", ".join(f"{v:.4f}" for v in state.position)
            print(f"joint_states positions=[{positions}] max_err={max_err:.4f}")
            last_print = now
        if max_err <= threshold:
            print(f"target reached (max_err={max_err:.4f} <= {threshold:.2f})")
            break
        time.sleep(0.05)
    else:
        print("timeout waiting for target; check the robot and try again.")

    cmd_pub.stop()
    state_sub.stop()


if __name__ == "__main__":
    main()
