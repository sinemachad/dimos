#!/usr/bin/env python3
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

"""
Simple test script to verify ArduPilot setup and basic functionality.
"""

import sys
import os
import glob
from pathlib import Path


def test_imports():
    """Test that all required modules can be imported."""
    print("Testing imports...")

    try:
        import numpy as np

        print("✓ NumPy imported successfully")
    except ImportError as e:
        print(f"✗ NumPy import failed: {e}")
        return False

    try:
        from pymavlink import mavutil

        print("✓ pymavlink imported successfully")
        print(f"  Version: {mavutil.VERSION if hasattr(mavutil, 'VERSION') else 'Unknown'}")
    except ImportError as e:
        print(f"✗ pymavlink import failed: {e}")
        print("  Please install pymavlink: pip install pymavlink")
        return False

    try:
        from reactivex import interval
        from reactivex import operators as ops

        print("✓ ReactiveX imported successfully")
    except ImportError as e:
        print(f"✗ ReactiveX import failed: {e}")
        print("  Please install reactivex: pip install reactivex")
        return False

    try:
        from dimos.core import Module, Out, In, rpc

        print("✓ DIMOS core imported successfully")
    except ImportError as e:
        print(f"✗ DIMOS core import failed: {e}")
        return False

    try:
        from dimos.hardware.ardupilot import ArduPilotModule, ArduPilotInterface

        print("✓ ArduPilot module imported successfully")
    except ImportError as e:
        print(f"✗ ArduPilot module import failed: {e}")
        return False

    # Test LCM message imports
    try:
        from dimos_lcm.sensor_msgs import NavSatFix, NavSatStatus, Imu
        from dimos_lcm.nav_msgs import Odometry, Path
        from dimos_lcm.geometry_msgs import PoseStamped, Twist

        print("✓ Standard LCM messages imported successfully")
    except ImportError as e:
        print(f"✗ Standard LCM messages import failed: {e}")
        return False

    # Test optional MAVROS messages
    try:
        from dimos_lcm.mavros_msgs import State, ExtendedState, ActuatorControl

        print("✓ MAVROS LCM messages imported successfully")
    except ImportError as e:
        print(f"⚠️  MAVROS LCM messages not available: {e}")
        print("  Extended functionality will be limited")

    return True


def test_hardware_detection():
    """Test if ArduPilot hardware is detected."""
    print("\nTesting hardware detection...")

    # Common ArduPilot device paths
    device_patterns = [
        "/dev/ttyACM*",  # USB ACM devices
        "/dev/ttyUSB*",  # USB serial devices
        "/dev/cu.usbmodem*",  # macOS
        "/dev/cu.usbserial*",  # macOS
    ]

    found_devices = []
    for pattern in device_patterns:
        devices = glob.glob(pattern)
        found_devices.extend(devices)

    if found_devices:
        print(f"Found {len(found_devices)} potential ArduPilot device(s):")
        for device in found_devices:
            # Check if device is readable
            try:
                if os.access(device, os.R_OK | os.W_OK):
                    print(f"  ✓ {device} (readable/writable)")
                else:
                    print(f"  ⚠️  {device} (permission denied)")
            except Exception:
                print(f"  ? {device} (unknown status)")
    else:
        print("No ArduPilot devices detected")
        print("Expected device paths:")
        for pattern in device_patterns:
            print(f"  {pattern}")

    # Check for SITL (Software In The Loop) option
    print("\nChecking for SITL option:")
    try:
        import socket

        # Test if we can connect to common SITL ports
        sitl_addresses = [
            ("127.0.0.1", 14550),  # UDP SITL
            ("127.0.0.1", 5760),  # TCP SITL
        ]

        for addr, port in sitl_addresses:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(1)
                result = sock.connect_ex((addr, port))
                sock.close()
                if result == 0:
                    print(f"  ✓ SITL detected on {addr}:{port}")
                    found_devices.append(f"tcp:{addr}:{port}")
                else:
                    print(f"  - No SITL on {addr}:{port}")
            except Exception:
                print(f"  ? Could not test {addr}:{port}")
    except Exception as e:
        print(f"  Error testing SITL: {e}")

    return len(found_devices) > 0, found_devices


def test_basic_functionality():
    """Test basic ArduPilot functionality using the module's test setup."""
    print("\nTesting basic functionality...")

    try:
        from dimos.hardware.ardupilot import ArduPilotModule, MAVROS_MSGS_AVAILABLE
        from dimos_lcm.sensor_msgs import NavSatFix
        from dimos_lcm.nav_msgs import Odometry, Path
        from dimos import core

        print("✓ All required imports successful")

        # Initialize DIMOS
        dimos = core.start(1)
        print("✓ DIMOS core started successfully")

        # Deploy ArduPilot module
        ardupilot = dimos.deploy(
            ArduPilotModule, connection_string="/dev/ttyACM0", publish_rate=10.0
        )
        print("✓ ArduPilotModule deployed successfully")

        # Test that module has expected outputs
        expected_outputs = ["global_position", "local_odom", "mission_waypoints"]
        for output in expected_outputs:
            if hasattr(ardupilot, output):
                print(f"  ✓ {output} output available")
            else:
                print(f"  ✗ {output} output missing")
                return False

        # Configure LCM transports (this tests the transport system)
        try:
            ardupilot.global_position.transport = core.LCMTransport(
                "/mavros/global_position/global", NavSatFix
            )
            ardupilot.local_odom.transport = core.LCMTransport(
                "/mavros/local_position/odom", Odometry
            )
            ardupilot.mission_waypoints.transport = core.LCMTransport(
                "/mavros/mission/waypoints", Path
            )
            print("✓ LCM transports configured successfully")
        except Exception as e:
            print(f"✗ LCM transport configuration failed: {e}")
            return False

        # Test MAVROS functionality
        if MAVROS_MSGS_AVAILABLE:
            print("✓ MAVROS messages available - extended functionality enabled")
            # Test optional MAVROS outputs
            mavros_outputs = ["state", "extended_state", "rc_in", "rc_out"]
            mavros_available = 0
            for output in mavros_outputs:
                if hasattr(ardupilot, output):
                    print(f"  ✓ {output} MAVROS output available")
                    mavros_available += 1
                else:
                    print(f"  - {output} MAVROS output not available")
        else:
            print("⚠️  MAVROS messages not available - using core functionality only")

        # Test module methods
        try:
            status = ardupilot.get_connection_status()
            print(f"✓ Connection status method works: {status}")
        except Exception as e:
            print(f"⚠️  Connection status method failed (expected without hardware): {e}")

        print("✓ Basic functionality test completed successfully")
        return True

    except Exception as e:
        print(f"✗ Basic functionality test failed: {e}")
        import traceback

        traceback.print_exc()
        return False


def test_connection_attempt(devices):
    """Test actual connection to ArduPilot (optional)."""
    if not devices:
        return True

    print("\nTesting connection (optional)...")
    print("This will attempt to connect to detected devices")

    # Ask user if they want to test connection
    response = input("Attempt connection test? [y/N]: ").lower().strip()
    if response != "y":
        print("Skipping connection test")
        return True

    from dimos.hardware.ardupilot import ArduPilotInterface

    for device in devices:
        print(f"\nTesting connection to {device}...")
        try:
            interface = ArduPilotInterface(connection_string=device)

            # Try to connect with timeout
            if interface.connect():
                print(f"✓ Successfully connected to {device}")

                # Test heartbeat
                interface.send_heartbeat()
                print("✓ Heartbeat sent successfully")

                # Disconnect
                interface.disconnect()
                print("✓ Disconnected successfully")
                return True
            else:
                print(f"✗ Failed to connect to {device}")

        except Exception as e:
            print(f"✗ Connection test failed for {device}: {e}")

    print("⚠️  No successful connections")
    return False


def main():
    """Run all tests."""
    print("ArduPilot Setup Test")
    print("=" * 50)

    # Test imports
    if not test_imports():
        print("\n❌ Import tests failed. Please install missing dependencies.")
        print("Required packages:")
        print("  pip install pymavlink reactivex numpy")
        return False

    # Test hardware detection
    hardware_found, devices = test_hardware_detection()
    if not hardware_found:
        print("\n⚠️  No ArduPilot hardware detected.")
        print("Options:")
        print("  1. Connect ArduPilot hardware via USB")
        print("  2. Run ArduPilot SITL (Software In The Loop)")
        print("  3. Use simulation data")

    # Test basic functionality
    if not test_basic_functionality():
        print("\n❌ Basic functionality tests failed.")
        return False

    # Optional connection test
    if hardware_found:
        test_connection_attempt(devices)

    print("\n" + "=" * 50)
    if hardware_found:
        print("✅ All tests passed! ArduPilot setup is ready.")
        print("\nExample usage:")
        print("  from dimos.hardware.ardupilot import ArduPilotModule")
        print("  from dimos import core")
        print("  ")
        print("  dimos = core.start(1)")
        print("  ardupilot = dimos.deploy(ArduPilotModule, connection_string='/dev/ttyACM0')")
        print("  ardupilot.start()")
    else:
        print("✅ Setup is ready, but no hardware detected.")
        print("\nTo test with SITL:")
        print("  1. Install ArduPilot SITL")
        print("  2. Run: sim_vehicle.py --vehicle=copter")
        print("  3. Use connection_string='udp:127.0.0.1:14550'")

    return True


if __name__ == "__main__":
    # Add the project root to Python path
    sys.path.append(str(Path(__file__).parent.parent))

    success = main()
    sys.exit(0 if success else 1)
