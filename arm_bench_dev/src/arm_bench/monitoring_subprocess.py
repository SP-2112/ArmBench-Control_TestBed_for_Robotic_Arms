#!/usr/bin/env python3
"""
Subprocess launcher for MonitoringInterface to avoid Tk() segfaults.
This runs in a completely separate process with fresh Tkinter state.
"""

import sys
import json
from pathlib import Path


def main():
    """Launch monitoring interface in fresh subprocess"""
    if len(sys.argv) < 4:
        print("Usage: monitoring_subprocess.py <mode> <motors_json> <cameras_json>")
        sys.exit(1)
    
    mode = sys.argv[1]
    motors_json = sys.argv[2]
    cameras_json = sys.argv[3]
    
    # Parse motors and cameras from JSON strings
    motors = json.loads(motors_json)
    cameras = json.loads(cameras_json)
    
    # Import gui module (fresh Python process = fresh Tkinter state)
    from arm_bench.gui import MonitoringInterface
    
    print("Opening monitoring dashboard window...")
    print("(Close the GUI window to exit)")
    
    # Create and run monitoring interface
    try:
        monitor = MonitoringInterface(mode, motors, cameras)
        print(" Monitoring interface is now running")
        monitor.root.mainloop()
        print("Monitoring interface closed")
    except Exception as e:
        print(f"Error in monitoring interface: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
