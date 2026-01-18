"""
Test script to verify real-time communication with UR3e simulator.
Reads and displays TCP pose and velocity data from the robot.
"""

import time
from robot_communication import get_base_cartesian_pose, get_base_cartesian_velocity, _robot_state

def format_pose(pose):
    """Format pose values nicely."""
    x, y, z, rx, ry, rz = pose
    return (f"  Position: x={x:.4f}m, y={y:.4f}m, z={z:.4f}m\n"
            f"  Rotation: rx={rx:.4f}rad, ry={ry:.4f}rad, rz={rz:.4f}rad")

def format_velocity(vel):
    """Format velocity values nicely."""
    dx, dy, dz, drx, dry, drz = vel
    return (f"  Linear:  dx={dx:.4f}m/s, dy={dy:.4f}m/s, dz={dz:.4f}m/s\n"
            f"  Angular: drx={drx:.4f}rad/s, dry={dry:.4f}rad/s, drz={drz:.4f}rad/s")

def main():
    print("=" * 60)
    print("UR3e Real-Time Interface Test")
    print("=" * 60)
    print("\nConnecting to robot...")
    
    # Test reading pose and velocity
    try:
        pose = get_base_cartesian_pose()
        print("\n[OK] Connection successful!")
        
        print("\n" + "-" * 60)
        print("Reading robot state (press Ctrl+C to stop)...")
        print("-" * 60)
        
        while True:
            pose = get_base_cartesian_pose()
            velocity = get_base_cartesian_velocity()
            
            print(f"\n[{time.strftime('%H:%M:%S')}] TCP Pose:")
            print(format_pose(pose))
            print(f"\n[{time.strftime('%H:%M:%S')}] TCP Velocity:")
            print(format_velocity(velocity))
            
            print("-" * 40)
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        print("\nMake sure:")
        print("  1. URSim is running")
        print("  2. Robot IP is correct (currently: 192.168.40.128)")
        print("  3. Robot is powered on with brakes released")
    finally:
        _robot_state.disconnect()
        print("Disconnected from robot.")

if __name__ == "__main__":
    main()

