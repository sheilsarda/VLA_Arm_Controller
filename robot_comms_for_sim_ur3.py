'''
Below are a list of functions that you may use to get the described information from the robot. The internals of this code are kept hidden.
You may fully rely on the descriptions of each function to accurately convey what is being returned by the function itself.
'''

from typing import List, Optional
from socket import socket, AF_INET, SOCK_STREAM
import struct
import threading
import time

#############################################
##
## UR Robot Real-Time Interface Connection
##
#############################################

ROBOT_IP = "192.168.40.128"
REALTIME_PORT = 30003  # Real-time interface - streams state at 125Hz

# UR e-Series Real-time packet structure offsets (in bytes)
# Packet size: 1116 bytes for e-Series
PACKET_SIZE = 1116
TCP_POSE_OFFSET = 444      # actual_TCP_pose: 6 doubles (48 bytes)
TCP_SPEED_OFFSET = 492     # actual_TCP_speed: 6 doubles (48 bytes)


class URRobotState:
    """Singleton class to manage persistent connection to UR robot real-time interface."""
    
    _instance: Optional['URRobotState'] = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    instance = super().__new__(cls)
                    instance._initialized = False
                    instance._socket = None
                    instance._connected = False
                    instance._tcp_pose = [0.0] * 6
                    instance._tcp_speed = [0.0] * 6
                    instance._data_lock = threading.Lock()
                    instance._reader_thread = None
                    instance._running = False
                    cls._instance = instance
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
        self._initialized = True
    
    def connect(self) -> bool:
        """Connect to the UR robot real-time interface."""
        if self._connected:
            return True
        
        try:
            self._socket = socket(AF_INET, SOCK_STREAM)
            self._socket.settimeout(5.0)
            self._socket.connect((ROBOT_IP, REALTIME_PORT))
            self._connected = True
            self._running = True
            
            # Start background thread to continuously read robot state
            self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._reader_thread.start()
            
            # Wait a moment for first data to arrive
            time.sleep(0.1)
            print(f"Connected to UR robot at {ROBOT_IP}:{REALTIME_PORT}")
            return True
            
        except Exception as e:
            print(f"Failed to connect to robot real-time interface: {e}")
            self._connected = False
            return False
    
    def disconnect(self):
        """Disconnect from the robot."""
        self._running = False
        if self._socket:
            try:
                self._socket.close()
            except:
                pass
        self._socket = None
        self._connected = False
    
    def _read_loop(self):
        """Background thread that continuously reads and parses robot state."""
        while self._running and self._socket:
            try:
                # Read exactly one packet
                data = self._receive_exact(PACKET_SIZE)
                if data and len(data) == PACKET_SIZE:
                    self._parse_packet(data)
            except Exception as e:
                if self._running:
                    print(f"Error reading robot state: {e}")
                    time.sleep(0.1)
    
    def _receive_exact(self, num_bytes: int) -> Optional[bytes]:
        """Receive exactly num_bytes from the socket."""
        data = b''
        while len(data) < num_bytes and self._running:
            try:
                chunk = self._socket.recv(num_bytes - len(data))
                if not chunk:
                    return None
                data += chunk
            except:
                return None
        return data
    
    def _parse_packet(self, data: bytes):
        """Parse the real-time data packet and extract TCP pose and speed."""
        try:
            # Extract TCP pose (6 doubles starting at TCP_POSE_OFFSET)
            # UR uses big-endian format
            tcp_pose = struct.unpack('>6d', data[TCP_POSE_OFFSET:TCP_POSE_OFFSET + 48])
            
            # Extract TCP speed (6 doubles starting at TCP_SPEED_OFFSET)
            tcp_speed = struct.unpack('>6d', data[TCP_SPEED_OFFSET:TCP_SPEED_OFFSET + 48])
            
            with self._data_lock:
                self._tcp_pose = list(tcp_pose)
                self._tcp_speed = list(tcp_speed)
                
        except struct.error as e:
            print(f"Error parsing packet: {e}")
    
    def get_tcp_pose(self) -> List[float]:
        """Get the current TCP pose [x, y, z, rx, ry, rz] in meters and radians."""
        if not self._connected:
            self.connect()
        
        with self._data_lock:
            return self._tcp_pose.copy()
    
    def get_tcp_speed(self) -> List[float]:
        """Get the current TCP speed [dx, dy, dz, drx, dry, drz] in m/s and rad/s."""
        if not self._connected:
            self.connect()
        
        with self._data_lock:
            return self._tcp_speed.copy()


# Global robot state instance
_robot_state = URRobotState()


#############################################
##
## Robot Arm Low-Level Functions
##
#############################################


### Cartesian Functions

# Position

def get_base_cartesian_pose() -> List[float]:
    '''
    Returns a list of 6 floats describing the cartesian TCP pose (x, y, z, rx, ry, rz) 
    in the robot base frame.
    
    Units:
    - x, y, z: meters
    - rx, ry, rz: radians (axis-angle rotation vector)
    
    Connects to UR robot real-time interface (port 30003) to get live data.
    '''
    return _robot_state.get_tcp_pose()


def move_to_base_cartesian_position(x: int, y: int, z: int, rx: int, ry: int, rz: int) -> None:
    '''Moves the robot to the specified position (x, y, z, rx, ry, rz) in base frame'''

# Velocity

def get_base_cartesian_velocity() -> List[float]:
    '''
    Returns a list of 6 floats describing the cartesian TCP velocity (dx, dy, dz, drx, dry, drz)
    in the robot base frame.
    
    Units:
    - dx, dy, dz: meters/second
    - drx, dry, drz: radians/second
    
    Connects to UR robot real-time interface (port 30003) to get live data.
    '''
    return _robot_state.get_tcp_speed()


def set_base_cartesian_velocity(cartesian_velocity: List[int]) -> None:
    '''Takes in a list of velocities for each cartesian value (in base frame) (x, y, z, rx, ry, rz) and sets each position to the specified velocity value'''


# Acceleration

def get_base_cartesian_acceleration() -> List[int]:
    '''Returns a list of 6 integers, each describing the (x, y, z, rx, ry, rz) acceleration of the robot.'''
    return [1, 1, 1, 1, 1, 1]


def set_base_cartesian_acceleration(cartesian_acceleration: List[int]) -> None:
    '''Takes in a list of accelerations for each cartesian value (in base frame) (x, y, z, rx, ry, rz) and sets each position to the specified acceleration value'''

# Force

def get_cartesian_base_force() -> List[int]:
    '''Returns a list of 6 integers, each describing the force felt by the base of the robot in Newtons. Force is described by: (x, y, z, rx, ry, rz)'''
    return [1, 1, 1, 1, 1, 1]


def set_cartesian_base_force(cartesian_force: List[int]) -> None:
    '''Sets the amount of force the robot should exert on the object in the base of the robot. Force is input as: (x, y, z, rz, ry, rz) all in Newtons'''


### Joint Functions

# Position

def get_angular_pose() -> List[int]:
    '''Returns a list of 6 integers, each describing the angular (joint 1, joint 2, joint 3, joint 4, joint 5, joint 6) position of the robot.'''
    return [1, 1, 1, 1, 1, 1]


def move_to_angular_position(j1: int, j2: int, j3: int, j4: int, j5: int, j6: int) -> None:
    '''Moves the robot to the specified position (j1, j2, j3, j4, j5, j6)'''


# Velocity

def get_angular_velocity() -> List[int]:
    '''Returns a list of 6 integers, each describing the angular (joint 1, joint 2, joint 3, joint 4, joint 5, joint 6) velocity of the robot.'''
    return [1, 1, 1, 1, 1, 1]

def set_angular_velocity(joint_velocity: List[int]) -> None:
    '''Takes in a list of velocities for each joint and sets each joint to the specified velocity value'''


# Accelerations

def get_angular_acceleration() -> List[int]:
    '''Returns a list of 6 integers, each describing the angular (joint 1, joint 2, joint 3, joint 4, joint 5, joint 6) acceleration of the robot.'''
    return [1, 1, 1, 1, 1, 1]


def set_angular_acceleration(joint_acceleration: List[int]) -> None:
    '''Takes in a list of accelerations for each joint and sets each joint to the specified acceleration value'''


# End of Arm Tool Power

def power_on_tool() -> None:
    '''Sends power to the end of arm tool to be used'''


#############################################
##
## End of Arm Tool Low-Level Functions
##
#############################################


def get_tool_voltage() -> int:
    '''Returns 24 if the tool is powered with 24 V. Returns 0 if the tool is not powered on.'''


def engage_tool() -> int:
    '''Engages the tool'''


def disengage_tool() -> int:
    '''Disengages the tool'''


#############################################
##
## Robot Rail Low-Level Driver
##
#############################################


def get_rail_pose() -> int:
    '''Returns an integer that describes the rail position of the robot'''
    return 1


def move_to_rail_pose(rail_pose: int) -> None:
    '''Moves the robot to the specified position on the rail'''


def set_rail_speed(speed: int) -> None:
    '''Sets the speed of the rail'''


def get_rail_speed() -> int:
    '''Gets the speed of the rail'''
