'''
Below are a list of functions that you may use to get the described information from the robot. The internals of this code are kept hidden.
You may fully rely on the descriptions of each function to accurately convey what is being returned by the function itself.
'''

from typing import List, Optional, Union
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
REALTIME_PORT = 30003    # Real-time interface - streams state at 125Hz
PRIMARY_PORT = 30001     # Primary interface - for sending URScript commands

# UR e-Series Real-time packet structure offsets (in bytes)
# Packet size: 1116 bytes for e-Series
PACKET_SIZE = 1116

# Data offsets in the real-time packet (all values are big-endian doubles)
JOINT_POSITIONS_OFFSET = 252    # q_actual: 6 doubles (48 bytes) - joint positions in radians
JOINT_VELOCITIES_OFFSET = 300   # qd_actual: 6 doubles (48 bytes) - joint velocities in rad/s
JOINT_CURRENTS_OFFSET = 348     # I_actual: 6 doubles (48 bytes) - joint currents in A
TCP_POSE_OFFSET = 444           # actual_TCP_pose: 6 doubles (48 bytes) - [x,y,z,rx,ry,rz]
TCP_SPEED_OFFSET = 492          # actual_TCP_speed: 6 doubles (48 bytes)
TCP_FORCE_OFFSET = 540          # generalizedForcesFromFT: 6 doubles (48 bytes) - TCP forces/torques
JOINT_TEMPERATURES_OFFSET = 692 # Joint temperatures: 6 doubles (48 bytes)
TOOL_VOLTAGE_OFFSET = 740       # Tool voltage actual: 1 double (8 bytes)
SPEED_SCALING_OFFSET = 756      # Speed scaling: 1 double (8 bytes)

# Default motion parameters
DEFAULT_ACCELERATION = 1.2  # m/s^2 for linear, rad/s^2 for joint
DEFAULT_VELOCITY = 0.25     # m/s for linear, rad/s for joint


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
                    instance._rt_socket = None
                    instance._cmd_socket = None
                    instance._connected = False
                    instance._cmd_connected = False
                    
                    # Robot state data
                    instance._tcp_pose = [0.0] * 6
                    instance._tcp_speed = [0.0] * 6
                    instance._tcp_force = [0.0] * 6
                    instance._joint_positions = [0.0] * 6
                    instance._joint_velocities = [0.0] * 6
                    instance._joint_currents = [0.0] * 6
                    instance._joint_temperatures = [0.0] * 6
                    instance._tool_voltage = 0.0
                    instance._speed_scaling = 1.0
                    
                    # Motion parameters (stored for acceleration/velocity control)
                    instance._linear_acceleration = DEFAULT_ACCELERATION
                    instance._linear_velocity = DEFAULT_VELOCITY
                    instance._joint_accelerations = [DEFAULT_ACCELERATION] * 6
                    instance._joint_acceleration = DEFAULT_ACCELERATION
                    instance._joint_velocity = DEFAULT_VELOCITY
                    instance._joint_velocity_cmd = [DEFAULT_VELOCITY] * 6
                    
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
            self._rt_socket = socket(AF_INET, SOCK_STREAM)
            self._rt_socket.settimeout(5.0)
            self._rt_socket.connect((ROBOT_IP, REALTIME_PORT))
            self._connected = True
            self._running = True
            
            # Start background thread to continuously read robot state
            self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._reader_thread.start()
            
            # Wait a moment for first data to arrive
            time.sleep(0.1)
            print(f"Connected to UR robot real-time interface at {ROBOT_IP}:{REALTIME_PORT}")
            return True
            
        except Exception as e:
            print(f"Failed to connect to robot real-time interface: {e}")
            self._connected = False
            return False
    
    def connect_command(self) -> bool:
        """Connect to the UR robot primary interface for sending commands."""
        if self._cmd_connected:
            return True
        
        try:
            self._cmd_socket = socket(AF_INET, SOCK_STREAM)
            self._cmd_socket.settimeout(5.0)
            self._cmd_socket.connect((ROBOT_IP, PRIMARY_PORT))
            self._cmd_connected = True
            print(f"Connected to UR robot primary interface at {ROBOT_IP}:{PRIMARY_PORT}")
            return True
            
        except Exception as e:
            print(f"Failed to connect to robot primary interface: {e}")
            self._cmd_connected = False
            return False
    
    def disconnect(self):
        """Disconnect from the robot."""
        self._running = False
        if self._rt_socket:
            try:
                self._rt_socket.close()
            except:
                pass
        if self._cmd_socket:
            try:
                self._cmd_socket.close()
            except:
                pass
        self._rt_socket = None
        self._cmd_socket = None
        self._connected = False
        self._cmd_connected = False
    
    def send_urscript(self, script: str) -> bool:
        """Send a URScript command to the robot."""
        if not self._cmd_connected:
            if not self.connect_command():
                return False
        
        try:
            # URScript commands must end with newline
            if not script.endswith('\n'):
                script += '\n'
            self._cmd_socket.sendall(script.encode('utf-8'))
            return True
        except Exception as e:
            print(f"Failed to send URScript command: {e}")
            self._cmd_connected = False
            return False
    
    def _read_loop(self):
        """Background thread that continuously reads and parses robot state."""
        while self._running and self._rt_socket:
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
                chunk = self._rt_socket.recv(num_bytes - len(data))
                if not chunk:
                    return None
                data += chunk
            except:
                return None
        return data
    
    def _parse_packet(self, data: bytes):
        """Parse the real-time data packet and extract all robot state data."""
        try:
            # UR uses big-endian format for all values
            
            # Joint positions (radians)
            joint_pos = struct.unpack('>6d', data[JOINT_POSITIONS_OFFSET:JOINT_POSITIONS_OFFSET + 48])
            
            # Joint velocities (rad/s)
            joint_vel = struct.unpack('>6d', data[JOINT_VELOCITIES_OFFSET:JOINT_VELOCITIES_OFFSET + 48])
            
            # Joint currents (A)
            joint_curr = struct.unpack('>6d', data[JOINT_CURRENTS_OFFSET:JOINT_CURRENTS_OFFSET + 48])
            
            # TCP pose [x, y, z, rx, ry, rz] (meters and radians)
            tcp_pose = struct.unpack('>6d', data[TCP_POSE_OFFSET:TCP_POSE_OFFSET + 48])
            
            # TCP speed (m/s and rad/s)
            tcp_speed = struct.unpack('>6d', data[TCP_SPEED_OFFSET:TCP_SPEED_OFFSET + 48])
            
            # TCP force/torque (N and Nm)
            tcp_force = struct.unpack('>6d', data[TCP_FORCE_OFFSET:TCP_FORCE_OFFSET + 48])
            
            # Joint temperatures (Celsius)
            joint_temp = struct.unpack('>6d', data[JOINT_TEMPERATURES_OFFSET:JOINT_TEMPERATURES_OFFSET + 48])
            
            # Tool voltage (V)
            tool_voltage = struct.unpack('>d', data[TOOL_VOLTAGE_OFFSET:TOOL_VOLTAGE_OFFSET + 8])[0]
            
            # Speed scaling (0.0 - 1.0)
            speed_scaling = struct.unpack('>d', data[SPEED_SCALING_OFFSET:SPEED_SCALING_OFFSET + 8])[0]
            
            with self._data_lock:
                self._joint_positions = list(joint_pos)
                self._joint_velocities = list(joint_vel)
                self._joint_currents = list(joint_curr)
                self._tcp_pose = list(tcp_pose)
                self._tcp_speed = list(tcp_speed)
                self._tcp_force = list(tcp_force)
                self._joint_temperatures = list(joint_temp)
                self._tool_voltage = tool_voltage
                self._speed_scaling = speed_scaling
                
        except struct.error as e:
            print(f"Error parsing packet: {e}")
    
    # ============== Getters for robot state ==============
    
    def get_tcp_pose(self) -> List[float]:
        """Get the current TCP pose [x, y, z, rx, ry, rz] in meters and radians."""
        if not self._connected:
            self.connect()
        with self._data_lock:
            return list(self._tcp_pose)
    
    def get_tcp_speed(self) -> List[float]:
        """Get the current TCP speed [dx, dy, dz, drx, dry, drz] in m/s and rad/s."""
        if not self._connected:
            self.connect()
        with self._data_lock:
            return list(self._tcp_speed)
    
    def get_tcp_force(self) -> List[int]:
        """Get the current TCP force/torque [Fx, Fy, Fz, Tx, Ty, Tz] in N and Nm."""
        if not self._connected:
            self.connect()
        with self._data_lock:
            return list(self._tcp_force)
    
    def get_joint_positions(self) -> List[int]:
        """Get the current joint positions [j1, j2, j3, j4, j5, j6] in radians."""
        if not self._connected:
            self.connect()
        with self._data_lock:
            return list(self._joint_positions)
    
    def get_joint_velocities(self) -> List[int]:
        """Get the current joint velocities [j1, j2, j3, j4, j5, j6] in rad/s."""
        if not self._connected:
            self.connect()
        with self._data_lock:
            return list(self._joint_velocities)
    
    def get_tool_voltage(self) -> int:
        """Get the current tool voltage in V."""
        if not self._connected:
            self.connect()
        with self._data_lock:
            return self._tool_voltage
    
    # ============== Motion parameters ==============
    
    def set_linear_acceleration(self, accel: Union[float, List[float]]):
        """
        Set the linear acceleration for cartesian moves (m/s^2).
        This only updates in-memory and does not send to the robot, since we don't have a way of setting / getting per joint accelerations.
        """
        if isinstance(accel, list):
            if not accel:
                raise ValueError("accel list must not be empty")
            accel = max(abs(value) for value in accel)
        with self._data_lock:
            self._linear_acceleration = float(accel)
    
    def get_linear_acceleration(self) -> float:
        """Get the linear acceleration for cartesian moves (m/s^2)."""
        with self._data_lock:
            return self._linear_acceleration
    
    def set_linear_velocity(self, vel: float):
        """
        Set the linear velocity for cartesian moves (m/s).
        This only updates in-memory and does not send to the robot, since we don't have a way of setting / getting per joint velocities.
        """
        with self._data_lock:
            self._linear_velocity = vel
    
    def set_joint_accelerations(self, joint_accelerations: Union[float, List[float]]):
        """Set the joint accelerations for joint moves (rad/s^2).
        This only updates in-memory and does not send to the robot, since we don't have a way of setting / getting per joint accelerations.
        """
        if isinstance(joint_accelerations, list):
            if not joint_accelerations:
                raise ValueError("joint_accelerations must not be empty")
            accelerations = [float(value) for value in joint_accelerations]
            scalar_accel = max(abs(value) for value in accelerations)
        else:
            scalar_accel = float(joint_accelerations)
            accelerations = [scalar_accel] * 6
        with self._data_lock:
            self._joint_accelerations = accelerations
            self._joint_acceleration = scalar_accel
    
    def get_joint_accelerations(self) -> List[float]:
        """Get the joint accelerations for joint moves (rad/s^2).
        This only returns the in-memory value, since we don't have a way of getting per joint accelerations.
        """
        with self._data_lock:
            return list(self._joint_accelerations)
    
    def set_joint_velocities(self, joint_velocities: Union[float, List[float]]):
        """
        Set the joint velocities for joint moves (rad/s).
        This only updates in-memory and does not send to the robot, since we don't have a way of setting / getting per joint velocities.
        """
        if isinstance(joint_velocities, list):
            if not joint_velocities:
                raise ValueError("joint_velocities must not be empty")
            velocities = [float(value) for value in joint_velocities]
            scalar_velocity = max(abs(value) for value in velocities)
        else:
            scalar_velocity = float(joint_velocities)
            velocities = [scalar_velocity] * 6
        with self._data_lock:
            self._joint_velocity_cmd = velocities
            self._joint_velocity = scalar_velocity
    
    # ============== Movement commands ==============
    
    def move_linear(self, x: float, y: float, z: float, rx: float, ry: float, rz: float) -> bool:
        """Move to a cartesian position using linear interpolation."""
        with self._data_lock:
            a = self._linear_acceleration
            v = self._linear_velocity
        script = f"movel(p[{x}, {y}, {z}, {rx}, {ry}, {rz}], a={a}, v={v})"
        return self.send_urscript(script)
    
    def move_joints(self, j1: float, j2: float, j3: float, j4: float, j5: float, j6: float) -> bool:
        """Move to joint positions using joint interpolation."""
        with self._data_lock:
            a = self._joint_acceleration
            v = self._joint_velocity
        script = f"movej([{j1}, {j2}, {j3}, {j4}, {j5}, {j6}], a={a}, v={v})"
        return self.send_urscript(script)
    
    def set_tcp_velocity(self, dx: float, dy: float, dz: float, drx: float, dry: float, drz: float, 
                         accel: float = 0.5, time_seconds: float = 0.0) -> bool:
        """Set TCP velocity (velocity mode). If time_seconds > 0, move for that duration."""
        if time_seconds > 0:
            script = f"speedl([{dx}, {dy}, {dz}, {drx}, {dry}, {drz}], {accel}, {time_seconds})"
        else:
            script = f"speedl([{dx}, {dy}, {dz}, {drx}, {dry}, {drz}], {accel})"
        return self.send_urscript(script)
    
    def set_joint_velocity_cmd(self, velocities: List[float], accel: float = 0.5, time_seconds: float = 0.0) -> bool:
        """Set joint velocities (velocity mode)."""
        v_str = ', '.join(str(v) for v in velocities)
        if time_seconds > 0:
            script = f"speedj([{v_str}], {accel}, {time_seconds})"
        else:
            script = f"speedj([{v_str}], {accel})"
        return self.send_urscript(script)
    
    def stop_robot(self, deceleration: float = 2.0) -> bool:
        """Stop the robot with specified deceleration."""
        script = f"stopj({deceleration})"
        return self.send_urscript(script)
    
    def set_tool_voltage_cmd(self, voltage: int) -> bool:
        """Set tool voltage (0, 12, or 24 V)."""
        if voltage not in [0, 12, 24]:
            print(f"Invalid tool voltage {voltage}. Must be 0, 12, or 24.")
            return False
        script = f"set_tool_voltage({voltage})"
        return self.send_urscript(script)
    
    def set_tool_digital_out(self, pin: int, value: bool) -> bool:
        """Set a tool digital output."""
        script = f"set_tool_digital_out({pin}, {'True' if value else 'False'})"
        return self.send_urscript(script)


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
    _robot_state.move_linear(x, y, z, rx, ry, rz)

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
    if len(cartesian_velocity) != 6:
        raise ValueError("cartesian_velocity must have exactly 6 elements")
    dx, dy, dz, drx, dry, drz = cartesian_velocity
    _robot_state.set_tcp_velocity(dx, dy, dz, drx, dry, drz)

# Acceleration

def get_base_cartesian_acceleration() -> float:
    '''Returns the scalar linear acceleration used for cartesian moves (m/s^2).'''
    return _robot_state.get_linear_acceleration()


def set_base_cartesian_acceleration(cartesian_acceleration: Union[float, List[float]]) -> None:
    '''Sets the scalar linear acceleration used for cartesian moves (m/s^2).'''
    _robot_state.set_linear_acceleration(cartesian_acceleration)

# Force

def get_cartesian_base_force() -> List[int]:
    '''Returns a list of 6 integers, each describing the force felt by the base of the robot in Newtons. Force is described by: (x, y, z, rx, ry, rz)'''
    return _robot_state.get_tcp_force()


def set_cartesian_base_force(cartesian_force: List[int]) -> None:
    '''Sets the amount of force the robot should exert on the object in the base of the robot. Force is input as: (x, y, z, rz, ry, rz) all in Newtons'''
    print("set_cartesian_base_force not implemented")


### Joint Functions

# Position

def get_angular_pose() -> List[int]:
    '''
    Returns a list of 6 integers, each describing the angular (joint 1, joint 2, joint 3, joint 4, joint 5, joint 6) position of the robot.
    '''
    return _robot_state.get_joint_positions()


def move_to_angular_position(j1: int, j2: int, j3: int, j4: int, j5: int, j6: int) -> None:
    '''Moves the robot to the specified position (j1, j2, j3, j4, j5, j6)'''
    _robot_state.move_joints(j1, j2, j3, j4, j5, j6)


# Velocity

def get_angular_velocity() -> List[int]:
    '''
    Returns a list of 6 integers, each describing the angular (joint 1, joint 2, joint 3, joint 4, joint 5, joint 6) velocity of the robot.
    '''
    return _robot_state.get_joint_velocities()


def set_angular_velocity(joint_velocity: Union[float, List[float]]) -> None:
    '''
    Sets the scalar joint velocity used for joint moves (rad/s).
    '''
    _robot_state.set_joint_velocities(joint_velocity)


# Accelerations

def get_angular_acceleration() -> List[float]:
    '''
    Returns the per-joint accelerations used for joint moves (rad/s^2).
    '''
    return _robot_state.get_joint_accelerations()


def set_angular_acceleration(joint_acceleration: Union[float, List[float]]) -> None:    
    '''
    Sets the per-joint accelerations used for joint moves (rad/s^2).
    '''
    _robot_state.set_joint_accelerations(joint_acceleration)


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
    return _robot_state.get_tool_voltage()


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
