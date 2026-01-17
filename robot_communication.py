'''
Below are a list of functions that you may use to get the described information from the robot. The internals of this code are kept hidden.
You may fully rely on the descriptions of each function to accurately convey what is being returned by the function itself.
'''


#############################################
##
## Robot Arm Low-Level Functions
##
#############################################


### Cartesian Functions

# Position

def get_base_cartesian_pose() -> List[int]:
    '''Returns a list of 6 integers, each describing the cartesian base (x, y, z, rx, ry, rz) position of the robot.'''
    return [1, 1, 1, 1, 1, 1]


def move_to_base_cartesian_position(x: int, y: int, z: int, rx: int, ry: int, rz: int) -> None:
    '''Moves the robot to the specified position (x, y, z, rx, ry, rz) in base frame'''

# Velocity

def get_base_cartesian_velocity() -> List[int]:
    '''Returns a list of 6 integers, each describing the cartesian base (x, y, z, rx, ry, rz) velocity of the robot.'''
    return [1, 1, 1, 1, 1, 1]


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
