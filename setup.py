import os
from robot_FK import *
from robot_IK import *

# Dynamixel Initialization
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * # Uses Dynamixel SDK library

# Dynamixel Setups
ADDR_TORQUE_ENABLE = 64
ADDR_LED_RED = 65
LEN_LED_RED = 1  # Data Byte Length
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4  # Data Byte Length
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4  # Data Byte Length
DXL_MINIMUM_POSITION_VALUE = 0  # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE = 4095  # Refer to the Maximum Position Limit of product eManual
BAUDRATE = 57600

PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
DXL3_ID                     = 3                 # Dynamixel#1 ID : 3
DXL4_ID                     = 4                 # Dynamixel#1 ID : 4
DXL_ID = [DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID]

DEVICENAME                  = 'COM3'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 5                # Dynamixel moving status threshold

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupBulkWrite instance
groupBulkWrite = GroupBulkWrite(portHandler, packetHandler)

# Initialize GroupBulkRead instace for Present Position
groupBulkRead = GroupBulkRead(portHandler, packetHandler)

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

for ID in DXL_ID:
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % ID)
    dxl_addparam_result = groupBulkRead.addParam(ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupBulkRead addparam failed" % ID)
        quit()


# Robot Initialization
cur_jAng = [90, 90, 90, 90]
target_jAng = [45, 45, 45, 45]
ball2robCali = 10  # calibration constant from pixel to m
param_jAng = [[0] * 4] * len(DXL_ID) # 2D array for position val for each joint

# Read Initial Ball position
# ball_pos = CV()
ball_pos = [200, 200]  # place holder
ball_last_pos = ball_pos
ball_dpos = [0, 0]

# Read Initial robot position
dxl_comm_result = groupBulkRead.txRxPacket()
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

for ID in DXL_ID:
    # Check if groupbulkread data of Dynamixel#1 is available
    dxl_getdata_result = groupBulkRead.isAvailable(ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    if dxl_getdata_result != True:
        print("[ID:%03d] groupBulkRead getdata failed" % ID)
        quit()
    cur_jAng[ID-1] = groupBulkRead.getData(ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

rob_pos = fk(cur_jAng)
rob_next_pos = rob_pos
rob_dpos = [0, 0]