import os
from robot_FK import *
from robot_IK import *
import time

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

param_jAng = [[0] * 4] * len(DXL_ID) # 2D array for position val for each joint
cur_jAng = [90, 90, 90, 90]


while 1:
    if getch() == chr(0x1b): # ESC to break loop
        break

    goal = input("Please enter your goal:")
    goal = goal.split(" ")
    for j in range(4):
        goal[j] = int(goal[j])
    # target_jAng = goal
    target_jAng = goal

    for i in range(len(DXL_ID)):
        # format each joint angle to byte array
        ang = target_jAng[i]
        param_jAng[i] = [DXL_LOBYTE(DXL_LOWORD(ang)), DXL_HIBYTE(DXL_LOWORD(ang)), DXL_LOBYTE(DXL_HIWORD(ang)), DXL_HIBYTE(DXL_HIWORD(ang))]
        # Add Dynamixel#1 goal position value to the Bulkwrite parameter storage
        dxl_addparam_result = groupBulkWrite.addParam(DXL_ID[i], ADDR_GOAL_POSITION, LEN_GOAL_POSITION, param_jAng[i])
        if dxl_addparam_result != True:
            print("[ID:%03d] groupBulkWrite addparam failed" % DXL_ID[i])
            quit()
    # Send data
    dxl_comm_result = groupBulkWrite.txPacket()
    # Check results
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear bulkwrite parameter storage
    groupBulkWrite.clearParam()

    while 1:  # Reading data until moved to position
        # Bulkread present position and LED status
        dxl_comm_result = groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        for ID in DXL_ID:  # Check data available
            dxl_getdata_result = groupBulkRead.isAvailable(ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % ID)
                quit()
            # Read j angle data
            cur_jAng[ID-1] = groupBulkRead.getData(ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            print("[ID:%03d] Present Position : %d \t" % (ID, cur_jAng[ID-1]))
        state = 0
        for i in range(len(DXL_ID)):  # check if all joints already moved to position
            if abs(cur_jAng[i] - target_jAng[i]) < DXL_MOVING_STATUS_THRESHOLD:
                state = state + 1
        if state == len(DXL_ID):
            break
    # ball_last_pos = ball_pos
    # end_t = time.time()
    # dt = end_t - start_t
########################################################################################################################

# Clear bulkread parameter storage
groupBulkRead.clearParam()

# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketErr
        or(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()