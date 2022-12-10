import os
from robot_FK import *
from robot_IK import *
import time
import math
import numpy as py


def homegoto(P_start, home, N = 101):
    Px = P_start[0]; Py = P_start[1]; Pz = P_start[2]
    Hx = home[0]; Hy = home[1]; Hz = home[2]

    # Goal Trajectory: Cosine Function --- z = Acos(wt)+A
    T = np.sqrt((Px - Hx) ** 2 + (Py - Hy) ** 2); # T is half period of Cosine Function
    t = np.linspace(0,T,N)
    A = (Hz - Pz) / 2
    w = np.pi / T

    x = np.linspace(Hx, Px, N)
    y = np.linspace(Hy, Py, N)
    z = Pz + A * np.cos(w * t) + A
    traj = np.vstack((x, y, z))

    return traj
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

DEVICENAME                  = 'COM5'

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

# a=2048
# b=2048
# c=3072
# flag=1
# x=0
# y=0
# z=30

### Traj
# N=len
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

N = 101
M = 10
#home_position=np.array([0.06,0,0.36])
home_position = fk(cur_jAng)
print(home_position)
starting_point=np.array([0.1789,0.0289,0.1089])
traj=homegoto(starting_point,home_position)

film_point=0.1089

# for i in range(N):
#     film_point=np.array([0.1789+0.01*i,0.0289,0.1089])
# trajfilm=

jAng =[0, 0, 0 ,0]
for i in range(N+M):
    if i < N:
        goal = [traj[0][i], traj[1][i],traj[2][i]]
        jAng = np.vstack((jAng,ik(goal,Mode=1)))
    else:
        goal = [0.1789 + 0.008 * (i-N), 0.0289, 0.1089]
        print(goal)
        # jAng = np.vstack((jAng,ik(goal,Mode=1)))
        


# jAng_film = [0, 0, 0 ,0]
#     for i in range(N):


print(jAng)
print(len(jAng))
# print(traj)
run=1
while 1:

    # if getch() == chr(0x1b): # ESC to break loop
    #     break
    
    # print(flag)
    # print(a)
    # if(flag==1):

    # # #     # goal = [2048, 3072, 2048, 2048] 
    # # #     # goal = goal.split(" ")
    # # #     # for j in range(4):
    # # #     #     goal[j] = int(goal[j])
    # # #     # target_jAng = goal
    #     target_jAng = [2048, 3072, 2048, 2048]  # Home Position
    #     flag = 2
    #     print(flag)
    #     print(target_jAng)

    # else:
    # if(a<2600):
    #     a=a-5
    #     b=b+5
    #     c=c-5
    # else:
    #     break
    # # target_jAng= [2048,c,b,a] # IK(x,y,z)
    # print(target_jAng)
    target_jAng=[0,0,0,0]
    
    target_jAng[0]=int(jAng[run][0])
    target_jAng[1]=int(jAng[run][1])
    target_jAng[2]=int(jAng[run][2])
    target_jAng[3]=int(jAng[run][3])
    print(type(target_jAng))
    print(type(target_jAng[0]))
    if(run == N):
        break
    else:
        run=run+1
    # target_jAng=ik(traj)



    for i in range(len(DXL_ID)):
        # format each joint angle to byte array
        ang = target_jAng[i]
        # ang = target_jAng[i]
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
    
    print('Action Done!')

    # Clear bulkwrite parameter storage
    groupBulkWrite.clearParam()

    # for i in range(100):

    time.sleep(0.05)

    if run == N:
        time.sleep(3)
    # while 1:  # Reading data until moved to position
    #     # Bulkread present position and LED status
    #     dxl_comm_result = groupBulkRead.txRxPacket()
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    #     for ID in DXL_ID:  # Check data available
    #         dxl_getdata_result = groupBulkRead.isAvailable(ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    #         if dxl_getdata_result != True:
    #             print("[ID:%03d] groupBulkRead getdata failed" % ID)
    #             quit()
    #         # Read j angle data
    #         cur_jAng[ID-1] = groupBulkRead.getData(ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    #         print("[ID:%03d] Present Position : %d \t" % (ID, cur_jAng[ID-1]))
    #     state = 0
    #     for i in range(len(DXL_ID)):  # check if all joints already moved to position
    #         if abs(cur_jAng[i] - target_jAng[i]) < DXL_MOVING_STATUS_THRESHOLD:
    #             state = state + 1
    #             break
    #         print(i)
                
    #     if state == len(DXL_ID):
    #         print('Reading Done!')
    #         break
    # # ball_last_pos = ball_pos
    # # end_t = time.time()
    # dt = end_t - start_t
########################################################################################################################

# Clear bulkread parameter storage
groupBulkRead.clearParam()

time.sleep(5)

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

# Disable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#4 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))


# Close port
portHandler.closePort()  


