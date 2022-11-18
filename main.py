from setup import *
dxl_addparam_result = groupBulkRead.addParam(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

if dxl_addparam_result != True:
    print("[ID:%03d] groupBulkRead addparam failed" % DXL1_ID)
    quit()

while 1:
    if getch() == chr(0x1b): # ESC to break loop
        break
    # CV -> get dx dy in pixels
    #






# Clear bulkread parameter storage
groupBulkRead.clearParam()

# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()