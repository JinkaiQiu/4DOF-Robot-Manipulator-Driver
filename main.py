from setup import *
while 1:
    if getch() == chr(0x1b): # ESC to break loop
        break
    # read current ball pos in CV
    # ball_pos = CV()
    ball_dpos = ball_pos - ball_last_pos
    rob_dpos = ball_dpos * ball2robCali
    rob_next_pos = rob_pos + rob_dpos
    target_jAng = ik(rob_next_pos)
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
    ball_last_pos = ball_pos

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