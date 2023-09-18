from __future__ import print_function
import os
import time
from bitstring import BitArray
import math  

########################################################################################
# Save data for initializing saturation range
def SaveData(data):
    text = open("Saturation.txt", "w")
    for i in data:
        text.write(str(i)+"\n")
    
    text.close()
########################################################################################
# Read data from textfile as list
def ReadData(FileName):
    text = open(FileName,'r')
    datas = text.readlines()

    for i in range(len(datas)):
        datas[i] = int(datas[i])
    
    text.close()
    return datas
########################################################################################

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
    def kbhit():
        return msvcrt.kbhit()
else:
    import termios, fcntl, sys, os
    from select import select
    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    new_term = termios.tcgetattr(fd)

    def getch():
        new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(fd, termios.TCSANOW, new_term)
        try:
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        return ch

    def kbhit():
        new_term[3] = (new_term[3] & ~(termios.ICANON | termios.ECHO))
        termios.tcsetattr(fd, termios.TCSANOW, new_term)
        try:
            dr,dw,de = select([sys.stdin], [], [], 0)
            if dr != []:
                return 1
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
            sys.stdout.flush()

        return 0

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430 (Note that XL320 does not support Extended Position Control Mode)

# Control table address
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_OPERATING_MODE         = 11
ADDR_VELOCITY_PROFILE       = 112

# Protocol version
PROTOCOL_VERSION            = 2.0            # See which protocol version is used in the Dynamixel

# Factory default ID of all DYNAMIXEL is 1
YAW_ID                      = 1             # XL330 is controlled for Yaw Moving
PITCH_ID                    = 2             # XL430 is controlled for Pitch Moving

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

BAUDRATE                    = 57600
VELOCITY_CONTROL_MODE       = 1                 # Value for Velocity control mode
POSITION_CONTROL_MODE       = 3                 # Value for Position control mode
EXT_POSITION_CONTROL_MODE   = 4                 # The value of Extended Position Control Mode that can be set through the Operating Mode (11)
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
MAX_POSITION_VALUE          = 8100        # Of MX with firmware 2.0 and X-Series the revolution on Extended Position Control Mode is 256 rev
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel will rotate between this value
CALIBRATION_THRESHOLD       = 300

ESC_ASCII_VALUE             = 0x1b
SPACE_ASCII_VALUE           = 0x20
BACKSPACE_ASCII_VALUE       = 0x08
W_ASCII_VALUE               = 0x77
A_ASCII_VALUE               = 0x61
S_ASCII_VALUE               = 0x73
D_ASCII_VALUE               = 0x64
H_ASCII_VALUE               = 0x68
L_ASCII_VALUE               = 0x6C
Q_ASCII_VALUE               = 0x71
X_ASCII_VALUE               = 0x78
R_ASCII_VALUE               = 0x72
P_ASCII_VALUE               = 0x70

Velocity_Value              = 200

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)
########################################################################################
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()
########################################################################################
# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
########################################################################################
# Set operating mode to extended position control mode
# Pitch motor
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, PITCH_ID, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Operating mode of XL430 changed to velocity control mode.")
# Yaw motor
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, YAW_ID, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Operating mode of XL330 changed to velocity control mode.")

########################################################################################
# Enable Dynamixel Torque
# Yaw motor
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, YAW_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("XL330 has been successfully connected")
# Pitch motor
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, PITCH_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("XL430 has been successfully connected")
########################################################################################
# default Velocity profile
# Yaw motor
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, YAW_ID, ADDR_VELOCITY_PROFILE, Velocity_Value)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("XL330 has been successfully connected")
# Pitch motor
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, PITCH_ID, ADDR_VELOCITY_PROFILE, Velocity_Value)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("XL430 has been successfully connected")

########################################################################################
# function for command motor's movin
def M_Control(motor_id, addr, value):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motor_id, addr, value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

########################################################################################
# function for reading motor's information
def M_Read(motor_id, addr):
    Value, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, motor_id, addr)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    if Value >= 2**31:
        Value -= 2**32
    
    return Value
    # print(" Value = %03d present = %03d (degree)" % (Value/4095*360, dxl_present_position/4095*360))
    
########################################################################################
# function for clearing multi-turn
def Clear(motor_id):
    dxl_comm_result, dxl_error = packetHandler.clearMultiTurn(portHandler, motor_id)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        
######################################################################################## 
# Main Fluke loop
packetHandler.clearMultiTurn(portHandler,YAW_ID)
packetHandler.clearMultiTurn(portHandler,PITCH_ID)

dxl_ID = 1
  
  ########################################################################################
# Read data
# !! You previously should have "Saturatoin.txt" !!
data = []

data = ReadData("Saturation.txt")
if len(data) < 10:
    data = [M_Read(YAW_ID,ADDR_PRESENT_POSITION), M_Read(PITCH_ID,ADDR_PRESENT_POSITION), 8100, -8100, 8100, 0, 8100, -8100, 8100,0]
# <Command Data Order>
# Yaw Present Position
# Pitch Present Position
# Past Yaw Maximum Value
# Past Yaw Minimum Value
# Past Pitch Maximum Value
# Past Pitch Minimum Value
# Present Yaw Maximum Value
# Present Yaw Minimum Value
# Present Pitch Maximum Value
# Present Pitch Minimum Value

########################################################################################
# Calibration


data[2:6] = data[6:]        # Update Past Data From Last Present MinMax Saturation
Rotation = 4095


#####Yaw Calibration####
Yaw_difference = data[0] - M_Read(YAW_ID,ADDR_PRESENT_POSITION)

if Yaw_difference - Rotation < CALIBRATION_THRESHOLD and Yaw_difference - Rotation > -CALIBRATION_THRESHOLD:
    # 360~720 range
    Yaw_Cal_var = -Rotation
elif Yaw_difference - 2*Rotation < CALIBRATION_THRESHOLD and Yaw_difference - 2*Rotation > -CALIBRATION_THRESHOLD:
    # 720~1080 range
    Yaw_Cal_var = -2*Rotation
elif Yaw_difference - 3*Rotation < CALIBRATION_THRESHOLD and Yaw_difference - 3*Rotation > -CALIBRATION_THRESHOLD:
    # 1080~1440 range
    Yaw_Cal_var = -3*Rotation
elif Yaw_difference + Rotation < CALIBRATION_THRESHOLD and Yaw_difference + Rotation > -CALIBRATION_THRESHOLD:
    # -0~-360 range
    Yaw_Cal_var = Rotation
elif Yaw_difference + 2*Rotation < CALIBRATION_THRESHOLD and Yaw_difference + 2*Rotation > -CALIBRATION_THRESHOLD:
    # -360~-720 range
    Yaw_Cal_var = 2*Rotation
elif Yaw_difference + 3*Rotation < CALIBRATION_THRESHOLD and Yaw_difference + 3*Rotation > -CALIBRATION_THRESHOLD:
    # -720~-1080 range
    Yaw_Cal_var = 3*Rotation
elif Yaw_difference < CALIBRATION_THRESHOLD and Yaw_difference > -CALIBRATION_THRESHOLD:
    # 0~360 range
    Yaw_Cal_var = 0
else :
    raise Exception("yaw calibration error")

data[6] = data[6] + Yaw_Cal_var
data[7] = data[7] + Yaw_Cal_var


# #####Pitch Calibration####
# Pitch_difference = data[1] - M_Read(PITCH_ID,ADDR_PRESENT_POSITION)

# if Pitch_difference - Rotation < CALIBRATION_THRESHOLD and Pitch_difference - Rotation > -CALIBRATION_THRESHOLD:
#     # 360~720 range
#     Pitch_Cal_var = - Rotation
# elif Pitch_difference + Rotation < CALIBRATION_THRESHOLD and Pitch_difference + Rotation > -CALIBRATION_THRESHOLD:
#     # -0~-360 range
#     Pitch_Cal_var = Rotation
# elif Pitch_difference < CALIBRATION_THRESHOLD and Pitch_difference > -CALIBRATION_THRESHOLD:
#     # 0~360 ranges
#     Pitch_Cal_var = 0
# else:
#     raise Exception("pitch calibration error")

# data[8] = data[8] + Pitch_Cal_var
# data[9] = data[9] + Pitch_Cal_var

########################################################################################



yaw_Sat = data[6:8]
pitch_Sat = data[8:]
# Sat = [Max Min]

# initialize motor pos safe
YawInitPos = M_Read(YAW_ID, ADDR_PRESENT_POSITION)
PitchInitPos = M_Read(PITCH_ID, ADDR_PRESENT_POSITION)
M_Control(YAW_ID, ADDR_GOAL_POSITION, YawInitPos)
M_Control(PITCH_ID, ADDR_GOAL_POSITION, PitchInitPos)

Yaw_Origin = int(sum(yaw_Sat) /2)


Predefined_Waypoint = open("Waypoints.txt", 'r')
Waypoints = Predefined_Waypoint.readlines()
if len(Waypoints) == 0:
    Save_Data = [["0", 0, 0, pitch_Sat[0]]]
    # Save_Data = [name, yaw_wp_val1, yaw_wp_val2, pitch_wp_val]
else:    
    for i in range(len(Waypoints)):
        Waypoints[i] = Waypoints[i][1:-2]
        Waypoints[i] = Waypoints[i].split(',')
        Waypoints[i][0] = Waypoints[i][0][1:-1]
        for j in range(3):
            Waypoints[i][j+1] = int(Waypoints[i][j+1])
        # Waypoints = [name, yaw1, yaw2, pitch]
    Save_Data = Waypoints
    
print(Save_Data)
# print(type(Waypoints[0][0]]))
while 1:
    # print("\nPress any key to continue! (or press ESC to quit!)")
    c = getch()
    if c == chr(ESC_ASCII_VALUE):
        # M_Control(PITCH_ID, ADDR_GOAL_POSITION, pitch_Sat[1])
        Clear(YAW_ID)
        Clear(PITCH_ID)
        break
    
    elif c == chr(W_ASCII_VALUE):
        # print("W key detected")
        M_Control(PITCH_ID, ADDR_GOAL_POSITION, pitch_Sat[0])
        dxl_ID = 2      # pitch id
        
    elif c == chr(A_ASCII_VALUE):
        # print("A key detected")
        M_Control(YAW_ID, ADDR_GOAL_POSITION, yaw_Sat[1])
        dxl_ID = 1      # yaw id
        
    elif c == chr(S_ASCII_VALUE):
        # print("S key detected")
        M_Control(PITCH_ID, ADDR_GOAL_POSITION, pitch_Sat[1])
        dxl_ID = 2      # pitch id
            
    elif c == chr(D_ASCII_VALUE):
        # print("D key detected")
        M_Control(YAW_ID, ADDR_GOAL_POSITION, yaw_Sat[0])
        dxl_ID = 1      # yaw id
        
    elif c == chr(H_ASCII_VALUE):
        # print("H key detected")
        Velocity_Value += 20
        print("speed up -> %d" % Velocity_Value)
        
        M_Control(YAW_ID, ADDR_VELOCITY_PROFILE, Velocity_Value)        # Yaw motor
        M_Control(PITCH_ID, ADDR_VELOCITY_PROFILE, Velocity_Value)         # Pitch motor
        
    elif c == chr(L_ASCII_VALUE):
        # print("L key detected")
        Velocity_Value -= 20
        print("speed down -> %d" % Velocity_Value)
        M_Control(YAW_ID, ADDR_VELOCITY_PROFILE, Velocity_Value)        # Yaw motor
        M_Control(PITCH_ID, ADDR_VELOCITY_PROFILE, Velocity_Value)         # Pitch motor
        
    elif c == chr(X_ASCII_VALUE):
        key = input("Delete way point : ")
        ind = 0
        while 1:
            if Save_Data[ind][0] == str(key):
                del Save_Data[ind]
                print(Save_Data)
                break
            elif ind+1 == len(Save_Data):
                print("No Waypoint of %s" % (key))
                break
            ind = ind + 1
        Waypoints = open("Waypoints.txt", "w")
        for i in Save_Data:
            Waypoints.write(str(i) + '\n')
        Waypoints.close()
                
        
    elif c == chr(Q_ASCII_VALUE):
        name_val = input("Name the save point as : ")
        # save waypoint's name
        Yaw_wp_val = [M_Read(YAW_ID, ADDR_PRESENT_POSITION)- Yaw_Origin]
        # from absolute origin position get relative waypoint position
        Pitch_wp_val = M_Read(PITCH_ID, ADDR_PRESENT_POSITION)
        # pitch waypoint
        
        
        if Yaw_wp_val[0] > 90:
            Yaw_wp_val.insert(0, Yaw_wp_val[0] - 2*Rotation) 
        elif Yaw_wp_val[0] < -90:
            Yaw_wp_val.append(Yaw_wp_val[0] + 2*Rotation) 
        else:
            raise Exception("way point error")
        
        Save_Data.append([name_val, Yaw_wp_val[0], Yaw_wp_val[1], Pitch_wp_val])
        Save_Data.sort(key=lambda x:-x[1])
        
        # print(name_val, "\n", Yaw_wp_val, "\n", Pitch_wp_val)
        print(Save_Data)
        Waypoints = open("Waypoints.txt", "w")
        for i in Save_Data:
            Waypoints.write(str(i) + '\n')
        Waypoints.close()
        

        # PitchWaypoint.close()
        # YawWaypoint.close()
        # NameWaypoint.close()
        # # q button for waypoint save
        
    elif c == chr(R_ASCII_VALUE):
        input_list = []
        direction_list = []
        
        for i in Save_Data:
            print(i[0],end= ' ')
        print("<- choose what you want\n")
        while 1:
            input_name = input("")
            
            if input_name == "e":           # end input command "e"
                print("input end\n")
                break
            
            for i in Save_Data:             # check input in Save_Data
                if input_name == i[0]:
                    input_list.append(i)    # append input
            # print(input_list,"\n")
            
            
        input_list.sort(key=lambda x:-x[1]) # Sorting input_list
        # name input
        
        for i in range(len(input_list)):
            print("%d. " %(i+1), end = '')
            direction_list.append([])
            for j in range(i, len(input_list)):
                direction_list[i].append(j)
                print("%s" %(input_list[j][0]),end = ' ')
            if i != 0:
                for k in range(0,i):
                    direction_list[i].append(k)
                    print("%s" %(input_list[k][0]),end = ' ')
            print("\n")
        input_direction = direction_list[int(input("choose direction : "))-1]
        
        print(input_direction)
        # print(direction_list)
        Yaw_waypoint_list = []
        Pitch_waypoint_list = []
        for i in input_direction:
            Yaw_waypoint_list.append(input_list[i][1])
            Pitch_waypoint_list.append(input_list[i][2])
                    
        print(Yaw_waypoint_list)
        print(Pitch_waypoint_list)
        if kbhit():
                    c = getch()
                    if c == chr(P_ASCII_VALUE):
                        print("\n  Stop ")
                        # Write the present position to the goal position to stop moving
                        
                        M_Control(dxl_ID, ADDR_GOAL_POSITION, dxl_present_position)      
                        break
        while 1 :
           
            for i in range(len(input_direction)):
                M_Control(YAW_ID, ADDR_GOAL_POSITION, Yaw_waypoint_list[i])
                M_Control(PITCH_ID, ADDR_GOAL_POSITION, Pitch_waypoint_list[i])
                dxl_present_position = Yaw_waypoint_list[i]
                time.sleep(3)
                if kbhit():
                    c = getch()
                    if c == chr(P_ASCII_VALUE):
                        break
            if c == chr(P_ASCII_VALUE):
                break
            for j in reversed(range(len(input_direction)-1)):    
               
                M_Control(YAW_ID, ADDR_GOAL_POSITION, Yaw_waypoint_list[j])
                M_Control(PITCH_ID, ADDR_GOAL_POSITION, Pitch_waypoint_list[j])
                dxl_present_position = Yaw_waypoint_list[j]
                time.sleep(3)
                if kbhit():
                    c = getch()
                    if c == chr(P_ASCII_VALUE):
                        break
            if c == chr(P_ASCII_VALUE):
                break
            
        continue        
           
    # print("  Press SPACE key to Stop")

    # Write goal position
    while 1:
        
        # Read present position
        dxl_present_position = M_Read(dxl_ID,ADDR_PRESENT_POSITION)
        Goal = M_Read(dxl_ID ,ADDR_GOAL_POSITION)
        
        if kbhit():
            c = getch()
            if c == chr(SPACE_ASCII_VALUE):
                print("\n  Stop ")
                # Write the present position to the goal position to stop moving
                
                M_Control(dxl_ID, ADDR_GOAL_POSITION, dxl_present_position)
                    
                time.sleep(0.2)
                    
                # Read present position
                dxl_present_position = M_Read(dxl_ID, ADDR_PRESENT_POSITION)
                Goal = M_Read(dxl_ID, ADDR_GOAL_POSITION)
                
                print("(ID = %d) Goal = %04d      present = %04d     (degree)\n" % (dxl_ID, Goal/4095*360, dxl_present_position/4095*360))
                
                data[dxl_ID-1] = Goal
                SaveData(data)
                break
            
            
        print("(ID = %d) Goal = %04d (%d)   present = %04d (%d)      "% (dxl_ID, Goal,Goal/4095*360,  dxl_present_position, dxl_present_position/4095*360),end = "\r")

        
        if not abs(Goal- dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            data[dxl_ID-1] = Goal
            SaveData(data)
            print("\n")
            break

########################################################################################
# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, YAW_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, PITCH_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
