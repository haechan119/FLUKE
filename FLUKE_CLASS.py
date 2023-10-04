from __future__ import print_function
import os
import time
from bitstring import BitArray
import math  
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

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

class Control_motor:
    
    def __init__(self):
        # Control table address
        self.ADDR_TORQUE_ENABLE          = 64
        self.ADDR_GOAL_POSITION          = 116
        self.ADDR_PRESENT_POSITION       = 132
        self.ADDR_OPERATING_MODE         = 11
        self.ADDR_VELOCITY_PROFILE       = 112

        # Protocol version
        self.PROTOCOL_VERSION            = 2.0            # See which protocol version is used in the Dynamixel

        # Factory default ID of all DYNAMIXEL is 1
        self.YAW_ID                      = 1             # XL330 is controlled for Yaw Moving
        self.PITCH_ID                    = 2             # XL430 is controlled for Pitch Moving

        # Use the actual port assigned to the U2D2.
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
        self.DEVICENAME                  = '/dev/ttyUSB0'

        self.BAUDRATE                    = 57600
        self.VELOCITY_CONTROL_MODE       = 1                 # Value for Velocity control mode
        self.POSITION_CONTROL_MODE       = 3                 # Value for Position control mode
        self.EXT_POSITION_CONTROL_MODE   = 4                 # The value of Extended Position Control Mode that can be set through the Operating Mode (11)
        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.MAX_POSITION_VALUE          = 8190              # Of MX with firmware 2.0 and X-Series the revolution on Extended Position Control Mode is 256 rev
        self.CALIBRATION_POSITION        = 8160
        self.DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel will rotate between this value
        self.calibration_threshold       = self.MAX_POSITION_VALUE - self.CALIBRATION_POSITION

        self.ESC_ASCII_VALUE             = 0x1b
        self.SPACE_ASCII_VALUE           = 0x20
        self.BACKSPACE_ASCII_VALUE       = 0x08
        self.W_ASCII_VALUE               = 0x77
        self.A_ASCII_VALUE               = 0x61
        self.S_ASCII_VALUE               = 0x73
        self.D_ASCII_VALUE               = 0x64
        self.H_ASCII_VALUE               = 0x68
        self.L_ASCII_VALUE               = 0x6C
        self.Q_ASCII_VALUE               = 0x71
        self.X_ASCII_VALUE               = 0x78
        self.R_ASCII_VALUE               = 0x72
        self.P_ASCII_VALUE               = 0x70
        self.M_ASCII_VALUE               = 0x6d

        self.Yaw_Velocity_Value          = 20    # initial Yaw Velocity
        self.Pitch_Velocity_Value        = self.Yaw_Velocity_Value * 4    # initial pitch Velocity
        
        self.dxl_ID                      = 1
        self.Rotation                    = 4095  # 1 revolution Value

        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        
        self.data = []
        self.yaw_Sat = []
        self.pitch_Sat = []
        self.Yaw_Origin = 0
        self.Save_Data = []
        self.dxl_ID = 1

########################################################################################
# Initial Setting
    def Intial_Setting(self):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()
        
        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
        
        # Set operating mode to extended position control mode
        # Pitch motor
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.PITCH_ID, self.ADDR_OPERATING_MODE, self.EXT_POSITION_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Operating mode of XL430 changed to velocity control mode.")
        # Yaw motor
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.YAW_ID, self.ADDR_OPERATING_MODE, self.EXT_POSITION_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Operating mode of XL330 changed to velocity control mode.")
        # Enable Dynamixel Torque
        # Yaw motor
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.YAW_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("XL330 has been successfully connected")
        # Pitch motor
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.PITCH_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("XL430 has been successfully connected")
        
        # default Velocity profile
        # Yaw motor
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.YAW_ID, self.ADDR_VELOCITY_PROFILE, self.Yaw_Velocity_Value)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("XL330 has been successfully connected")
        # Pitch motor
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.PITCH_ID, self.ADDR_VELOCITY_PROFILE, self.Pitch_Velocity_Value)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("XL430 has been successfully connected")
        # prevent error
        self.packetHandler.clearMultiTurn(self.portHandler,self.YAW_ID)
        self.packetHandler.clearMultiTurn(self.portHandler,self.PITCH_ID)
        data = self.ReadData()
        if len(data) < 10:
            data = [self.M_Read(self.YAW_ID, self.ADDR_PRESENT_POSITION), self.M_Read(self.PITCH_ID, self.ADDR_PRESENT_POSITION), self.Rotation, -self.Rotation, self.Rotation, -self.Rotation, self.Rotation, -self.Rotation, self.Rotation, -self.Rotation]

        """
        <Command Data Order>
        Yaw Present Position
        Pitch Present Position
        Past Yaw Maximum Value
        Past Yaw Minimum Value
        Past Pitch Maximum Value
        Past Pitch Minimum Value
        Present Yaw Maximum Value
        Present Yaw Minimum Value
        Present Pitch Maximum Value
        Present Pitch Minimum Value
        """
        data[2:6] = data[6:]        # Update Past Data From Last Present MinMax Saturation


        #####Yaw Calibration####
        Yaw_difference = data[0] - self.M_Read(self.YAW_ID, self.ADDR_PRESENT_POSITION)

        if Yaw_difference - self.Rotation < self.calibration_threshold and Yaw_difference - self.Rotation > -self.calibration_threshold:
            # 360~720 range
            Yaw_Cal_var = -self.Rotation
        elif Yaw_difference + self.Rotation < self.calibration_threshold and Yaw_difference + self.Rotation > -self.calibration_threshold:
            # -0~-360 range
            Yaw_Cal_var = self.Rotation
        elif Yaw_difference < self.calibration_threshold and Yaw_difference > -self.calibration_threshold:
            # 0~360 range
            Yaw_Cal_var = 0
        else :
            print("yaw calibration error & Reset Saturation")
            data = [self.M_Read(self.YAW_ID, self.ADDR_PRESENT_POSITION), self.M_Read(self.PITCH_ID, self.ADDR_PRESENT_POSITION), self.Rotation, -self.Rotation, self.Rotation, -self.Rotation, self.Rotation, -self.Rotation, self.Rotation, -self.Rotation]
            Yaw_Cal_var = 0

        data[6] = data[6] + Yaw_Cal_var
        data[7] = data[7] + Yaw_Cal_var
        self.data = data

        self.yaw_Sat = self.data[6:8]
        self.pitch_Sat = self.data[8:]
        # Sat = [Max Min]

        self.Yaw_Origin = int(sum(self.yaw_Sat) /2)


        Predefined_Waypoint = open("Waypoints.txt", 'r')
        Waypoints = Predefined_Waypoint.readlines()
        if len(Waypoints) == 0:
            self.Save_Data = [["0", 0, self.pitch_Sat[0]]]
            # Save_Data = [name, yaw_wp_val1, yaw_wp_val2, pitch_wp_val]
        else:    
            for i in range(len(Waypoints)):
                Waypoints[i] = Waypoints[i][1:-2]
                Waypoints[i] = Waypoints[i].split(',')
                Waypoints[i][0] = Waypoints[i][0][1:-1]
                for j in range(2):
                    Waypoints[i][j+1] = int(Waypoints[i][j+1])
                # Waypoints = [name, yaw1, yaw2, pitch]
            self.Save_Data = Waypoints
    ########################################################################################
    # Save data for initializing saturation range
    def SaveData(self, data):
        text = open("Saturation.txt", "w")
        for i in data:
            text.write(str(i)+"\n")
        
        text.close()
    ########################################################################################
    # Read data from textfile as list
    def ReadData(self):
        text = open("Saturation.txt","r")
        datas = text.readlines()

        for i in range(len(datas)):
            datas[i] = int(datas[i])
        
        text.close()
        return datas
    ########################################################################################
    # function for command motor's movin
    def M_Control(self, motor_id, addr, value):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, addr, value)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    ########################################################################################
    # function for reading motor's information
    def M_Read(self, motor_id, addr):
        Value, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, motor_id, addr)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        if Value >= 2**31:
            Value -= 2**32
        
        return Value
        # print(" Value = %03d present = %03d (degree)" % (Value/4095*360, dxl_present_position/4095*360))
    ########################################################################################
    # function for clearing multi-turn
    def Clear(self, motor_id):
        dxl_comm_result, dxl_error = self.packetHandler.clearMultiTurn(self.portHandler, motor_id)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
    ########################################################################################
    def SaveGoalPos(self, dxl_ID, data):
        Goal = self.M_Read(dxl_ID , self.ADDR_GOAL_POSITION)
        data[dxl_ID-1] = Goal
        self.SaveData(data)
        print("\n")
    ########################################################################################
    def PitchSpeedSpanning(self, Ystartpoint, Yendpoint, Pstartpoint, Pendpoint):
        if Ystartpoint - Yendpoint != 0:
            Pitch_Velocity_Value = self.Yaw_Velocity_Value * abs(Pstartpoint - Pendpoint) / abs(Ystartpoint - Yendpoint)
            self.M_Control(self.PITCH_ID, self.ADDR_VELOCITY_PROFILE, int(Pitch_Velocity_Value))
            sleepTime = abs(Ystartpoint - Yendpoint) / self.Yaw_Velocity_Value / 15.7
        else:
            sleepTime = 3
        return sleepTime
    ######################################################################################## 
    # Read data & Calibration
        # !! You previously should have "Saturatoin.txt" !!
    def shutdown(self):
        # if c == chr(ESC_ASCII_VALUE):
        self.M_Control(self.PITCH_ID, self.ADDR_VELOCITY_PROFILE, 200)
        self.M_Control(self.PITCH_ID, self.ADDR_GOAL_POSITION, self.pitch_Sat[0])
        time.sleep(10)
        self.Clear(self.YAW_ID)
        self.Clear(self.PITCH_ID)
        self.SaveData(self.data)
        # I have no idea what am I doing
        ########################################################################################
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.YAW_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.PITCH_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Close port
        self.portHandler.closePort()
        # break was used to shutdown while loop
    
    def control_up(self):
        # elif c == chr(W_ASCII_VALUE):
        self.M_Control(self.PITCH_ID, self.ADDR_VELOCITY_PROFILE, self.Pitch_Velocity_Value)
        self.M_Control(self.PITCH_ID, self.ADDR_GOAL_POSITION, self.pitch_Sat[0])
        self.SaveGoalPos(self.PITCH_ID, self.data)
        self.dxl_ID = 2
    
    def control_down(self):
        # elif c == chr(S_ASCII_VALUE):
        self.M_Control(self.PITCH_ID, self.ADDR_VELOCITY_PROFILE, self.Pitch_Velocity_Value)
        self.M_Control(self.PITCH_ID, self.ADDR_GOAL_POSITION, self.pitch_Sat[1])
        self.SaveGoalPos(self.PITCH_ID, self.data)
        self.dxl_ID = 2      # pitch id
    
    def control_left(self):
        # elif c == chr(A_ASCII_VALUE):
        self.M_Control(self.YAW_ID, self.ADDR_GOAL_POSITION, self.yaw_Sat[1])
        self.SaveGoalPos(self.YAW_ID, self.data)
        self.dxl_ID = 1      # yaw id
    
    def control_right(self):
        self.M_Control(self.YAW_ID, self.ADDR_GOAL_POSITION, self.yaw_Sat[0])
        self.SaveGoalPos(self.YAW_ID, self.data)
        self.dxl_ID = 1      # yaw id
    
    def control_speed_up(self):
        if self.Yaw_Velocity_Value < 30:
            self.Yaw_Velocity_Value += 10
            self.Pitch_Velocity_Value = self.Yaw_Velocity_Value * 4
        else:
            print("Already Maximum Velocity value")
        print("speed up -> %d" % self.Yaw_Velocity_Value)
        
        self.M_Control(self.YAW_ID, self.ADDR_VELOCITY_PROFILE, self.Yaw_Velocity_Value)        # Yaw motor
        self.M_Control(self.PITCH_ID, self.ADDR_VELOCITY_PROFILE, self.Pitch_Velocity_Value)         # Pitch motor
    
    def control_speed_down(self):
        if self.Yaw_Velocity_Value > 10:
            self.Yaw_Velocity_Value -= 10
            self.Pitch_Velocity_Value = self.Yaw_Velocity_Value * 4
        else:
            print("Already Minimum Velocity value")
        
        print("speed down -> %d" % self.Yaw_Velocity_Value)
        self.M_Control(self.YAW_ID, self.ADDR_VELOCITY_PROFILE, self.Yaw_Velocity_Value)        # Yaw motor
        self.M_Control(self.PITCH_ID, self.ADDR_VELOCITY_PROFILE, self.Pitch_Velocity_Value)         # Pitch motor
    
    def waypoint_delete(self, command):
        key = command[0]
        ind = 0
        while 1:
            if self.Save_Data[ind][0] == str(key):
                del self.Save_Data[ind]
                print(self.Save_Data)
                break
            elif ind+1 == len(self.Save_Data):
                print("No Waypoint of %s" % (key))
                break
            ind = ind + 1
        Waypoints = open("Waypoints.txt", "w")
        for i in self.Save_Data:
            Waypoints.write(str(i) + '\n')
        Waypoints.close()
    
    def waypoint_save(self, command):
        name_val = str(command[0])
        # save waypoint's name
        Yaw_wp_val = self.M_Read(self.YAW_ID, self.ADDR_PRESENT_POSITION)- self.Yaw_Origin
        # from absolute origin position get relative waypoint position
        Pitch_wp_val = self.M_Read(self.PITCH_ID, self.ADDR_PRESENT_POSITION)
        # pitch waypoint
        
        
        self.Save_Data.append([name_val, Yaw_wp_val, Pitch_wp_val])
        self.Save_Data.sort(key=lambda x:-x[1])
        
        # print(name_val, "\n", Yaw_wp_val, "\n", Pitch_wp_val)
        print(self.Save_Data)
        Waypoints = open("Waypoints.txt", "w")
        for i in self.Save_Data:
            Waypoints.write(str(i) + '\n')
        Waypoints.close()
    
    def control_rotation(self, command): 
        input_list = []
        
        # for i in self.Save_Data:
        #     print(i[0],end= ' ')
        # print("<- choose what you want\n")
        # while 1:
        for k in command:
            # if command == "e":           # end input command "e"
            #     print("input end\n")
            #     break
            for i in self.Save_Data:             # check input in Save_Data
                if k == i[0]:
                    input_list.append(i)    # append input
            # print(input_list,"\n")
            
        input_list.sort(key=lambda x:-x[1]) # Sorting input_list
        # name input
        
        Yaw_waypoint_list = []
        Pitch_waypoint_list = []
        dxl_present_pos = self.M_Read(self.YAW_ID, self.ADDR_PRESENT_POSITION)
        ################################################################################
        if abs(input_list[0][1] + self.Yaw_Origin - dxl_present_pos) > abs(input_list[-1][1] + self.Yaw_Origin  - dxl_present_pos):
            input_list.sort(key=lambda x:x[1])
        ################################################################################
        for i in range(len(input_list)):
            Yaw_waypoint_list.append(input_list[i][1])
            Pitch_waypoint_list.append(input_list[i][2])
                    
        print(Yaw_waypoint_list)
        print(Pitch_waypoint_list)     
        c = ""        
        LoopStart = 1
        while 1:
            
            for i in range(1, len(input_list)):
                
                text = open("Command.txt","r")
                datas = text.readlines()
                if len(datas) == 0:
                    pass
                else:
                    datas = str(datas[0][:-1])
                    print(datas)
                if datas == chr(self.SPACE_ASCII_VALUE):
                    self.control_stop()
                    text.close()
                    text = open("Command.txt","w")
                    text.close()
                    break
                
                print(i)
                
                if LoopStart != 1 :
                    sleepTime = self.PitchSpeedSpanning(Yaw_waypoint_list[i], Yaw_waypoint_list[i-1], Pitch_waypoint_list[i], Pitch_waypoint_list[i-1])
                    self.M_Control(self.YAW_ID, self.ADDR_GOAL_POSITION, Yaw_waypoint_list[i]+ self.Yaw_Origin)
                    self.M_Control(self.PITCH_ID, self.ADDR_GOAL_POSITION, Pitch_waypoint_list[i])
                    print("Yaw " + str(Yaw_waypoint_list[i]) + "\n")
                    print("Pitch" + str(Pitch_waypoint_list[i]) + "\n")
                    time.sleep(sleepTime)
                else:
                    LoopStart = 0
                    sleepTime = self.PitchSpeedSpanning(Yaw_waypoint_list[i] + self.Yaw_Origin, self.M_Read(self.YAW_ID, self.ADDR_PRESENT_POSITION), Pitch_waypoint_list[i], self.M_Read(self.PITCH_ID, self.ADDR_PRESENT_POSITION))
                    self.M_Control(self.YAW_ID, self.ADDR_GOAL_POSITION, Yaw_waypoint_list[i]+ self.Yaw_Origin)
                    self.M_Control(self.PITCH_ID, self.ADDR_GOAL_POSITION, Pitch_waypoint_list[i])
                    print("Yaw " + str(Yaw_waypoint_list[i]) + "\n")
                    print("Pitch" + str(Pitch_waypoint_list[i]) + "\n")
                    time.sleep(sleepTime)
                
                if kbhit():
                    c = getch()
                    if c == chr(self.P_ASCII_VALUE):
                        self.SaveGoalPos(self.PITCH_ID, self.data)
                        self.SaveGoalPos(self.YAW_ID, self.data)
                        break
                    
            if c == chr(self.P_ASCII_VALUE):
                break
            if datas == chr(self.SPACE_ASCII_VALUE):
                break
            
            for j in reversed(range(len(input_list)-1)):
                
                text = open("Command.txt","r")
                datas = text.readlines()
                if len(datas) == 0:
                    pass
                else:
                    datas = str(datas[0][:-1])
                    print(datas)
                if datas == chr(self.SPACE_ASCII_VALUE):
                    self.control_stop()
                    text.close()
                    text = open("Command.txt","w")
                    text.close()
                    break
                
                sleepTime = self.PitchSpeedSpanning(Yaw_waypoint_list[j], Yaw_waypoint_list[j+1], Pitch_waypoint_list[j], Pitch_waypoint_list[j+1])
                self.M_Control(self.YAW_ID, self.ADDR_GOAL_POSITION, Yaw_waypoint_list[j]+ self.Yaw_Origin)
                self.M_Control(self.PITCH_ID, self.ADDR_GOAL_POSITION, Pitch_waypoint_list[j])
                print(j)
                print("Yaw " + str(Yaw_waypoint_list[j]) + "\n")
                print("Pitch" + str(Pitch_waypoint_list[j]) + "\n")
                time.sleep(sleepTime)
                if kbhit():
                    c = getch()
                    if c == chr(self.P_ASCII_VALUE):
                        self.SaveGoalPos(self.PITCH_ID, self.data)
                        self.SaveGoalPos(self.YAW_ID, self.data)
                        break
                    
            if c == chr(self.P_ASCII_VALUE):
                break
            if datas == chr(self.SPACE_ASCII_VALUE):
                break
            
    def control_stop(self):
        print("\n  Stop ")
        
        # pitch Stop
        pitch_dxl_present_position = self.M_Read(self.PITCH_ID, self.ADDR_PRESENT_POSITION)
        self.M_Control(self.PITCH_ID, self.ADDR_GOAL_POSITION, pitch_dxl_present_position)
        yaw_dxl_present_position = self.M_Read(self.YAW_ID, self.ADDR_PRESENT_POSITION)
        self.M_Control(self.YAW_ID, self.ADDR_GOAL_POSITION, yaw_dxl_present_position)
        
        time.sleep(0.2)
            
        # Read present position
        pitch_dxl_present_position = self.M_Read(self.PITCH_ID, self.ADDR_PRESENT_POSITION)
        pitch_Goal = self.M_Read(self.PITCH_ID, self.ADDR_GOAL_POSITION)
        yaw_dxl_present_position = self.M_Read(self.YAW_ID, self.ADDR_PRESENT_POSITION)
        yaw_Goal = self.M_Read(self.YAW_ID, self.ADDR_GOAL_POSITION)
        
        print("(Pitch motor) Goal = %04d present = %04d (degree)" %(pitch_Goal/4095*360, pitch_dxl_present_position/4095*360))
        print("(Yaw motor) Goal = %04d present = %04d (degree)\n" % (yaw_Goal/4095*360, yaw_dxl_present_position/4095*360))
        
        
        
        self.data[0:2] = [yaw_Goal, pitch_Goal]
        self.SaveData(self.data)
    
    def control_move(self, command):
        for i in self.Save_Data:
            print(i[0],end= ' ')
        print("<- choose what you want\n")
        move_name = str(command[0])
        for i in self.Save_Data:             # check input in Save_Data
                if move_name == i[0]:
                   get_waypoint = i
                   print(get_waypoint)
        
        self.M_Control(self.PITCH_ID, self.ADDR_GOAL_POSITION, get_waypoint[2])
        self.M_Control(self.YAW_ID, self.ADDR_GOAL_POSITION, get_waypoint[1] + self.Yaw_Origin)
        
        self.SaveGoalPos(self.YAW_ID, self.data)
        self.SaveGoalPos(self.PITCH_ID, self.data)
