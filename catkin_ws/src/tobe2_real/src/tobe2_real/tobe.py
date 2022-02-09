import random
from threading import Thread
import math
import rospy
import time
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from dynio import *

import os

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


class Tobe:

    def __init__(self,ns="/tobe/"):
        self.ns=ns
        # array of TOBE joint names in Robotis order
        self.joints=["r_shoulder_sagittal","l_shoulder_sagittal","r_shoulder_frontal","l_shoulder_frontal","r_elbow","l_elbow",
        "r_hip_swivel","l_hip_swivel","r_hip_frontal","l_hip_frontal","r_hip_sagittal","l_hip_sagittal","r_knee","l_knee",
        "r_ankle_sagittal","l_ankle_sagittal","r_ankle_frontal","l_ankle_frontal"]
        
        # create joint command publishers
        rospy.loginfo("+Creating joint command publishers...")
        self._pub_joints={}
        self._pub_joint_cmds={}
        for j in self.joints:
            p=rospy.Publisher(self.ns+j+"/command",Float64, queue_size=10)
            q=rospy.Publisher(self.ns+j+"/angle",Float64, queue_size=10)
            self._pub_joint_cmds[j]=p
            self._pub_joints[j]=q
            rospy.loginfo(" -Found: "+j)
        

        # Control table address
        ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
        ADDR_MX_GOAL_POSITION      = 30
        ADDR_MX_PRESENT_POSITION   = 36
        
        # Data Byte Length
        LEN_MX_GOAL_POSITION       = 4
        LEN_MX_PRESENT_POSITION    = 4
        
        # Protocol version
        PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel
        
        # Default setting
        DXL1_ID                     = 1                 # Dynamixel
        DXL2_ID                     = 2                 # Dynamixel
        DXL3_ID                     = 3                 # Dynamixel
        DXL4_ID                     = 4                 # Dynamixel
        DXL5_ID                     = 5                 # Dynamixel
        DXL6_ID                     = 6                 # Dynamixel
        DXL7_ID                     = 7                 # Dynamixel
        DXL8_ID                     = 8                 # Dynamixel
        DXL9_ID                     = 9                 # Dynamixel
        DXL10_ID                     = 10                 # Dynamixel
        DXL11_ID                     = 11                 # Dynamixel
        DXL12_ID                     = 12                 # Dynamixel
        DXL13_ID                     = 13                 # Dynamixel
        DXL14_ID                     = 14                 # Dynamixel
        DXL15_ID                     = 15                 # Dynamixel
        DXL16_ID                     = 16                 # Dynamixel
        DXL17_ID                     = 17                 # Dynamixel
        DXL18_ID                     = 18                 # Dynamixel
        BAUDRATE                    = 1000000             # Dynamixel AX12 default baudrate : 1000000
        DEVICENAME                  = '/dev/ttyUSB1'    # Check which port is being used on your controller, Linux: "/dev/ttyUSB0"
        
        TORQUE_ENABLE               = 1                 # Value for enabling the torque
        TORQUE_DISABLE              = 0                 # Value for disabling the torque
        
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        portHandler = PortHandler(DEVICENAME)
        
        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(portHandler, self.packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
        
        # Open port
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

        
        # set up motor connections
        # Enable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL1_ID)
    
        # Enable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL2_ID)
    
        # Enable Dynamixel#3 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL3_ID)
    
        # Enable Dynamixel#4 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL4_ID)
    
        # Enable Dynamixel#5 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL5_ID)
    
        # Enable Dynamixel#6 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL6_ID)
    
        # Enable Dynamixel#7 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL7_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL7_ID)
    
        # Enable Dynamixel#8 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL8_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL8_ID)
    
        # Enable Dynamixel#9 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL9_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL9_ID)
    
        # Enable Dynamixel#10 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL10_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL10_ID)
    
        # Enable Dynamixel#11 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL11_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL11_ID)
    
        # Enable Dynamixel#12 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL12_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL12_ID)
        
        # Enable Dynamixel#13 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL13_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL13_ID)
    
        # Enable Dynamixel#14 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL14_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL14_ID)
    
        # Enable Dynamixel#15 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL15_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL15_ID)
    
        # Enable Dynamixel#16 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL16_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL16_ID)
    
        # Enable Dynamixel#17 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL17_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL17_ID)
    
        # Enable Dynamixel#18 Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(portHandler, DXL18_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL18_ID)       

    def command_sag_motors(self,vals):
        # this function sends commands to the motors for sagittal shoulders (ID:1,2), hips (ID:11,12), and ankles (ID: 15,16)
        # 'cmds' is just a 6-element array of the 10-bit values to the joints in that order
        
        # sagittal joint motor assignments:
        DXL1_ID                     = 1                 # Dynamixel
        DXL2_ID                     = 2                 # Dynamixel
        DXL11_ID                    = 11                 # Dynamixel
        DXL12_ID                    = 12                 # Dynamixel
        DXL15_ID                    = 15                 # Dynamixel
        DXL16_ID                    = 16                 # Dynamixel
        
        # Allocate goal position values into byte arrays:
        cmds=[int(x) for x in vals]
        
        param_goal_position1 = [DXL_LOBYTE(DXL_LOWORD(cmds[0])), DXL_HIBYTE(DXL_LOWORD(cmds[0])), DXL_LOBYTE(DXL_HIWORD(cmds[0])), DXL_HIBYTE(DXL_HIWORD(cmds[0]))]
        param_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(cmds[1])), DXL_HIBYTE(DXL_LOWORD(cmds[1])), DXL_LOBYTE(DXL_HIWORD(cmds[1])), DXL_HIBYTE(DXL_HIWORD(cmds[1]))]
        param_goal_position3 = [DXL_LOBYTE(DXL_LOWORD(cmds[2])), DXL_HIBYTE(DXL_LOWORD(cmds[2])), DXL_LOBYTE(DXL_HIWORD(cmds[2])), DXL_HIBYTE(DXL_HIWORD(cmds[2]))]
        param_goal_position4 = [DXL_LOBYTE(DXL_LOWORD(cmds[3])), DXL_HIBYTE(DXL_LOWORD(cmds[3])), DXL_LOBYTE(DXL_HIWORD(cmds[3])), DXL_HIBYTE(DXL_HIWORD(cmds[3]))]
        param_goal_position5 = [DXL_LOBYTE(DXL_LOWORD(cmds[4])), DXL_HIBYTE(DXL_LOWORD(cmds[4])), DXL_LOBYTE(DXL_HIWORD(cmds[4])), DXL_HIBYTE(DXL_HIWORD(cmds[4]))]
        param_goal_position6 = [DXL_LOBYTE(DXL_LOWORD(cmds[5])), DXL_HIBYTE(DXL_LOWORD(cmds[5])), DXL_LOBYTE(DXL_HIWORD(cmds[5])), DXL_HIBYTE(DXL_HIWORD(cmds[5]))]
        
        # Combine parameters into single groupSyncWrite command:
        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL1_ID, param_goal_position1)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
            quit()

        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL2_ID, param_goal_position2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
            quit()
        # Add Dynamixel#11 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL11_ID, param_goal_position3)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL11_ID)
            quit()

        # Add Dynamixel#12 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL12_ID, param_goal_position4)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL12_ID)
            quit()    
        # Add Dynamixel#15 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL15_ID, param_goal_position5)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL15_ID)
            quit()

        # Add Dynamixel#16 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL16_ID, param_goal_position6)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL16_ID)
            quit() 
        
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()
        
        
    def command_all_motors(self,cmds):
        # this function sends motor commands to all 18 joints
        
        # motor assignments:
        DXL1_ID                     = 1                 # Dynamixel
        DXL2_ID                     = 2                 # Dynamixel
        DXL3_ID                     = 3                 # Dynamixel
        DXL4_ID                     = 4                 # Dynamixel
        DXL5_ID                     = 5                 # Dynamixel
        DXL6_ID                     = 6                 # Dynamixel
        DXL7_ID                     = 7                 # Dynamixel
        DXL8_ID                     = 8                 # Dynamixel
        DXL9_ID                     = 9                 # Dynamixel
        DXL10_ID                     = 10                 # Dynamixel
        DXL11_ID                     = 11                 # Dynamixel
        DXL12_ID                     = 12                 # Dynamixel
        DXL13_ID                     = 13                 # Dynamixel
        DXL14_ID                     = 14                 # Dynamixel
        DXL15_ID                     = 15                 # Dynamixel
        DXL16_ID                     = 16                 # Dynamixel
        DXL17_ID                     = 17                 # Dynamixel
        DXL18_ID                     = 18                 # Dynamixel
        
        # Allocate goal position values into byte arrays:
        vals=[int(x) for x in cmds]
        
        param_goal_position1 = [DXL_LOBYTE(DXL_LOWORD(vals[0])), DXL_HIBYTE(DXL_LOWORD(vals[0])), DXL_LOBYTE(DXL_HIWORD(vals[0])), DXL_HIBYTE(DXL_HIWORD(vals[0]))]
        param_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(vals[1])), DXL_HIBYTE(DXL_LOWORD(vals[1])), DXL_LOBYTE(DXL_HIWORD(vals[1])), DXL_HIBYTE(DXL_HIWORD(vals[1]))]
        param_goal_position3 = [DXL_LOBYTE(DXL_LOWORD(vals[2])), DXL_HIBYTE(DXL_LOWORD(vals[2])), DXL_LOBYTE(DXL_HIWORD(vals[2])), DXL_HIBYTE(DXL_HIWORD(vals[2]))]
        param_goal_position4 = [DXL_LOBYTE(DXL_LOWORD(vals[3])), DXL_HIBYTE(DXL_LOWORD(vals[3])), DXL_LOBYTE(DXL_HIWORD(vals[3])), DXL_HIBYTE(DXL_HIWORD(vals[3]))]
        param_goal_position5 = [DXL_LOBYTE(DXL_LOWORD(vals[4])), DXL_HIBYTE(DXL_LOWORD(vals[4])), DXL_LOBYTE(DXL_HIWORD(vals[4])), DXL_HIBYTE(DXL_HIWORD(vals[4]))]
        param_goal_position6 = [DXL_LOBYTE(DXL_LOWORD(vals[5])), DXL_HIBYTE(DXL_LOWORD(vals[5])), DXL_LOBYTE(DXL_HIWORD(vals[5])), DXL_HIBYTE(DXL_HIWORD(vals[5]))]
        param_goal_position7 = [DXL_LOBYTE(DXL_LOWORD(vals[6])), DXL_HIBYTE(DXL_LOWORD(vals[6])), DXL_LOBYTE(DXL_HIWORD(vals[6])), DXL_HIBYTE(DXL_HIWORD(vals[6]))]
        param_goal_position8 = [DXL_LOBYTE(DXL_LOWORD(vals[7])), DXL_HIBYTE(DXL_LOWORD(vals[7])), DXL_LOBYTE(DXL_HIWORD(vals[7])), DXL_HIBYTE(DXL_HIWORD(vals[7]))]
        param_goal_position9 = [DXL_LOBYTE(DXL_LOWORD(vals[8])), DXL_HIBYTE(DXL_LOWORD(vals[8])), DXL_LOBYTE(DXL_HIWORD(vals[8])), DXL_HIBYTE(DXL_HIWORD(vals[8]))]
        param_goal_position10 = [DXL_LOBYTE(DXL_LOWORD(vals[9])), DXL_HIBYTE(DXL_LOWORD(vals[9])), DXL_LOBYTE(DXL_HIWORD(vals[9])), DXL_HIBYTE(DXL_HIWORD(vals[9]))]
        param_goal_position11 = [DXL_LOBYTE(DXL_LOWORD(vals[10])), DXL_HIBYTE(DXL_LOWORD(vals[10])), DXL_LOBYTE(DXL_HIWORD(vals[10])), DXL_HIBYTE(DXL_HIWORD(vals[10]))]
        param_goal_position12 = [DXL_LOBYTE(DXL_LOWORD(vals[11])), DXL_HIBYTE(DXL_LOWORD(vals[11])), DXL_LOBYTE(DXL_HIWORD(vals[11])), DXL_HIBYTE(DXL_HIWORD(vals[11]))]
        param_goal_position13 = [DXL_LOBYTE(DXL_LOWORD(vals[12])), DXL_HIBYTE(DXL_LOWORD(vals[12])), DXL_LOBYTE(DXL_HIWORD(vals[12])), DXL_HIBYTE(DXL_HIWORD(vals[12]))]
        param_goal_position14 = [DXL_LOBYTE(DXL_LOWORD(vals[13])), DXL_HIBYTE(DXL_LOWORD(vals[13])), DXL_LOBYTE(DXL_HIWORD(vals[13])), DXL_HIBYTE(DXL_HIWORD(vals[13]))]
        param_goal_position15 = [DXL_LOBYTE(DXL_LOWORD(vals[14])), DXL_HIBYTE(DXL_LOWORD(vals[14])), DXL_LOBYTE(DXL_HIWORD(vals[14])), DXL_HIBYTE(DXL_HIWORD(vals[14]))]
        param_goal_position16 = [DXL_LOBYTE(DXL_LOWORD(vals[15])), DXL_HIBYTE(DXL_LOWORD(vals[15])), DXL_LOBYTE(DXL_HIWORD(vals[15])), DXL_HIBYTE(DXL_HIWORD(vals[15]))]
        param_goal_position17 = [DXL_LOBYTE(DXL_LOWORD(vals[16])), DXL_HIBYTE(DXL_LOWORD(vals[16])), DXL_LOBYTE(DXL_HIWORD(vals[16])), DXL_HIBYTE(DXL_HIWORD(vals[16]))]
        param_goal_position18 = [DXL_LOBYTE(DXL_LOWORD(vals[17])), DXL_HIBYTE(DXL_LOWORD(vals[17])), DXL_LOBYTE(DXL_HIWORD(vals[17])), DXL_HIBYTE(DXL_HIWORD(vals[17]))] 
        
        # Combine parameters into single groupSyncWrite command:
        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL1_ID, param_goal_position1)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
            quit()
        
        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL2_ID, param_goal_position2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
            quit()
        
        # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL3_ID, param_goal_position3)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
            quit()

        # Add Dynamixel#4 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL4_ID, param_goal_position4)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL4_ID)
            quit()

        # Add Dynamixel#5 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL5_ID, param_goal_position5)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL5_ID)
            quit()

        # Add Dynamixel#6 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL6_ID, param_goal_position6)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL6_ID)
            quit()
            
        # Add Dynamixel#7 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL7_ID, param_goal_position7)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL7_ID)
            quit()

        # Add Dynamixel#8 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL8_ID, param_goal_position8)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL8_ID)
            quit()

        # Add Dynamixel#9 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL9_ID, param_goal_position9)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL9_ID)
            quit()

        # Add Dynamixel#10 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL10_ID, param_goal_position10)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL10_ID)
            quit()

        # Add Dynamixel#11 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL11_ID, param_goal_position11)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL11_ID)
            quit()

        # Add Dynamixel#12 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL12_ID, param_goal_position12)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL12_ID)
            quit()
            
        # Add Dynamixel#13 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL13_ID, param_goal_position13)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL13_ID)
            quit()

        # Add Dynamixel#14 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL14_ID, param_goal_position14)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL14_ID)
            quit()

        # Add Dynamixel#15 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL15_ID, param_goal_position15)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL15_ID)
            quit()

        # Add Dynamixel#16 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL16_ID, param_goal_position16)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL16_ID)
            quit()

        # Add Dynamixel#17 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL17_ID, param_goal_position17)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL17_ID)
            quit()

        # Add Dynamixel#18 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = self.groupSyncWrite.addParam(DXL18_ID, param_goal_position18)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL18_ID)
            quit()
        
        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()       
     
    
    #def read_sag_motor_positions(self):
    #    # this function reads the motor positions for the sagittal joints
    #    p1=self.motor01.get_position()
    #    p2=self.motor02.get_position()
    #    p3=self.motor11.get_position()
    #    p4=self.motor12.get_position()
    #    p5=self.motor15.get_position()
    #    p6=self.motor16.get_position()
    #    p=[p1,p2,p3,p4,p5,p6]
    #    return p

            
    def publish_all_motor_commands(self,angs):
        # this function publishes commanded joint angles to all 18 publisher topics
        self._pub_joints["r_shoulder_sagittal"].publish(angs[0])
        self._pub_joints["l_shoulder_sagittal"].publish(angs[1])
        self._pub_joints["r_shoulder_frontal"].publish(angs[2])
        self._pub_joints["l_shoulder_frontal"].publish(angs[3])
        self._pub_joints["r_elbow"].publish(angs[4])
        self._pub_joints["l_elbow"].publish(angs[5])
        self._pub_joints["r_hip_swivel"].publish(angs[6])
        self._pub_joints["l_hip_swivel"].publish(angs[7])
        self._pub_joints["r_hip_frontal"].publish(angs[8])
        self._pub_joints["l_hip_frontal"].publish(angs[9])
        self._pub_joints["r_hip_sagittal"].publish(angs[10])
        self._pub_joints["l_hip_sagittal"].publish(angs[11])
        self._pub_joints["r_knee"].publish(angs[12])
        self._pub_joints["l_knee"].publish(angs[13])
        self._pub_joints["r_ankle_sagittal"].publish(angs[14])
        self._pub_joints["l_ankle_sagittal"].publish(angs[15])
        self._pub_joints["r_ankle_frontal"].publish(angs[16])
        self._pub_joints["l_ankle_frontal"].publish(angs[17])
    
    def publish_sag_cmds(self,angs):
        # the function publishes the commanded angles to the corresponding publisher topics:
        self._pub_joint_cmds["r_shoulder_sagittal"].publish(angs[0])
        self._pub_joint_cmds["l_shoulder_sagittal"].publish(angs[1])
        self._pub_joint_cmds["r_hip_sagittal"].publish(angs[2])
        self._pub_joint_cmds["l_hip_sagittal"].publish(angs[3])
        self._pub_joint_cmds["r_ankle_sagittal"].publish(angs[4])
        self._pub_joint_cmds["l_ankle_sagittal"].publish(angs[5])        

    def publish_sag_angs(self,angs):
        # the function publishes the commanded angles to the corresponding publisher topics:
        self._pub_joints["r_shoulder_sagittal"].publish(angs[0])
        self._pub_joints["l_shoulder_sagittal"].publish(angs[1])
        self._pub_joints["r_hip_sagittal"].publish(angs[2])
        self._pub_joints["l_hip_sagittal"].publish(angs[3])
        self._pub_joints["r_ankle_sagittal"].publish(angs[4])
        self._pub_joints["l_ankle_sagittal"].publish(angs[5])             
        
    def convert_angles_to_commands(self,ids,angles):
        # this function converts an array of angle values (in radians) to the corresponding 10-bit motor values,
        # assuming that the 'ids' array contains matching ID numbers for the angle values of 'angles' array
        
        b=[60,240,60,240,150,150,150,150,150,150,150,150,150,150,150,150,150,150] # motor offsets
        c=[1,1,-1,1,-1,-1,-1,1,1,1,-1,-1,-1,-1,-1,-1,1,1] # motor polarities
        
        cmds=np.zeros(len(ids)) # initialize cmds array
        for j in range(len(ids)): # get 10-bit motor values:
            num=ids[j] # get motor ID number from array
            ang=angles[j] # get desired joint angle value in radians
            cmds[j]=(1023/300)*((ang*(180/math.pi)*c[num-1])+b[num-1]) # convert to degrees, apply polarity, convert to 10-bit, add offset
        return cmds

    def convert_motor_positions_to_angles(self,ids,cmds):
        # this function converts an array of angle values (in radians) to the corresponding 10-bit motor values,
        # assuming that the 'ids' array contains matching ID numbers for the angle values of 'angles' array
        
        b=[60,240,60,240,150,150,150,150,150,150,150,150,150,150,150,150,150,150] # motor offsets
        c=[1,1,-1,1,-1,-1,-1,1,1,1,-1,-1,-1,-1,-1,-1,1,1] # motor polarities
        
        angs=np.zeros(len(ids)) # initialize cmds array
        for j in range(len(ids)): # get 10-bit motor values:
            num=ids[j] # get motor ID number from array
            cmd=cmds[j] # get motor position as 10-bit value
            angs[j]=(((300/1023)*cmd)-b[num-1])*c[num-1]*(math.pi/180) # convert from 10-bit, subtract offset, apply polarity, go to degrees
        return angs    
