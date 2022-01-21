from commands import Commands
from serialInterface import SerialInterface

import time
import numpy as np
import datetime
from datetime import datetime
import os



class Y3SpaceDriver(SerialInterface):
    def __init__(self, baudrate, timeout):
        super().__init__(baudrate=baudrate, timeout=timeout)
        self.commands = Commands()

    def __del__(self):
        pass

    def restoreFactorySettings(self):
        self.writeCommand(self.commands.RESTORE_FACTORY_SETTINGS)

    def getSoftwareVersion(self):
        self.writeCommand(self.commands.GET_FIRMWARE_VERSION_STRING)

        version = self.readLine()
        print(f"Software Version: {version}")
        return version

    def getAxisDirection(self):
        self.writeCommand(self.commands.GET_AXIS_DIRECTION)

        buffer = self.readLine()

        direction = self.direction(buffer)
        print(f"Axis Direction: {direction}")
        return direction


    def direction(self, buf):
        if(buf == "0"):
            return "X: Right, Y: Up, Z: Forward"
        elif ( buf == "1"):
            return "X: Right, Y: Forward, Z: Up"
        elif ( buf == "2"):
            return "X: Up, Y: Right, Z: Forward"
        elif (buf == "3"):
            return "X: Forward, Y: Right, Z: Up"
        elif( buf == "4"):
            return "X: Up, Y: Forward, Z: Right"
        elif( buf == "5"):
            return "X: Forward, Y: Up, Z: Right"
        elif (buf == "19"):
            return "X: Forward, Y: Left, Z: Up"
        else:
            return "Unknown"


    def startGyroCalibration(self):

        print("Starting Auto Gyro Calibration...")
        self.writeCommand(self.commands.BEGIN_GYRO_AUTO_CALIB)
        time.sleep(5)
        print("Proceeding")

    def setMIMode(self, on):
        if(on):
            self.writeCommand(self.commands.SET_MI_MODE_ENABLED)
        else:
            self.writeCommand(self.commands.SET_MI_MODE_DISABLED)

    def getCalibMode(self):
        self.writeCommand(self.commands.GET_CALIB_MODE)

        buffer = self.readLine()

        calib = ''

        if(buffer == "0"):
            calib = "Bias"
        elif(buffer == "1"):
            calib = "Scale and Bias"
        else:
            calib = "Unknown"

        print(f"Calibration Mode: {calib}")
        return calib


    def getMIMode(self):
        self.writeCommand(self.commands.GET_MI_MODE_ENABLED)

        buffer = self.readLine()

        mimode = ''

        if(buffer == "0"):
            mimode = "Disabled"
        elif(buffer == "1"):
            mimode = "Enabled"
        else:
            mimode = "Unknown"

        print(f"MI Mode: {mimode}")
        return mimode

    def run(self):
        self.prepare()
        while True:
            (roll, pitch, yaw) = self.get_angles()
            print(f"roll: {roll} / pitch: {pitch} / yaw: {yaw}")



    def prepare(self):
        self.startGyroCalibration()
        self.getSoftwareVersion()
        self.getAxisDirection()
        self.getCalibMode()
        self.getMIMode()

        self.writeCommand(self.commands.SET_STREAMING_SLOTS_QUATERNION)
        self.writeCommand(self.commands.TARE_WITH_CURRENT_ORIENTATION)
        self.writeCommand(self.commands.TARE_WITH_CURRENT_QUATERNION)

        self.writeCommand(self.commands.START_STREAMING)
        print("Ready...")

    def get_angles(self):
        rate = 10 #ms
        line = 0
        buffer_total = ""
        while(True):
            line += 1
            buffer = self.readLine()
            buffer_total = buffer_total+','+buffer

            if(line == 4):
                buffer_total = buffer_total.split(',')[1:]
               # print(buffer_total)
                line = 0
                #if(len(buffer_total)==10):
                orientation_x          = float(buffer_total[0])
                orientation_y          = float(buffer_total[1])
                orientation_z          = float(buffer_total[2])
                orientation_w          = float(buffer_total[3])
                angular_velocity_x     = float(buffer_total[4])
                angular_velocity_y     = float(buffer_total[5])
                angular_velocity_z     = float(buffer_total[6])
                linear_acceleration_x  = float(buffer_total[7])
                linear_acceleration_y  = float(buffer_total[8])
                linear_acceleration_z  = float(buffer_total[9])

                (roll, pitch, yaw) = self.quaternion_to_euler_angle_vectorized(
                    orientation_x,
                    orientation_y,
                    orientation_z,
                    orientation_w
                )
                return (roll, pitch, yaw)


    def quaternion_to_euler_angle_vectorized(self, x, y, z, w):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = np.degrees(np.arctan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = np.where(t2>+1.0,+1.0,t2)
        #t2 = +1.0 if t2 > +1.0 else t2

        t2 = np.where(t2<-1.0, -1.0, t2)
        #t2 = -1.0 if t2 < -1.0 else t2
        Y = np.degrees(np.arcsin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = np.degrees(np.arctan2(t3, t4))

        return X, Y, Z
