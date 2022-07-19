#!/usr/bin/env python

import rospy
import serial
import time
from cryotemplate.srv import Status, Angles

SHARP = chr(35)  # String '#'

# Relationship between encoder counts and mm:
US_DEG_2_COUNT = 2000.0 / 2.5349
PE_DEG_2_COUNT = 500  # 5500.0/16.51


class Communication:

    def __init__(self):

        # Initialize the node and name it.
        rospy.init_node('communication', anonymous=True)
        rospy.loginfo('Communication Node')

        # Services

        rospy.Service('move_motors', Angles, self.move_motors)
        rospy.Service('get_status', Status, self.getStatus)
        rospy.loginfo('Services implemented')

        # Open serial connection with Galil
        if self.open_connection():
            # Set all motors to absolute motion
            self.absoluteMode = self.set_absolute_motion(['A', 'B'])
            if self.absoluteMode:
                rospy.loginfo('Controller initialized')

    #################################################################################################
    #####    Functions from Controller node    ######################################################
    #################################################################################################

    # Open serial connection with Galil board
    # TODO: Make it a loop
    def open_connection(self):
        try:
            # Try serial ttyUSB0
            self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)  # open serial port
            self.connectionStatus = True
            rospy.loginfo('Connected to Galil - ttyUSB0')
            return 1
        except:
            try:
                # Try serial ttyUSB1
                self.ser = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=1)  # open serial port
                self.connectionStatus = True
                rospy.loginfo('Connected to Galil - ttyUSB1')
                return 1
            except:
                self.connectionStatus = False
                rospy.loginfo("*** could not open the serial communication ***")
                return 0

    # Set motor channels for absolute value
    def set_absolute_motion(self, channels):
        for channel in channels:
            try:
                self.ser.write(str("DP%s=0\r" % channel))
                self.ser.write(str("PT%s=1\r" % channel))
                self.ser.write(str("SH;"))
                rospy.loginfo('Channel ' + channel + ' in absolute mode')
            except:
                rospy.loginfo("*** Could not set Galil to absolute mode ***")
                return 0
        return 1

    # Send motor absolute position
    def SendAbsolutePosition(self,Channel,X):
        try:
            if self.absoluteMode:
                self.ser.write(str("SH;"))
                time.sleep(0.01)
                self.ser.write(str("PT%s=1;" % Channel))
                time.sleep(0.01)
                self.ser.write(str("PA%s=%d;" % (Channel,X)))
                return 1
            else:
                rospy.loginfo("*** PA not available ***")
                return 0
        except:
            rospy.loginfo("*** could not send command ***")
            return 0

    def getMotorPosition(self):
        try:
            self.ser.flushInput()
            time.sleep(0.5)
            self.ser.write(str("TP;"))
            time.sleep(0.1)
            bytesToRead = self.ser.inWaiting()
            data_temp = self.ser.read(bytesToRead-3)
            print(data_temp)
        except:
            print("*** could not send command ***")
            return str(0)
        return data_temp


    #################################################################################################
    #####    Service Functions from Communication node    ##############################################
    #################################################################################################

    # Return current serial connection status with Galil controller
    def get_controller_connection_status(self):
        return self.ser.isOpen()
        # return self.controllerConnection

    # Initialize requested motors
    def move_motors(self, req):
        rospy.loginfo('Send commands to Galil')
        SendAbsolutePosition("A",req[0]*US_DEG_2_COUNT)
        SendAbsolutePosition("A",req[1]*PE_DEG_2_COUNT)
        return True

    def getStatus(self,req):
        rospy.loginfo('here status')
        return self.getMotorPosition()


def main():
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        # Initialize controller
        comm = Communication()
    except rospy.ROSInterruptException:
        rospy.loginfo('Could not initialize Controller Node')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
