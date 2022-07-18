#!/usr/bin/env python

import rospy
import serial
import time
from cryotemplate.srv import Status, Angles

SHARP = chr(35)  # String '#'

# Relationship between encoder counts and mm:
US_MM_2_COUNT = 2000.0 / 2.5349
PE_MM_2_COUNT = 500  # 5500.0/16.51


class Communication:

    def __init__(self):

        # Initialize the node and name it.
        rospy.init_node('communication', anonymous=True)
        rospy.loginfo('Communication Node')

        # Services

        rospy.Service('move_motors', Angles, self.move_motors)
        rospy.Service('get_status', Status, self.getStatus)

        # Open serial connection with Galil
        if self.open_connection():
            # Set all motors to absolute motion
            if self.set_absolute_motion(['A', 'B', 'C', 'D']) and self.check_controller():
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

    # Verify controller configuration (motors and encoders)
    # TODO: Find out if really necessary
    def check_controller(self):
        try:
            self.ser.write(str("EO 0\r"))
            time.sleep(0.1)

            self.ser.flushInput()
            self.ser.write(str("MT ?\r"))
            time.sleep(0.1)
            MT1 = float(self.ser.read(4))

            self.ser.flushInput()
            self.ser.write(str("MT ,?\r"))
            time.sleep(0.1)
            MT2 = float(self.ser.read(4))

            self.ser.flushInput()
            self.ser.write(str("CE ?\r"))
            time.sleep(0.1)
            CE1 = float(self.ser.read(2))

            self.ser.flushInput()
            self.ser.write(str("CE ,?\r"))
            time.sleep(0.1)
            CE2 = float(self.ser.read(2))

            if MT1 == 1.0 and MT2 == 1.0 and CE1 == 0.0 and CE2 == 0.0:
                rospy.loginfo('Controller verified')
                return 1
            else:
                rospy.loginfo('Wrong motor or encoder configuration - Check Galil setup')
                return 0

        except:
            rospy.loginfo("*** could not send request to Galil ***")
            return 0

    #################################################################################################
    #####    Service Functions from Controller node    ##############################################
    #################################################################################################

    # Return current serial connection status with Galil controller
    def get_controller_connection_status(self):
        return self.ser.isOpen()
        # return self.controllerConnection

    # Initialize requested motors
    def move_motors(self, req):
        rospy.loginfo('here')
        print("here")
        print(req)
        return True

    def getStatus(self):
        rospy.loginfo('here status')
        return "hare is status"


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
