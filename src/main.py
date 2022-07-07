#!/usr/bin/env python

import rospy
from ros_igtl_bridge.msg import igtltransform, igtlstring
from ros_galil_2022.srv import Status, Config
from std_msgs.msg import Float64
import numpy

# State Machine
NONE = 0    # robot not connected yet
CONNECT = 1 # robot connected to 3DSlicer
INIT = 2    # initializing
ZFRAME = 3  # define zFrame transform
IDLE = 4    # waiting
TARGET = 5  # define target
MOVE = 6    # move to target


class Interface:
    def __init__(self):

        # ROS Topics
        rospy.Subscriber('IGTL_STRING_IN', igtlstring, self.callbackString)
        rospy.Subscriber('IGTL_TRANSFORM_IN', igtltransform, self.callbackTransformation)

        self.angle1 = rospy.Publisher('IGTL_STRING_OUT', Float64)
        self.angle2 = rospy.Publisher('IGTL_STRING_OUT', igtlstring, queue_size=10)


        # Variables
        #TODO: Discuss definition of flags and states
        self.flagInit = False
        self.flagAngle = False
        self.state = IDLE

        # Initialize the node and name it.
        rospy.init_node('interface')
        rospy.loginfo('Interface Node')

        # Set timer for state machine loop
        self.rate = rospy.Rate(10) #10hz

#################################################################################################
#####    Callback Functions for subscribed topics     ###########################################
#################################################################################################

    # Received STRING from 3DSlicer OpenIGTLink Bridge
    def callbackString(self, msg):
        rospy.loginfo(rospy.get_caller_id() + 'Received command %s', msg.name)

#TODO ==========
        if msg.name == 'INIT':
            initCondition = msg.data[4] + msg.data[5]
        elif msg.name == 'MOVE':
            rospy.wait_for_service('move_motors')
            try:
                move_motors = rospy.ServiceProxy('move_motors', Config)
                if (move_motors(angles)):
                    rospy.loginfo("motors moved")
                else:
                    rospy.loginfo("Could not initialize motors")
            except rospy.ServiceException as e:
                rospy.loginfo("Controller service call failed: %s" % e)
        else:
            rospy.loginfo('Invalid message, returning to IDLE state')
            self.state = IDLE
#================


    # Received TRANSFORM from 3DSlicer OpenIGTLink Bridge
    def callbackTransformation(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'Received transformation ' + data.name)
        if data.name == 'ANGLE':
            pos = numpy.array(
                [data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
            quat = numpy.array([data.transform.rotation.w, data.transform.rotation.x, data.transform.rotation.y,
                                data.transform.rotation.z])
            self.angles = self.quaternion2ht(quat, pos)
            self.flagAngle = True


        else:
            rospy.loginfo('Invalid message, returning to IDLE state')
            self.state = IDLE

    def quaternion2ht(self,quat,pos):
        H = numpy.matrix('1.0 0.0 0.0 0.0; 0.0 1.0 0.0 0.0 ; 0.0 0.0 1.0 0.0; 0.0 0.0 0.0 1.0')
        H[0, 3] = pos[0]
        H[1, 3] = pos[1]
        H[2, 3] = pos[2]

        H[0, 0] = 1.0-2.0*(quat[2]*quat[2]+quat[3]*quat[3])
        H[0, 1] = 2.0*(quat[1]*quat[2]-quat[0]*quat[3])
        H[0, 2] = 2.0*(quat[1]*quat[3]+quat[0]*quat[2])

        H[1, 0] = 2.0*(quat[1]*quat[2]+quat[0]*quat[3])
        H[1, 1] = 1.0-2.0*(quat[1]*quat[1]+quat[3]*quat[3])
        H[1, 2] = 2.0*(quat[2]*quat[3]-quat[0]*quat[1])

        H[2, 0] = 2.0*(quat[1]*quat[3]-quat[0]*quat[2])
        H[2, 1] = 2.0*(quat[2]*quat[3]+quat[0]*quat[1])
        H[2, 2] = 1.0-2.0*(quat[1]*quat[1]+quat[2]*quat[2])

        return H


def main():
    try:
        #Instantiate object of node class
        interface = Interface()
    except rospy.ROSInterruptException:
        rospy.loginfo('Could not initialize Interface Node')

    while not rospy.is_shutdown():
        #NONE State - Wait for OpenIGTLink Connection
        if (interface.state == IDLE):
            rospy.loginfo("Waiting")

        elif (interface.state==MOVE):
            if (interface.flagAngle):
                rospy.loginfo("Move the robot...")

        #Do nothing
        else:
            pass
        interface.rate.sleep()

if __name__ == '__main__':
    main()
