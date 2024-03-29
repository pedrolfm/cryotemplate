#!/usr/bin/env python

import rospy
from ros_igtl_bridge.msg import igtltransform, igtlstring
from cryotemplate.srv import Status, Angles
#from ros_galil_2022.srv import Status, Config
from std_msgs.msg import Float32
import numpy


# State Machine
NONE = 0    # robot not connected yet
CONNECT = 1 # robot connected to 3DSlicer
INIT = 2    # initializing
ZFRAME = 3  # define zFrame transform
IDLE = 4    # waiting
ANGLE = 5  # define target
MOVE = 6    # move to target

# UPDATE WITH VALUES PROVIDED BY PSI
CONST1 = 1000
CONST2 = 2000
# =================================

class Interface:
    def __init__(self):
        # ROS Topics
        rospy.Subscriber('IGTL_STRING_IN', igtlstring, self.callbackString)
        rospy.Subscriber('IGTL_TRANSFORM_IN', igtltransform, self.callbackTransformation)

        self.statusMsg = rospy.Publisher('IGTL_STRING_OUT',igtlstring,queue_size=10)
        self.angle1 = rospy.Publisher('alpha', Float32,queue_size=10)
        self.angle2 = rospy.Publisher('beta', Float32,queue_size=10)

        self.move_motors = rospy.ServiceProxy('move_motors', Angles)
        self.getStatus = rospy.ServiceProxy('get_status', Status)

        # Variables
        #TODO: Discuss definition of flags and states
        self.flagInit = False
        self.flagAngle = False
        self.state = IDLE
        self.statusData = igtlstring()
        self.statusData.name = "status"
        
        # Initialize the node and name it.
        rospy.init_node('interface')
        rospy.loginfo('Interface Node')

        # Set timer for state machine loop
        self.rate = rospy.Rate(0.5) 


        

#################################################################################################
#####    Callback Functions for subscribed topics     ###########################################
#################################################################################################

    def getAndSendAtatusCallback(self):
        print("testing timer")
        rospy.wait_for_service('get_status')
        self.getStatus("teste")


    # Received STRING from 3DSlicer OpenIGTLink Bridge
    def callbackString(self, msg):
        rospy.loginfo(rospy.get_caller_id() + 'Received command %s', msg.name)
        if msg.name == 'INIT':
            initCondition = msg.data[4] + msg.data[5]
        elif msg.name == 'MOVE':
            if self.flagAngle:
                rospy.wait_for_service('move_motors')
                self.move_motors(self.angles[0],self.angles[1])
            else:
                rospy.loginfo('No angles received yet')
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
            self.angles = self.quaternion2angle(quat)
            print(self.angles)
            self.flagAngle = True
        else:
            rospy.loginfo('Invalid message, returning to IDLE state')
            self.state = IDLE

    def quaternion2angle(self,quat):
         o1 = 2*(quat[0] * quat[1] + quat[2] * quat[3])
         o2 = 1-2*(quat[1]*quat[1]+quat[2]*quat[2])
         angle1 = numpy.arctan2(o1,o2)
         angle2 = numpy.arcsin(2*(quat[0]*quat[2]-quat[3]*quat[1]))
         angle3 = numpy.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),1-2*(quat[2]*quat[2]+quat[3]*quat[3]))
         return [angle1,angle2,angle3]

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
        interface.rate.sleep()
        #NONE State - Wait for OpenIGTLink Connection
        if (interface.state == IDLE):
            rospy.loginfo("Waiting")
            interface.statusData.data = str(interface.getStatus())
            interface.statusMsg.publish(interface.statusData)
            print(interface.statusData.data)
        elif (interface.state==MOVE):
            rospy.loginfo("Move mode.")
            if (interface.flagAngle):
                rospy.loginfo("Move the robot...")
            interface.state = IDLE
        elif (interface.state==ANGLE):
            temp = Float32
            temp = interface.angles[0]
            rospy.loginfo("received angle insformation")
            interface.angle1.publish(interface.angles[0])
            interface.angle2.publish(interface.angles[1])
            interface.state = IDLE
        #Do nothing
        else:
            pass
        interface.rate.sleep()

if __name__ == '__main__':
    main()

