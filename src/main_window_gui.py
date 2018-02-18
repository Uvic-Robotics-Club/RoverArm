#!/usr/bin/env python
"""
Created on Wed Feb 14 12:01:38 2018

@author: joell
"""

from PyQt4 import QtCore, QtGui, uic  # Import the PyQt4 module we'll need
import sys  # We need sys so that we can pass argv to QApplication
import rospy
import threading

from RoverArm.msg import arm_velocity



global_msg = arm_velocity()

class MainScreen(QtGui.QMainWindow):
    def __init__(self):
        '''
        This is the main screen that the user interacts with.

        '''
        super(self.__class__, self).__init__()
        uic.loadUi('/home/student/catkin_ws/src/RoverArm/src/Main Window.ui', self)
        rospy.init_node('gui_listener', anonymous=True)
        rospy.Subscriber("Arm/AngleVelocities", arm_velocity, callback)
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.updateFromGlobal)
        timer.start(100)


    def __internalClose__(self):
        '''
        this is an internal only method that closes the main window
        '''
        self.close()

    def updateFromGlobal(self):
        global global_msg
        '''
        This function is called on a timer. This updates the main screen with values from rover.

        Currently implemented are:

        #. sending motor speeds
        #. displaying camera feed from rover
        #. showing messages from other parts of the program

        '''
        # global_msg is the place where all the data is stored
        self.lower_axis.setValue(global_msg.joint.lower*50+50)
        self.upper_axis.setValue(global_msg.joint.upper*50+50)
        self.rotate_axis.setValue(global_msg.joint.rotate*50+50)
        self.gripper_axis.setValue(global_msg.joint.gripper*50+50)
        self.speed_axis.setValue(global_msg.joint.speed*50+50)
        self.lower_lock.setChecked(not global_msg.enable.lower)
        self.upper_lock.setChecked(not global_msg.enable.upper)
        self.rotation_lock.setChecked(not global_msg.enable.rotate)
        self.speed_lock.setChecked(global_msg.enable.speed)
        if(rospy.is_shutdown()):
			self.__internalClose__()





def startGUI():
    '''
    This starts the Main screen in the main loop. When you try and start a Qt application from a thread it complains.
    This function is blocking.
    '''
    global form
    app = QtGui.QApplication(sys.argv)  # A new instance of QApplication
    form = MainScreen()  # We set the form to be our ExampleApp (design)
    form.show()  # Show the form
    app.exec_()  # and execute the app
    return

def callback(data):
    global global_msg
    global_msg = data

def StartROS():
    print "nothing"


if __name__ == '__main__':  # if we're running file directly and not importing it
    #t1 = threading.Thread(target=StartROS)
    #t1.start()
    startGUI()
