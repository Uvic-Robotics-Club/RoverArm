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
        uic.loadUi('Main Window.ui', self)

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.updateFromGlobal)
        timer.start(100)


        self.mplwidget.axes=Axes3D(self.mplwidget.figure)

        self.mplwidget.axes.plot([-.5,.5],[ 0,0],[0,0],color='k')
        self.mplwidget.axes.plot([0,0],[ -.5,.5],[0,0],color='k')

        self.mplwidget.axes.set_xlim((-10,10))
        self.mplwidget.axes.set_ylim((-10,10))


        self.mplwidget.axes.set_xlabel('x')
        self.mplwidget.axes.set_ylabel('y')
        self.mplwidget.axes.set_zlabel('z')

    def __internalClose__(self,button,value):
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
        self.lower_axis.setValue(global_msg.joint.lower)
        self.upper_axis.setValue(global_msg.joint.upper)
        self.rotate_axis.setValue(global_msg.joint.rotate)
        self.gripper_axis.setValue(global_msg.joint.gripper)
        self.speed_axis.setValue(global_msg.joint.speed)
        self.lower_lock.setChecked(global_msg.enable.lower)
        self.upper_lock.setChecked(global_msg.enable.upper)
        self.rotate_lock.setChecked(global_msg.enable.rotate)
        self.speed_lock.setChecked(global_msg.enable.speed)






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
    rospy.init_node('gui_listener', anonymous=True)
    rospy.Subscriber("Arm/AngleVelocities", arm_velocity, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':  # if we're running file directly and not importing it
    t1 = threading.Thread(target=StartROS)
    t1.start()
    startGUI()
