#!/usr/bin/env python
"""
Created on Wed Feb 14 12:01:38 2018

@author: joell
"""
from __future__ import unicode_literals
from PyQt4 import QtCore, QtGui, uic  # Import the PyQt4 module we'll need
import sys  # We need sys so that we can pass argv to QApplication
import rospy
import threading

from RoverArm.msg import arm_velocity
from RoverArm.msg import joint_angles
from RoverArm.srv import angles_to_points,point_to_angle

from geometry_msgs.msg import Point
from sensor_msgs.msg import Image


import sys
import os
import random
from matplotlib.backends import qt_compat
use_pyside = qt_compat.QT_API == qt_compat.QT_API_PYSIDE
if use_pyside:
    from PySide import QtGui, QtCore
else:
    from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import QImage
from numpy import arange, sin, pi
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np



# Gloabl Ros message
global_msg = arm_velocity()
global_feedback_msg = joint_angles()

arm_fk = rospy.ServiceProxy('arm_fk',angles_to_points)
arm_ik = rospy.ServiceProxy('arm_ik',point_to_angle)

class MyMplCanvas(FigureCanvas):
    """Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)."""

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111, projection='3d')
        self.axes.set_xlabel('x')
        self.axes.set_ylabel('y')
        self.axes.set_zlabel('z')
        

        self.compute_initial_figure()

        FigureCanvas.__init__(self, fig)
        self.setParent(parent)

        FigureCanvas.setSizePolicy(self,
                                   QtGui.QSizePolicy.Expanding,
                                   QtGui.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)
        plt.ion()
        axes3d.Axes3D.mouse_init(self.axes)
        

    def compute_initial_figure(self):
        pass

class MyDynamicMplCanvas(MyMplCanvas):
    """A canvas that updates itself every second with a new plot."""

    def __init__(self, *args, **kwargs):
        MyMplCanvas.__init__(self, *args, **kwargs)
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.update_figure)
        timer.start(1000)

    def compute_initial_figure(self):
        self.axes.plot([0, 1, 2, 3], [1, 2, 0, 4], 'r')
        axes3d.Axes3D.mouse_init(self.axes)
        

    def update_figure(self):
        # Build a list of 4 random integers between 0 and 10 (both inclusive)
        l = [random.randint(0, 10) for i in range(4)]
        self.axes.cla()
        self.axes.plot([0, 1, 2, 3], l, 'r')
        #arm_fk
        #self.axes.scatter3D(x_joints[0], y_joints[0], z_joints[0], color='g', label='p1')
        #self.axes.scatter3D(x_joints[1], y_joints[1], z_joints[1], color='r', label='p2')
        #self.axes.scatter3D(x_joints[2], y_joints[2], z_joints[2], color='m', label='p3')
        #self.axes.scatter3D(x_joints[3], y_joints[3], z_joints[3], color='k', label='p4')
        self.axes.set_xlabel('x')
        self.axes.set_ylabel('y')
        self.axes.set_zlabel('z')
        self.draw()
       

class MainScreen(QtGui.QMainWindow):
    def __init__(self):
        '''
        This is the main screen that the user interacts with.

        '''
        super(self.__class__, self).__init__()
        uic.loadUi('/home/student/catkin_ws/src/RoverArm/src/Main Window.ui', self)
        #l = self.verticalLayout
        #dc = MyDynamicMplCanvas(self, width=5, height=4, dpi=100)
        #l.addWidget(dc)
        self.wd  = 640
        self.ht = 480



        rospy.init_node('gui_listener', anonymous=True)
        rospy.Subscriber("Arm/AngleVelocities", arm_velocity, callback)
        rospy.Subscriber("Arm/Feedback",joint_angles, feedback_callback)
        self.arm_pub = rospy.Publisher("Arm/Goal", Point,queue_size=10)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.qimage = None
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.updateFromGlobal)
        timer.start(100)
        
        self.dial.valueChanged.connect(self.setPosition)
        self.horizontalSlider.valueChanged.connect(self.setPosition)
        self.verticalSlider.valueChanged.connect(self.setPosition)

    def image_callback(self,msg):
        #rospy.logerr("got a new image")
        np_arr = np.fromstring(msg.data, np.uint8)
        sz = (msg.height, msg.width, msg.step / msg.width)
        image = np.reshape(np_arr, sz)
        self.qimage = QImage.rgbSwapped(QImage(image.tostring(),self.wd, self.ht, QImage.Format_RGB888))
        
    def __internalClose__(self):
        '''
        this is an internal only method that closes the main window
        '''
        self.close()
    
    def setPosition(self, data):
        global global_feedback_msg
        new_message = Point()
        new_message.x = (self.horizontalSlider.value())/100.0
        new_message.y = (self.verticalSlider.value())/100.0
        new_message.z = (self.dial.value())/100.0
        self.arm_pub.publish(new_message)

    def updateFromGlobal(self):
        global global_msg
        global global_feedback_msg
        global arm_fk
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
        if self.qimage is not None:
            self.picture_placeholder.setPixmap(QtGui.QPixmap.fromImage(self.qimage))
        try:
            ans = arm_fk(global_feedback_msg)
            self.actual_x_LCD.display(ans.end_of_link_two.x)
            self.actual_y_LCD.display(ans.end_of_link_two.y)
            self.actual_z_LCD.display(ans.end_of_link_two.z)
        except Exception:
            pass
        
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
    
def feedback_callback(data):
    global global_feedback_msg
    global_feedback_msg = data



def StartROS():
    print "nothing"


if __name__ == '__main__':  # if we're running file directly and not importing it
    #t1 = threading.Thread(target=StartROS)
    #t1.start()
    startGUI()
