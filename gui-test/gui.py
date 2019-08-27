#!/usr/bin/env python

 ############################################################
 #  Created on: 2013.04                                     #
 #  Author: LIAO Wenlong                                    #
 #  Email:  Volans.liao@gmail.com                           #

 #  This is part of the Localize and Navigation Toolkit     #
 ############################################################

from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtCore import Qt


import sys, math, copy, threading, subprocess, inspect, time
from math import sin, cos, atan2

import Node,os
import sys; [sys.path.append( i ) for i in ['..', '../msg', './msg']]
import stdmsg

from ConfigParser import SafeConfigParser
parser = SafeConfigParser()
try:
    parser.read('gui.ini')
    HOST = parser.get('url', 'host')
    PORT_RPC = parser.get('url', 'rpc')
    PORT_SUB = parser.get('url', 'sub')
except :
    HOST = "127.0.0.1"
    PORT_RPC = '9000'
    PORT_SUB = '9001'''
    

class VirtualJoystick(QtGui.QMainWindow):   
    x_min = -0.50
    x_max = 0.50
    r_min =  -0.5
    r_max =  0.5
    def __init__(self):  
        super(VirtualJoystick, self).__init__()
        self.timer_rate = 50
        self.initUI()
        self.nh = Node.Node("tcp://*:9010")
        vel = stdmsg.Velocity()
        vel.v = 0
        vel.w = 0
        self.nh.publish('velocity', vel)
           
    def initUI(self):      
        img_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "\pics\crosshair.jpg"
        img_path = img_path.replace('\\', '/')
        self.statusBar()
        self.setStyleSheet("border-image: url(%s); " % img_path)
        
        self.setGeometry(100, 100, 200, 200)
        self.setWindowTitle('Virtual Joystick')
        self.show()
        self.timer = QtCore.QBasicTimer()
        self.statusBar().showMessage('started')
        
    def mousePressEvent(self, event):
        self.statusBar().showMessage('mouse clicked')
        self.timer.start(self.timer_rate, self)
        self.get_position(event)
           
    def mouseReleaseEvent(self, event): 
        self.statusBar().showMessage('mouse released')
        self.timer.stop()
        vel = stdmsg.Velocity()
        vel.v = 0
        vel.w = 0
        self.nh.publish('velocity', vel)
          
    def mouseMoveEvent(self, event):  
        self.get_position(event)
        
    def get_position(self, event): 
        s = self.size()
        s_w = s.width()
        s_h = s.height()
        pos = event.pos()
        self.x = 1.0 * pos.x() / s_w
        self.y = 1.0 * pos.y() / s_h
        
    def timerEvent(self, event): 
        self.pubTwist()
        
    def pubTwist(self):
        v = (1-self.y) * (self.x_max - self.x_min) + self.x_min
        w = (1-self.x) * (self.r_max - self.r_min) + self.r_min
        
        if v > self.x_max:
            v = self.x_max
        if v < self.x_min:
            v = self.x_min
        if w > self.r_max:
            w = self.r_max
        if w < self.r_min:
            w = self.r_min
    
        self.statusBar().showMessage('vel (%0.2f, %0.2f)' % (v, w))
        vel = stdmsg.Velocity()
        vel.v = v
        vel.w = w
        self.nh.publish('velocity', vel)
   
class AutoPath:
    goal_list = []
    current_goal_index = 0
    def __init__(self):
        fp = open('path.txt')
        for line in fp:
            argv = line.split(' ')
            if len(argv) < 3: continue
            x = float( argv[0] )
            y = float( argv[1] )
            t = float( argv[2] )
            self.goal_list.append( [ x, y, t ] )
        fp.close()
    def next(self):
        x,y,t = self.goal_list[self.current_goal_index]
        self.current_goal_index += 1
        if self.current_goal_index >= len( self.goal_list ): 
            self.current_goal_index = 0
        return x, y, t
class QMainWindow(QtGui.QMainWindow):
    # to indicate the server is running or not
    
    initialed = False
    goal_reached_time = time.time()
    autopath = AutoPath()
    def __init__(self, *args ):
        super(QMainWindow, self).__init__()
        self.ui = uic.loadUi('navigation.ui', self)
        self.view = self.graphicsView
        
        self.timer = QtCore.QTimer()
        self.connect(self.timer, QtCore.SIGNAL('timeout()'),   self.update)
        
        self.connect_host()
        self.timer.setInterval(20)
        self.timer.start()

       # self.stopRobot.setEnabled(True);
       # self.startRobot.setEnabled(True);


    def resizeEvent(self, e):
        if not self.initialed:
            self.initialed = True
            return
        size = e.size() - e.oldSze()
        self.dial.move(self.dial.x() + size.width(), self.dial.y() + size.height() )
        
        
    def update(self):
        self.nh.run(0.001)

    def startGoal (self):
        cal = stdmsg.String()
        cal.str = 'AUTONOMOS'
        print cal
        self.rpc.call('set_mode',cal, 1)
        self.view.startGoal()
        joystick.hide()

    def stopGoal (self):
        cal = stdmsg.String()
        cal.str = 'MANUAL'
        print cal
        self.rpc.call('set_mode',cal, 1)
        self.view.stopGoal()
        joystick.show()

 #   def startTopoTask (self):
#
 #       pass
    def setHost(self):
        import socket
        while 1:
            ip, ok = QtGui.QInputDialog.getText(self, "host ip", "IP address:" );
            if not ok:return None
            ip = str(ip)
            try:socket.inet_aton(ip)
            except: continue
            break
        parser = SafeConfigParser()
        parser.add_section('url')
        parser.set('url', 'host', ip )
        parser.set('url', 'rpc',  PORT_RPC)
        parser.set('url', 'sub', PORT_SUB)
        parser.write(open('gui.ini','w'))
        
    def connect_host(self):      
        self.rpc = Node.RPC()
        self.nh = Node.Node("tcp://*:5165")
        
        self.nh.subscrible('laser',self.ScanHandler)

        print 'connect to', HOST
        self.nh.subscrible('scan',self.ScanHandler)
        self.nh.subscrible('global_plan',self.PlanHandler)  
        
        self.rpc.connect("tcp://"+HOST+":"+PORT_RPC)
        self.nh.connect( "tcp://"+HOST+":"+PORT_SUB)
        
        self.view.init(self.rpc, self)

    @Node.msg_callback(stdmsg.Laser_Scan)
    def ScanHandler(self, scan):
        minr = min( scan.ranges )
        for index in range( 0, len( scan.ranges) ):
            if scan.ranges[index] == minr: 
                break
        #print scan.steer, minr, index
        #print scan.robot
        self.view.ScanHandler(scan)
        self.laserDisplay.setLaser( scan, 0.2 )
        #print tuple(scan.ranges)

    @Node.msg_callback(stdmsg.Global_Plan)
    def PlanHandler(self, plan):
        print plan.goal_reached,
        if plan.goal_reached and time.time() - self.goal_reached_time > 5:
            self.goal_reached_time = time.time()
            x,y,theta = self.autopath.next()
            pos = stdmsg.Pose()
            pos.position.x = x
            pos.position.y = y
            pos.orentation.yaw = theta
            print 'goal_set is :', x,y,theta
            self.rpc.call('set_goal', pos )
        self.view.PlanHandler(plan)

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    gui = QMainWindow()   
    joystick = VirtualJoystick()
    joystick.hide()
    gui.show()
    app.exec_()



