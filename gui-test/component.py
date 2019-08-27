#!/usr/bin/env python
# -*- coding: utf-8 -*-
 ############################################################
 #  Created on: 2013.04                                     #
 #  Author: LIAO Wenlong                                    #
 #  Email:  Volans.liao@gmail.com                           #
 #  This is part of the Localize and Navigation Toolkit     #
 ############################################################

from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtCore import Qt
import sys, math, copy, threading
from math import sin, cos, atan2
import stdmsg
import ConfigParser


flag = False
class LaserScan (QtGui.QGraphicsItem):
    Type = QtGui.QGraphicsItem.UserType + 3

    def __init__(self, scan, resolution = 0.1):
        super(LaserScan, self).__init__()
        self.setAcceptDrops(False)
        # global  flag
        # if (flag == False):
        #    pos = stdmsg.Pose()
        #    try:
        #        f2 = open('../robot.txt')
        #        via1 = f2.read()
        #        via1 = via1.split(',')
        #        x1 = float(via1[0])
        #        y1 = float(via1[1])
        #        theta1 = float(via1[2])
        #        f2.close()
        #    except:
        #        x1=0
        #        y1=0
        #        theta1=0
        #        f = open('../robot.txt', 'r+')
        #        mat = "%.3f,%.3f,%.3f," % (0, 0, 0)
        #        f.write(mat)
        #        f.close()
        #    pos.position.x = x1
        #    pos.position.y = y1
        #    pos.orentation.yaw = theta1
        #    if rpc != None:
        #         rpc.call('initial_pose', pos)
        #    else:
        #         print 'rpc not set'
        # flag = True
        self.scan = scan
        self.resolution = resolution
        self.normalize()


    def normalize(self):
        self.points = QtGui.QPolygonF()
        minx, miny, maxx, maxy = 1000, 1000, -1000, -1000
        angle = self.scan.config.angle_min + self.scan.pose.orentation.yaw
        for i in range(0, len(self.scan.ranges) ):
            if self.scan.ranges[i] >= self.scan.config.range_max - 1.5:
                angle += self.scan.config.angle_increment
                continue
            
            lx = cos( angle ) * self.scan.ranges[i] + self.scan.pose.position.x
            ly = sin( angle ) * self.scan.ranges[i] + self.scan.pose.position.y
            lx /= self.resolution
            ly /= self.resolution
            angle += self.scan.config.angle_increment
            self.points.append( QtCore.QPointF(lx, ly) )
            if (lx > maxx): maxx = lx;
            if (lx < minx): minx = lx;
            if (ly > maxy): maxy = ly;
            if (ly < miny): miny = ly;
        
        self.rect = QtCore.QRectF(minx, miny, maxx - minx, maxy - miny)

    def type(self):
        return LaserScan.Type

    def boundingRect(self):
        return self.rect

    def paint(self, qp, option, widget):
        pen = QtGui.QPen(Qt.red)
        pen.setWidth(1)
        qp.setPen(pen)
        qp.drawPoints(self.points)

class Robot (QtGui.QGraphicsItem):
    Type = QtGui.QGraphicsItem.UserType + 3

    def __init__(self, resolution = 1):
        super(Robot, self).__init__()
        self.setAcceptDrops(False)
        self.pic = QtGui.QPixmap('./pics/agv.png')
        self.scale( 1.1/(self.pic.width() / 20.0 * 12) / resolution, -1.1/(self.pic.width() / 20.0 * 12) /resolution )
        self.setZValue(1)

    def type(self):
        return Robot.Type

    def boundingRect(self):
        return QtCore.QRectF(-self.pic.width() / 2.0,- self.pic.height() / 2.0,
                              self.pic.width(), self.pic.height() ) 

    def paint(self, qp, option, widget):
        qp.translate( -self.pic.width()/2,-self.pic.height()/2)
        qp.drawPixmap(0,0,self.pic)
        
class laserWidget(QtGui.QWidget):
    resolution = 1
    def __init__(self, parent = None):
        super(laserWidget, self).__init__(parent)
        self.points = QtGui.QPolygonF()
    def setLaser(self, scan, resolution):
        self.resolution = resolution
        self.points = QtGui.QPolygonF()
        angle = scan.config.angle_min + scan.pose.orentation.yaw
        for i in range(0, len(scan.ranges) ):
            if scan.ranges[i] >= scan.config.range_max - 1.5:
                angle += scan.config.angle_increment
                continue
            
            lx = cos( angle ) * scan.ranges[i] + 1
            ly = sin( angle ) * scan.ranges[i] + 1
            lx /= self.resolution
            ly /= self.resolution
            angle += scan.config.angle_increment
            self.points.append( QtCore.QPointF(lx, ly) )
        self.repaint()
        
    def paintEvent(self, e):
        qp = QtGui.QPainter()
        pen = QtGui.QPen()
        pen.setWidth(1)
        qp.begin(self)
        qp.setPen(pen)

        gradient = QtGui.QRadialGradient(self.width()/2.0 , self.height(), self.height())
        gradient.setFocalPoint( self.width()/2.0, self.height())
        gradient.setColorAt(1, QtGui.QColor(Qt.gray).light(120))
        gradient.setColorAt(0, QtGui.QColor(Qt.gray).light(240))
        qp.setBrush(gradient)
        qp.drawRect(0,0,self.width(),self.height())
        qp.translate(self.width()/2.0 ,self.height())

        qp.rotate(-90)
        qp.drawPoints(self.points)
        qp.end()
    
class Map (QtGui.QGraphicsItem):
    Type = QtGui.QGraphicsItem.UserType + 3

    GOAL, POSE = range(0,2)
    SETMODE = None
    resolution = 0.05

    def __init__(self, map, resolution = 0.1):
        super(Map, self).__init__()
        self.map = map
        self.resolution = resolution
        self.getGoal = False
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable,False)
        self.setAcceptDrops(False)

    def SetGetGoal(self, flag):
        self.getGoal = flag


    def type(self):
        return Map.Type

    def boundingRect(self):
        return QtCore.QRectF( 0,0, self.map.width(), self.map.height() )

    def paint(self, qp, option, widget):
        qp.setPen(Qt.red)
        qp.drawPixmap(0,0,self.map)
        if self.SETMODE != None and hasattr( self, 'startPos') and hasattr(self, 'endPos') and self.endPos != None:
            qp.drawLine(self.startPos.x(), self.startPos.y(), self.endPos.x(),self.endPos.y())

    def mousePressEvent(self, e):
        self.startPos = e.pos()
        

    def mouseMoveEvent(self, e):
        if self.SETMODE == None:
            return
        self.endPos = e.pos()
        self.update()
        

    def mouseReleaseEvent (self, e):
        if self.SETMODE == None:
            return
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable,False)
        x = self.startPos.x() * self.resolution
        y = self.startPos.y() * self.resolution
        theta = math.atan2( e.pos().y() - self.startPos.y(), \
                                e.pos().x() - self.startPos.x() )
        
        try:
            if self.SETMODE == self.POSE:
                pos = stdmsg.Pose()
                pos.position.x = x
                pos.position.y = y
                pos.orentation.yaw = theta
                print 'initial_pose is :', x,y,theta
                if(rpc!= None):rpc.call('initial_pose', pos )
                else: print 'rpc not set'
            elif self.SETMODE == self.GOAL:
                pos = stdmsg.Pose()
                pos.position.x = x
                pos.position.y = y
                pos.orentation.yaw = theta
                print 'initial_pose is :', x,y,theta
                if(rpc!= None):rpc.call('set_goal', pos )
                else: print 'rpc not set'
        except e: 
            print e, 'socket error!'
        self.SETMODE = None
    
        
    def set_pose (self):
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable,True)
        self.SETMODE = self.POSE
        self.endPos = None
    
    def set_goal (self):
        self.setFlag(QtGui.QGraphicsItem.ItemIsSelectable,True)
        self.SETMODE = self.GOAL
        self.endPos = None
        #print self.SETMODE
        
    def go_org(self,index):
  #      self.mappingset.currentindex()
  #     QtGui.QComboBox.currentIndex()+1
  #      getcombo = QtGui.QComboBox
  ##      pos = stdmsg.Pose()
  ##       f2 = open('origin.txt')
  ##       via = f2.read()
  ##       via = via.split(',')
  ##       x1 = float(via[0])
  ##       y1 = float(via[1])
  ##       theta1 = float(via[2])
  ##       f2.close()
  ##       pos.position.x = x1
  ##       pos.position.y = y1
  ##       pos.orentation.yaw = theta1
  ##       print 'initial_pose is :', x1, y1, theta1
  ##       if rpc != None:
  ##           rpc.call('initial_pose', pos)
  ##       else:
  ##           print 'rpc not set'
		
        pos = stdmsg.Pose()		
        mappingset = "../mapping_point.ini"
        mapRect= ConfigParser.ConfigParser()
        mapRect.read(mappingset)
        liststr=mapRect.sections()
        if len(liststr)>=1 and liststr[index]=='pos1':    ##匹配点1
            X=float(mapRect.get("pos1","x"))
            Y = float(mapRect.get("pos1", "y"))
            Qt = float(mapRect.get("pos1", "th"))
        if len(liststr)>=1 and liststr[index]=='pos2':    ##匹配点2
            X=float(mapRect.get("pos2","x"))
            Y = float(mapRect.get("pos2", "y"))
            Qt = float(mapRect.get("pos2", "th"))
        if len(liststr)>=1 and liststr[index]=='pos3':    ##匹配点3
            X=float(mapRect.get("pos3","x"))
            Y = float(mapRect.get("pos3", "y"))
            Qt = float(mapRect.get("pos3", "th"))
        if len(liststr)>=1 and liststr[index]=='pos4':    ##匹配点4
            X=float(mapRect.get("pos4","x"))
            Y = float(mapRect.get("pos4", "y"))
            Qt = float(mapRect.get("pos4", "th"))
        if len(liststr)>=1 and liststr[index]=='pos5':    ##匹配点5
            X=float(mapRect.get("pos5","x"))
            Y = float(mapRect.get("pos5", "y"))
            Qt = float(mapRect.get("pos5", "th"))
        if len(liststr)>=1 and liststr[index]=='pos6':    ##匹配点6
            X=float(mapRect.get("pos6","x"))
            Y = float(mapRect.get("pos6", "y"))
            Qt = float(mapRect.get("pos6", "th"))
		
        pos.position.x = X
        pos.position.y = Y
        pos.orentation.yaw = Qt
        print 'the current_pose is :', X, Y, Qt
        if rpc != None:
            rpc.call('initial_pose', pos)
        else:
            print 'rpc not set'

    def go_firstmatch(self):
        pos1 = stdmsg.Pose()
        f2 = open('origin1.txt')
        via = f2.read()
        via = via.split(',')
        x1 = float(via[0])
        y1 = float(via[1])
        theta1 = float(via[2])
        f2.close()
        pos1.position.x = x1
        pos1.position.y = y1
        pos1.orentation.yaw = theta1
        print 'first match pose is :', x1, y1, theta1
        if rpc != None:
            rpc.call('initial_pose', pos1)
        else:
            print 'rpc not set'
            
    def go_secondmatch2(self):
        pos1 = stdmsg.Pose()
        f2 = open('origin2.txt')
        via = f2.read()
        via = via.split(',')
        x1 = float(via[0])
        y1 = float(via[1])
        theta1 = float(via[2])
        f2.close()
        pos1.position.x = x1
        pos1.position.y = y1
        pos1.orentation.yaw = theta1
        print 'second match pose is :', x1, y1, theta1
        if rpc != None:
            rpc.call('initial_pose', pos1)
        else:
            print 'rpc not set'

class inputdlg(QtGui.QDialog):
    def __init__(self, arg):
        super(inputdlg, self).__init__()
        self.resize(240, 150)
        
        grid = QtGui.QGridLayout()
        tishi = QtGui.QLabel("input ordinate")
        self.zb = QtGui.QLineEdit('0,0,0')
        grid.addWidget(tishi,0,0)
        grid.addWidget(self.zb,0,1)
       
        buttonBox = QtGui.QDialogButtonBox(parent=self)
        buttonBox.setOrientation(QtCore.Qt.Horizontal) 
        buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok) # 确定和取消两个按钮
        # 连接信号和槽
        buttonBox.accepted.connect(self.accept) # 确定
        buttonBox.rejected.connect(self.reject) # 取消
        
        layout = QtGui.QVBoxLayout()
 
        # 加入前面创建的表格布局
        layout.addLayout(grid)

        # ButtonBox
        layout.addWidget(buttonBox)      
 
        self.setLayout(layout)

    def outzb(self):
        return self.zb.text()

class GraphWidget(QtGui.QGraphicsView):
    resolution = 0.1
    focusOnRobot = False
    showHistoryRobot = False
    historyRobotItemList = []
    Reflecor_flag=0

    # TimerFlag = 1
    # list_time=[0 for i in range(60)]
    # list_dqx=[0 for i in range(60)]
    # list_dqy=[0 for i in range(60)]
    # list_dqth=[0 for i in range(60)]
    # list_index=0
    
    def __init__(self, parent):
        super(GraphWidget, self).__init__()
        self.scene = QtGui.QGraphicsScene(self)
        self.setScene(self.scene)

        self.scale(1,-1)
        self.setCacheMode(QtGui.QGraphicsView.CacheBackground)
        self.setViewportUpdateMode(QtGui.QGraphicsView.BoundingRectViewportUpdate)
        self.setTransformationAnchor(QtGui.QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QtGui.QGraphicsView.AnchorViewCenter)
        self.setDragMode(QtGui.QGraphicsView.ScrollHandDrag)
        self.setBackgroundBrush( QtGui.QBrush(QtGui.QColor(240,  240, 240), Qt.Dense3Pattern))

        self.path = None
        self.scan = None
        self.map = None
        self.bd = 0

    def init(self, r, parentMain):
        global rpc
        rpc = r
        
        self.parentMain = parentMain
        self.is_save = False
        self.is_playing = False

        self.requestMap()
        self.robot = Robot(self.resolution)
        self.scene.addItem( self.robot )

    def saveMessage(self):
        if not self.is_save:
            self.is_save = True
            self.parentMain.ui.saveMessage.setText(u"停止并保存记录数据")
            ppos = stdmsg.Pose()
            ppos.position.x = 0
            ppos.position.y = 0
            ppos.orentation.yaw = 0
            rpc.call('saveMessage', ppos)
        else: 
            self.is_save = False
            self.parentMain.ui.saveMessage.setText(u"开始记录数据")
            ppos = stdmsg.Pose()
            ppos.position.x = 1
            ppos.position.y = 0
            ppos.orentation.yaw = 0
            rpc.call('saveMessage', ppos)
        
    def stop_and_play(self):
        if not self.is_playing:
            print("stop play")
            self.is_playing = True
            self.parentMain.ui.stopandplay.setText(u"播放")
            ppos = stdmsg.Pose()
            ppos.position.x = 0
            ppos.position.y = 0
            ppos.orentation.yaw = 0
            rpc.call('StopAndPlay', ppos)
        else: 
            print("play")
            self.is_playing = False
            self.parentMain.ui.stopandplay.setText(u"停止")
            ppos = stdmsg.Pose()
            ppos.position.x = 1
            ppos.position.y = 0
            ppos.orentation.yaw = 0
            rpc.call('StopAndPlay', ppos)
               ##用于控制数据读取的快慢
    def forward(self):
        ppos = stdmsg.Pose()
        ppos.position.x = 0
        ppos.position.y = 0
        ppos.orentation.yaw = 0
        if rpc != None:
            rpc.call('Forward', ppos)
        else:
            print 'rpc not set'

    def backward(self):
        ppos = stdmsg.Pose()
        ppos.position.x = 0
        ppos.position.y = 0
        ppos.orentation.yaw = 0
        if rpc != None:
            rpc.call('Backward', ppos)
        else:
            print 'rpc not set'
    def accelerate(self):
        ppos = stdmsg.Pose()
        ppos.position.x = 0
        ppos.position.y = 0
        ppos.orentation.yaw = 0
        if rpc != None:
            rpc.call('Accelerate', ppos)
        else:
            print 'rpc not set'
    def decelerate(self):
        ppos = stdmsg.Pose()
        ppos.position.x = 0
        ppos.position.y = 0
        ppos.orentation.yaw = 0
        if rpc != None:
            rpc.call('Decelerate', ppos)
        else:
            print 'rpc not set'


        
    def requestMap(self):
        # if self.map != None:
        #     self.scene.removeItem( self.map )
            
        
        # rmap = stdmsg.Data()
        # ret = rpc.call('map', rmap , 1)
        # #print len(ret)
        # if ret != None:    
        #     rmap.ParseFromString( ret )
        # else:
        #     print 'rpc error !'
            
        # #print len(rmap.data)
        # import tempfile
        # f = open('map.png', 'wb')
        # f.write( rmap.data )
        # f.close()

        img = QtGui.QImage('map.png')

            

        png = QtGui.QPixmap.fromImage( img )
        import struct
        s = struct.pack('I',img.pixel(0,0))
        s = s[1:3]+s[0]+s[3]
        self.resolution = struct.unpack('f',s)[0]
        self.map = Map( png, self.resolution )
        
        self.scene.addItem( self.map )
        
    def goorg(self):
      #  print QtGui.QComboBox.currentIndex()
        index=0
        self.map.go_org(index)

    def goorg1(self):
        #self.map.go_firstmatch()
        index=1
        self.map.go_org(index)

    def goorg2(self):
        #self.map.go_secondmatch2()
        index=2
        self.map.go_org(index)
        
    def goorg3(self):
        index=3
        self.map.go_org(index)
        
    def goorg4(self):
        index=4
        self.map.go_org(index)
        
    def goorg5(self):
        index=5
        self.map.go_org(index)

    def enterSetGoal(self):
        self.map.set_goal()

    def enterSetPose(self):
        self.map.set_pose()
        # self.TimerFlag = 1
        
    def SetReflector(self):
        self.Reflecor_flag= 1- self.Reflecor_flag
        Reflag = stdmsg.Orentation()
        Reflag.yaw=self.Reflecor_flag
        if rpc != None:
            rpc.call('set_reflector', Reflag)
        else:
            print 'rpc error !'

    def wheelEvent(self, event):
        self.scaleView(math.pow(2.0, event.delta() / 240.0))

    def scaleView(self, scaleFactor):
        factor = self.matrix().scale(scaleFactor, scaleFactor).\
                mapRect(QtCore.QRectF(0, 0, 1, 1)).width()
        if factor < 0.01 or factor > 100:
            return
        self.scale(scaleFactor, scaleFactor)

    def zoom (self, num):
        factor = pow(2.713,num/50.0)
        nowFactor = self.transform().mapRect(QtCore.QRectF(0, 0, 1, 1)).width()
        if (factor < 0.07 or factor > 100):
            return
        self.scale(factor/nowFactor, factor/nowFactor)
        
    def setFocusOnRobot (self, flag):
        self.focusOnRobot = flag
        if self.focusOnRobot:
            self.setDragMode(QtGui.QGraphicsView.NoDrag)
            self.centerOn(self.robot)
        else:
            self.setDragMode(QtGui.QGraphicsView.ScrollHandDrag)
    def setShowHistoryRobot (self, flag):
        self.showHistoryRobot = flag

        if( not self.showHistoryRobot):
            for item in self.historyRobotItemList:
                self.scene.removeItem(item)
            self.historyRobotItemList = []
            
    def startGoal (self):
        return
        try:
            print 'start_robot'
            client.send('start_robot ')
            print client.recv(flags=zmq.NOBLOCK)
        except:
            print 'socket error!'
    def stopGoal (self):
        return
        try:
            print 'stop_robot'
            client.send('stop_robot ')
            print client.recv(flags=zmq.NOBLOCK)
        except:
            print 'socket error!'

    def biaod(self):
        
        indlg = inputdlg(QtGui.QDialog)
        if indlg.exec_():
            zb1 = indlg.outzb() 
            
            #f.write(zb1)
            zb1 = zb1.split(',')
            self.bdx = float(zb1[0])
            self.bdy = float(zb1[1])
            self.bdth = float(zb1[2])

         
            rotd = (self.dqth - self.bdth) 
            pya = self.bdy * math.sin(rotd) - self.bdx * math.cos(rotd) + self.dqx 
            pyb = -self.bdx * math.sin(rotd) - self.bdy * math.cos(rotd) + self.dqy 

            #print zb1


          #  f = open('../input.txt','r+')
          #  matx = "%.3f,%.3f,%.3f" % (pya,pyb,rotd)
          #  f.write('1,')
          #  f.write(matx)
             #   self.bd += 1
          #  f.close()
            #else:
            #    f = open('../input.txt','a')
            #   f.write(',')
            #    matx = "%.3f,%.3f,%.3f" % (pya,pyb,rotd)
            #    f.write(matx)
                        ################# Amended by jzy,NOT revise###########
            Origin = "../input.ini"
            congui = ConfigParser.ConfigParser()
            congui.read(Origin)
            x_origin = "%f" % pya
            y_origin = "%f" % pyb
            th_origin = "%f" % rotd
            congui.set("Origin", "x_origin", x_origin)
            congui.set("Origin", "y_origin", y_origin)
            congui.set("Origin", "th_origin", th_origin)
            congui.write(open(Origin, "w"))
            #######################################################

            print  "%.3f,%.3f,%.3f" % (pya,pyb,rotd)


    def ScanHandler(self, scan):
        if self.scan != None:
            self.scene.removeItem(self.scan)
            self.scan = None

        self.scan = LaserScan(scan, self.resolution )
        self.scene.addItem(self.scan)

        self.dqx = scan.robot.position.x
        self.dqy = scan.robot.position.y
        self.dqth = scan.robot.orentation.yaw
        print self.dqx,self.dqy,self.dqth

        # if abs(self.dqx)<10000 and abs(self.dqy)<=10000 and abs(self.dqth)<=3.14:
        #     f = open('../robot.txt','r+')
        #     matx = "%.3f,%.3f,%.3f," % (self.dqx,self.dqy,self.dqth)
        #     f.write(matx)
        #     f.close()

        # f=open('../pflg.txt','r+')
        # pflg=int(f.read())
        # if(pflg==0 ):
        #     self.TimerFlag = 0
        #     if self.list_index != 60:
        #         self.list_dqx[self.list_index] = self.dqx
        #         self.list_dqy[self.list_index] = self.dqy
        #         self.list_dqth[self.list_index] = self.dqth
        #         self.list_time[self.list_index] = time.time()
        #         self.list_index += 1
        #     else:
        #         self.list_index = 0
        # if(pflg==1 and self.TimerFlag==0):
        #     self.list_index=self.list_time.index(min(self.list_time))
        #     pos = stdmsg.Pose()
        #     pos.position.x = self.list_dqx[self.list_index]
        #     pos.position.y = self.list_dqy[self.list_index]
        #     pos.orentation.yaw = self.list_dqy[self.list_index]
        #     if rpc != None:
        #         rpc.call('initial_pose', pos)
        #     else:
        #         print 'rpc not set'

        
        self.robot.setPos(scan.robot.position.x / self.resolution, scan.robot.position.y / self.resolution)
        self.robot.setRotation( -scan.robot.orentation.yaw / 3.141592653*180.0)
        self.robot.update()
        if self.focusOnRobot:
            self.centerOn(self.robot)
        if self.showHistoryRobot:
            point = QtCore.QPointF (scan.robot.position.x / self.resolution - 1, \
                                    scan.robot.position.y / self.resolution - 1)
            
            self.historyRobotItemList.append( \
                        self.scene.addRect( \
                                            point.x(),point.y(), 1, 1,
                                            QtGui.QPen(Qt.red),QtGui.QBrush(Qt.blue)\
                                           ) )

    def PlanHandler(self, plan):
        if self.path != None:
            self.scene.removeItem(self.path)
            self.path = None
        plan = plan.path
        if len(plan) == 0:
            return

        path = QtGui.QPainterPath()
        path.moveTo( plan[0].position.x/self.resolution, plan[0].position.y/self.resolution )
        for i in plan[1:]:
            path.lineTo( i.position.x/self.resolution,i.position.y/self.resolution )
        #for i in plan[1:]:
        #    path.addEllipse( i[0]/0.1 - 0.5, i[1]/0.1 - 0.5, 1, 1)
        self.path = self.scene.addPath(path)
        self.path.setZValue(10)
    
