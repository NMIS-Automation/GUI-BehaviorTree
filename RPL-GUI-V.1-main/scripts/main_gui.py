# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main_gui.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1451, 1108)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(280, 0, 1171, 1061))
        self.tabWidget.setObjectName("tabWidget")
        self.simTab = QtWidgets.QWidget()
        self.simTab.setObjectName("simTab")
        self.rvizView = QtWidgets.QWidget(self.simTab)
        self.rvizView.setGeometry(QtCore.QRect(359, 9, 791, 1011))
        self.rvizView.setObjectName("rvizView")
        self.sensorView3 = QtWidgets.QWidget(self.simTab)
        self.sensorView3.setGeometry(QtCore.QRect(10, 770, 341, 251))
        self.sensorView3.setObjectName("sensorView3")
        self.sensorViewLabel3 = QtWidgets.QLabel(self.simTab)
        self.sensorViewLabel3.setGeometry(QtCore.QRect(120, 740, 101, 17))
        self.sensorViewLabel3.setObjectName("sensorViewLabel3")
        self.sensorView2 = QtWidgets.QWidget(self.simTab)
        self.sensorView2.setGeometry(QtCore.QRect(10, 470, 341, 251))
        self.sensorView2.setObjectName("sensorView2")
        self.sensorViewLabel2 = QtWidgets.QLabel(self.simTab)
        self.sensorViewLabel2.setGeometry(QtCore.QRect(120, 440, 101, 17))
        self.sensorViewLabel2.setTextFormat(QtCore.Qt.PlainText)
        self.sensorViewLabel2.setObjectName("sensorViewLabel2")
        self.sensorView1 = QtWidgets.QWidget(self.simTab)
        self.sensorView1.setGeometry(QtCore.QRect(10, 180, 341, 251))
        self.sensorView1.setObjectName("sensorView1")
        self.sensorViewLabel1 = QtWidgets.QLabel(self.simTab)
        self.sensorViewLabel1.setGeometry(QtCore.QRect(120, 150, 101, 17))
        self.sensorViewLabel1.setObjectName("sensorViewLabel1")
        self.loadView = QtWidgets.QPushButton(self.simTab)
        self.loadView.setGeometry(QtCore.QRect(10, 10, 89, 31))
        self.loadView.setObjectName("loadView")
        self.saveView = QtWidgets.QPushButton(self.simTab)
        self.saveView.setGeometry(QtCore.QRect(10, 50, 89, 31))
        self.saveView.setObjectName("saveView")
        self.viewNameLoad = QtWidgets.QTextEdit(self.simTab)
        self.viewNameLoad.setGeometry(QtCore.QRect(120, 10, 181, 31))
        self.viewNameLoad.setObjectName("viewNameLoad")
        self.viewNameSave = QtWidgets.QTextEdit(self.simTab)
        self.viewNameSave.setGeometry(QtCore.QRect(120, 50, 181, 31))
        self.viewNameSave.setObjectName("viewNameSave")
        self.newSensorView = QtWidgets.QToolButton(self.simTab)
        self.newSensorView.setGeometry(QtCore.QRect(10, 90, 251, 31))
        self.newSensorView.setObjectName("newSensorView")
        self.refreshSensors = QtWidgets.QPushButton(self.simTab)
        self.refreshSensors.setGeometry(QtCore.QRect(270, 90, 31, 31))
        self.refreshSensors.setText("")
        self.refreshSensors.setObjectName("refreshSensors")
        self.tabWidget.addTab(self.simTab, "")
        self.graphTab = QtWidgets.QWidget()
        self.graphTab.setObjectName("graphTab")
        self.kpiView = QtWidgets.QWidget(self.graphTab)
        self.kpiView.setGeometry(QtCore.QRect(10, 70, 1151, 851))
        self.kpiView.setObjectName("kpiView")
        self.kpiName = QtWidgets.QTextBrowser(self.graphTab)
        self.kpiName.setGeometry(QtCore.QRect(0, 10, 1171, 931))
        self.kpiName.setObjectName("kpiName")
        self.kpiName.raise_()
        self.kpiView.raise_()
        self.tabWidget.addTab(self.graphTab, "")
        self.graphViewTab = QtWidgets.QWidget()
        self.graphViewTab.setObjectName("graphViewTab")
        self.graphView = QtWidgets.QWidget(self.graphViewTab)
        self.graphView.setGeometry(QtCore.QRect(10, 10, 1151, 911))
        self.graphView.setObjectName("graphView")
        self.tabWidget.addTab(self.graphViewTab, "")
        self.logoView = QtWidgets.QLabel(self.centralwidget)
        self.logoView.setGeometry(QtCore.QRect(60, 10, 151, 141))
        self.logoView.setText("")
        self.logoView.setPixmap(QtGui.QPixmap("../imgs/nmis-mini-logo.png"))
        self.logoView.setScaledContents(True)
        self.logoView.setObjectName("logoView")
        self.logPoseName = QtWidgets.QTextEdit(self.centralwidget)
        self.logPoseName.setGeometry(QtCore.QRect(120, 270, 141, 31))
        self.logPoseName.setObjectName("logPoseName")
        self.logPose = QtWidgets.QPushButton(self.centralwidget)
        self.logPose.setGeometry(QtCore.QRect(20, 270, 81, 31))
        self.logPose.setObjectName("logPose")
        self.motionTypeSet = QtWidgets.QToolButton(self.centralwidget)
        self.motionTypeSet.setGeometry(QtCore.QRect(20, 540, 241, 31))
        self.motionTypeSet.setPopupMode(QtWidgets.QToolButton.MenuButtonPopup)
        self.motionTypeSet.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.motionTypeSet.setAutoRaise(False)
        self.motionTypeSet.setObjectName("motionTypeSet")
        self.setReference = QtWidgets.QPushButton(self.centralwidget)
        self.setReference.setGeometry(QtCore.QRect(20, 350, 81, 31))
        self.setReference.setObjectName("setReference")
        self.setReferenceTopic = QtWidgets.QTextEdit(self.centralwidget)
        self.setReferenceTopic.setGeometry(QtCore.QRect(120, 350, 141, 31))
        self.setReferenceTopic.setObjectName("setReferenceTopic")
        self.editPoses = QtWidgets.QPushButton(self.centralwidget)
        self.editPoses.setGeometry(QtCore.QRect(20, 390, 241, 25))
        self.editPoses.setObjectName("editPoses")
        self.textBrowser_4 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_4.setGeometry(QtCore.QRect(10, 170, 261, 261))
        self.textBrowser_4.setObjectName("textBrowser_4")
        self.textBrowser_5 = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser_5.setGeometry(QtCore.QRect(10, 440, 261, 291))
        self.textBrowser_5.setObjectName("textBrowser_5")
        self.pointSelected1 = QtWidgets.QToolButton(self.centralwidget)
        self.pointSelected1.setGeometry(QtCore.QRect(20, 490, 241, 31))
        self.pointSelected1.setPopupMode(QtWidgets.QToolButton.MenuButtonPopup)
        self.pointSelected1.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        self.pointSelected1.setAutoRaise(False)
        self.pointSelected1.setObjectName("pointSelected1")
        self.runButton = QtWidgets.QPushButton(self.centralwidget)
        self.runButton.setGeometry(QtCore.QRect(20, 910, 71, 31))
        self.runButton.setObjectName("runButton")
        self.saveAllMotions = QtWidgets.QPushButton(self.centralwidget)
        self.saveAllMotions.setGeometry(QtCore.QRect(20, 960, 71, 31))
        self.saveAllMotions.setObjectName("saveAllMotions")
        self.loadButton = QtWidgets.QPushButton(self.centralwidget)
        self.loadButton.setGeometry(QtCore.QRect(20, 1010, 71, 31))
        self.loadButton.setObjectName("loadButton")
        self.runProgress = QtWidgets.QProgressBar(self.centralwidget)
        self.runProgress.setGeometry(QtCore.QRect(110, 912, 151, 31))
        self.runProgress.setProperty("value", 0)
        self.runProgress.setObjectName("runProgress")
        self.saveName = QtWidgets.QTextEdit(self.centralwidget)
        self.saveName.setGeometry(QtCore.QRect(110, 960, 151, 31))
        self.saveName.setObjectName("saveName")
        self.loadName = QtWidgets.QTextEdit(self.centralwidget)
        self.loadName.setGeometry(QtCore.QRect(110, 1010, 151, 31))
        self.loadName.setObjectName("loadName")
        self.textBrowser = QtWidgets.QTextBrowser(self.centralwidget)
        self.textBrowser.setGeometry(QtCore.QRect(10, 861, 261, 201))
        self.textBrowser.setObjectName("textBrowser")
        self.addMotionBttn = QtWidgets.QPushButton(self.centralwidget)
        self.addMotionBttn.setGeometry(QtCore.QRect(20, 640, 89, 31))
        self.addMotionBttn.setObjectName("addMotionBttn")
        self.vectorSpaceSetter = QtWidgets.QPushButton(self.centralwidget)
        self.vectorSpaceSetter.setGeometry(QtCore.QRect(20, 310, 241, 31))
        self.vectorSpaceSetter.setObjectName("vectorSpaceSetter")
        self.topicChecks = QtWidgets.QTextBrowser(self.centralwidget)
        self.topicChecks.setGeometry(QtCore.QRect(20, 590, 241, 31))
        self.topicChecks.setObjectName("topicChecks")
        self.motionNameSelected = QtWidgets.QTextEdit(self.centralwidget)
        self.motionNameSelected.setGeometry(QtCore.QRect(120, 640, 141, 31))
        self.motionNameSelected.setObjectName("motionNameSelected")
        self.motionsBrowser = QtWidgets.QTextBrowser(self.centralwidget)
        self.motionsBrowser.setGeometry(QtCore.QRect(10, 741, 261, 111))
        self.motionsBrowser.setObjectName("motionsBrowser")
        self.deleteMotion = QtWidgets.QToolButton(self.centralwidget)
        self.deleteMotion.setGeometry(QtCore.QRect(20, 690, 241, 31))
        self.deleteMotion.setObjectName("deleteMotion")
        self.motionControl = QtWidgets.QToolButton(self.centralwidget)
        self.motionControl.setGeometry(QtCore.QRect(20, 220, 241, 31))
        self.motionControl.setObjectName("motionControl")
        self.textBrowser.raise_()
        self.textBrowser_4.raise_()
        self.textBrowser_5.raise_()
        self.tabWidget.raise_()
        self.logoView.raise_()
        self.logPoseName.raise_()
        self.logPose.raise_()
        self.motionTypeSet.raise_()
        self.setReference.raise_()
        self.setReferenceTopic.raise_()
        self.editPoses.raise_()
        self.pointSelected1.raise_()
        self.runButton.raise_()
        self.saveAllMotions.raise_()
        self.loadButton.raise_()
        self.runProgress.raise_()
        self.saveName.raise_()
        self.loadName.raise_()
        self.addMotionBttn.raise_()
        self.vectorSpaceSetter.raise_()
        self.topicChecks.raise_()
        self.motionNameSelected.raise_()
        self.motionsBrowser.raise_()
        self.deleteMotion.raise_()
        self.motionControl.raise_()
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1451, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.sensorViewLabel3.setText(_translate("MainWindow", "Sensor View 3"))
        self.sensorViewLabel2.setText(_translate("MainWindow", "Sensor View 2"))
        self.sensorViewLabel1.setText(_translate("MainWindow", "Sensor View 1"))
        self.loadView.setText(_translate("MainWindow", "Load New"))
        self.saveView.setText(_translate("MainWindow", "Save Config"))
        self.newSensorView.setText(_translate("MainWindow", "New Sensor View"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.simTab), _translate("MainWindow", "Sensor Data"))
        self.kpiName.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:24pt;\">Key Performance Indicators</span></p></body></html>"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.graphTab), _translate("MainWindow", "Metric Performance"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.graphViewTab), _translate("MainWindow", "Graph View"))
        self.logPoseName.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">pose_0</p></body></html>"))
        self.logPose.setText(_translate("MainWindow", "Log"))
        self.motionTypeSet.setText(_translate("MainWindow", "Action Type"))
        self.setReference.setText(_translate("MainWindow", "Reference"))
        self.setReferenceTopic.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">None</p></body></html>"))
        self.editPoses.setText(_translate("MainWindow", "Edit Actions/Poses"))
        self.textBrowser_4.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:18pt;\">Key Point Setter</span></p></body></html>"))
        self.textBrowser_5.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:18pt;\">Motion Setters</span></p></body></html>"))
        self.pointSelected1.setText(_translate("MainWindow", "Pose"))
        self.runButton.setText(_translate("MainWindow", "Test"))
        self.saveAllMotions.setText(_translate("MainWindow", "Save"))
        self.loadButton.setText(_translate("MainWindow", "Load"))
        self.saveName.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">action_set1.json</p></body></html>"))
        self.loadName.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">action_set1.json</p></body></html>"))
        self.textBrowser.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:18pt;\">Finalise</span></p></body></html>"))
        self.addMotionBttn.setText(_translate("MainWindow", "Save"))
        self.vectorSpaceSetter.setText(_translate("MainWindow", "Joint"))
        self.topicChecks.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">topic checks</p></body></html>"))
        self.motionNameSelected.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">None</p></body></html>"))
        self.motionsBrowser.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Ubuntu\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" text-decoration: underline;\">Motions</span></p>\n"
"<p align=\"center\" style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>"))
        self.deleteMotion.setText(_translate("MainWindow", "Delete Motion"))
        self.motionControl.setText(_translate("MainWindow", "Motion Control"))
