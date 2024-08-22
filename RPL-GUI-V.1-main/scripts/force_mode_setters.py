from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import pyqtSignal

class ForceMotionWidget(QtWidgets.QDialog):
    actionSaved = pyqtSignal(dict)

    def __init__(self, filepath):
        super().__init__()
        uic.loadUi(filepath + "/ui_files/force_motions_settings.ui", self)
        self.fmsel = None
        self.setButton.clicked.connect(self.setParams)
        self.utButton.clicked.connect(self.setUTNorm)
        
    def setParams(self):
        # set to 50N max for this;
        force = [float(self.xfS.value())*50/100,float(self.yfS.value())*50/100,float(self.zfS.value())*50/100,float(self.xtS.value())*5/100,float(self.ytS.value())*5/100,float(self.ztS.value())*5/100]
        impedance = [float(self.xfS_1.value())*50/100,float(self.yfS_1.value())*50/100,float(self.zfS_1.value())*50/100,float(self.xtS_1.value())*5/100,float(self.ytS_1.value())*5/100,float(self.ztS_1.value())*5/100]
        deviation = [float(self.xfS_2.value())/100,float(self.yfS_2.value())/100,float(self.zfS_2.value())/100,float(self.xtS_2.value())*2/100,float(self.ytS_2.value())*2/100,float(self.ztS_2.value())*2/100]
        self.fmsel = {'force': force, 'impedance': impedance, 'deviation': deviation}
        self.actionSaved.emit(self.fmsel)
        self.accept()
        
    def setUTNorm(self):
        # standard params for a UT inspection - set to 50N contact force
        self.fmsel = {'force': [0,0,50,0,0,0], 'impedance': [0,0,1,0,0,0], 'deviation': [0,0,1,0,0,0]}
        self.actionSaved.emit(self.fmsel)
        self.accept()




