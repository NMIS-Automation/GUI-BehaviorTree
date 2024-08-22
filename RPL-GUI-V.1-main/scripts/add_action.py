from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import pyqtSignal

class CustomActionWidget(QtWidgets.QDialog):
    actionSaved = pyqtSignal(dict)

    def __init__(self, filepath):
        super().__init__()
        uic.loadUi(filepath + "/ui_files/add_action.ui", self)
        self.action = None
        self.saveBttn.clicked.connect(self.saveButton)

    def saveButton(self):
        subscr = self.controlTopic.toPlainText()
        if subscr == 'subscribe topic':
            subscr = None
        goal = self.goalService.toPlainText()
        if goal == 'goal service':
            goal = None

        if subscr is None and goal is None:
            QtWidgets.QMessageBox.critical(self, "Error", "No custom subscriber or goal set.")
            return

        node = self.nodeName.toPlainText()
        if node == 'node name':
            node = None
        pkg = self.packageName.toPlainText()
        if pkg == 'package name':
            pkg = None
        script = self.scriptName.toPlainText()
        if script == 'script name':
            script = None

        type1 = (node is not None) and (pkg is not None) and (script is not None)

        pkg2 = self.packageName_2.toPlainText()
        if pkg2 == 'package name':
            pkg2 = None
        lnch = self.launchName.toPlainText()
        if lnch == 'launch name':
            lnch = None

        type2 = (lnch is not None) and (pkg2 is not None)

        config = self.configFilename.toPlainText()
        if config == 'config file' and type2:
            QtWidgets.QMessageBox.critical(self, "Error", "No config file attached.")
            return


        if type1 and type2:
            QtWidgets.QMessageBox.critical(self, "Error", "Ambiguous definition of action. Please make it more clear.")
            return
        
        if type1:
            self.action = {'type': 'script', 'pkg': pkg, 'node': node, 'scr': script, 'subscribe': subscr, 'goal': goal}
        else:
            if type2:
                self.action = {'type': 'launch','pkg': pkg, 'launch': lnch, 'subscribe': subscr, 'goal': goal, 'config': config}

        # Emit the actionSaved signal with the action data
        self.actionSaved.emit(self.action)

        # Close the dialog
        self.accept()  # Use accept() to close the dialog and return QDialog.Accepted

