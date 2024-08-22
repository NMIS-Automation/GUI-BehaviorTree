import rospy
from geometry_msgs.msg import Twist
from PyQt5 import QtWidgets, uic
import threading



class CartesianButtonsWidget(QtWidgets.QDialog):
    def __init__(self,filepath):
        super().__init__()
        uic.loadUi(filepath + "/ui_files/cart_control_popup.ui", self)
        self.pose_pub = rospy.Publisher('/diff_kin/cartesian_vel_sim',Twist, queue_size = 10)
        self.command = None
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0
        self.ok = True
        self.thread = threading.Thread(target = self.send_cmmds)
        self.thread.start()
        self.plusX.pressed.connect(self.pX)
        self.plusX.released.connect(self.released_)
        self.plusRX.pressed.connect(self.pRX)
        self.plusRX.released.connect(self.released_)
        self.plusY.pressed.connect(self.pY)
        self.plusY.released.connect(self.released_)
        self.plusRY.pressed.connect(self.pRY)
        self.plusRY.released.connect(self.released_)
        self.plusZ.pressed.connect(self.pZ)
        self.plusZ.released.connect(self.released_)
        self.plusRZ.pressed.connect(self.pRZ)
        self.plusRZ.released.connect(self.released_)

        self.minusX.pressed.connect(self.mX)
        self.minusX.released.connect(self.released_)
        self.minusY.pressed.connect(self.mY)
        self.minusY.released.connect(self.released_)
        self.minusZ.pressed.connect(self.mZ)
        self.minusZ.released.connect(self.released_)
        self.minusRX.pressed.connect(self.mRX)
        self.minusRX.released.connect(self.released_)
        self.minusRY.pressed.connect(self.mRY)
        self.minusRY.released.connect(self.released_)
        self.minusRZ.pressed.connect(self.mRZ)
        self.minusRZ.released.connect(self.released_)

    def released_(self):
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0
        
    def pX(self):
        slider_value = float(self.linSlider.value())/100.0
        vx = 0.1*slider_value
        self.twist_msg = Twist()
        self.twist_msg.linear.x = vx
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0
        
    def mX(self):
        slider_value = float(self.linSlider.value())/100.0
        vx = -0.1*slider_value
        self.twist_msg = Twist()
        self.twist_msg.linear.x = vx
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

    def pRX(self):
        slider_value = float(self.rotSlider.value())/100.0
        vx = 0.1*slider_value
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = vx
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

    def mRX(self):
        slider_value = float(self.rotSlider.value())/100.0
        vx = -0.1*slider_value
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = vx
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

    def pY(self):
        slider_value = float(self.linSlider.value())/100.0
        vy = 0.1*slider_value
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = vy
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

    def mY(self):
        slider_value = float(self.linSlider.value())/100.0
        vy = -0.1*slider_value
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = vy
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

    def pRY(self):
        slider_value = float(self.rotSlider.value())/100.0
        vy = 0.1*slider_value
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = vy
        self.twist_msg.angular.z = 0

    def mRY(self):
        slider_value = float(self.rotSlider.value())/100.0
        vy = -0.1*slider_value
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = vy
        self.twist_msg.angular.z = 0

    def pZ(self):
        slider_value = float(self.linSlider.value())/100.0
        vz = 0.1*slider_value
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = vz
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

    def mZ(self):
        slider_value = float(self.linSlider.value())/100.0
        vz = -0.1*slider_value
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = vz
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

    def pRZ(self):
        slider_value = float(self.rotSlider.value())/100.0
        vz = 0.1*slider_value
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = vz

    def mRZ(self):
        slider_value = float(self.rotSlider.value())/100.0
        vz = -0.1*slider_value
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = vz

    def send_cmmds(self):
        r = rospy.Rate(10)
        while self.ok:
            self.pose_pub.publish(self.twist_msg)            
            r.sleep()

    def closeEvent(self,event):
        self.ok = False
        self.thread.join()
        self.pose_pub.unregister()
        event.accept()

