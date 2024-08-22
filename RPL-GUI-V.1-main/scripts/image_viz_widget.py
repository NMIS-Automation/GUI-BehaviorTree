import sys
from PyQt5 import QtWidgets, QtGui, QtCore
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import cv2

class ImageViewer(QtWidgets.QWidget):
    def __init__(self, name, element, parent=None):
        super().__init__(parent)

        # Create a QLabel to display the image
        self.image_label = QtWidgets.QLabel(self)
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)

        # Create a layout for the widget
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.image_label)
        element.setLayout(self.layout)

        self.bridge = CvBridge()

        # Subscribe to ROS Image topic
        self.image_sub = rospy.Subscriber(name, Image, self.image_callback)

    def image_callback(self, ros_image):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")

            # Convert OpenCV image to QPixmap
            q_image = self.convert_cv_image_to_qt(cv_image)

            # Display the image in the QLabel
            self.update_image(q_image)
        except Exception as e:
            print(e)

    def convert_cv_image_to_qt(self, cv_image):
        # Convert OpenCV image (BGR format) to QImage
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QtGui.QImage(cv_image.data, width, height, bytes_per_line, QtGui.QImage.Format_RGB888)
        q_image = q_image.rgbSwapped()  # Convert BGR to RGB

        return q_image

    def update_image(self, q_image):
        # Update the image displayed in the QLabel
        pixmap = QtGui.QPixmap.fromImage(q_image)
        self.image_label.setPixmap(pixmap.scaled(self.image_label.size(), QtCore.Qt.KeepAspectRatio))


