#! /usr/bin/env python3

from functools import partial
from PyQt5 import QtWidgets, uic
import sys

from PyQt5.QtCore import QSize
from PyQt5.QtGui import QIcon, QPixmap
import rospy
from rqt_gui.main import Main
from rqt_graph.ros_graph import *
import rospkg
from image_viz_widget import *
from plot_widget import *
from multiple_plots import *
from widget_layout import *
from cartesian_control_widget import *
from force_mode_setters import *
import rviz
from saved_pose_selection import *
from sensor_msgs.msg import JointState
from get_topics import *
from hand_gesture_control import *
import tf
from rpl_msgs.srv import PathPlanRequest
from std_msgs.msg import Float64MultiArray


"""
Version 1 of RPL GUI interface

- The UI configuration files are generated with the QT5 Designer interface, that are then resourced as .ui files for the main window and pop-out widgets
- Rviz is embedded as a window. There aren't many ways to skin this cat, this one seems the best and most reliable approach tested so far - 'setupVis'.
- The RQT graph is embedded within a frame. Due to the set-up of the embedding, the source code is un-modifiable and requires 'addWidget' and 'serial_number' to be attributed functions of the parent.
- Pose/Motion setters both rely on the saved_pose_selection.py file, saving points and paths as a dictionary object.
A YAML file-system was attempted, but ran into significant errors when converting from numpy data-types. Have reverted to JSON since it automatically converts single-element data-types to python native types.
- The computer-side motion control approaches (widget, hand, etc) broadcast to the same topics, and are de-registered upon ceasing that type of motion control to prevent clashes. These can be re-registered later on.




Author: Alastair Poole
Email: alastair.poole@strath.ac.uk
"""


class teach_ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(teach_ui, self).__init__()
        rp = rospkg.RosPack()

        self.filepath = rp.get_path('rpl_gui_1')
        self.filepath2 = rp.get_path('single_robot_prototype')
        uic.loadUi(self.filepath+'/ui_files/main_gui.ui', self)
        
        rospy.init_node("rpl_cell_visualiser")
        self.rqt_graph_ = RosGraph(self)
        self.rqt_graph_.initialized = True
        self.setupVis()

        # add a default config here;
        self.widgetCont = widget_handler(view_widgets = ['sensorView1, sensorView2, sensorView3'])
        self.vis_sub = []
        self.vis_topics = []

        # get live topic information;
        # only imageviews for now!
        self.get_topics = topics()
        self.get_topics.refresh()
        self.initButtons()
        self.show()

    def popup_cb(self):
        """Callback for Cartesian Control Buttons"""
        vwidge = CartesianButtonsWidget(self.filepath)
        vwidge.exec_()

    def command_pose_button(self):
        self.poseSetter.setPopupMode(QtWidgets.QToolButton.MenuButtonPopup)
        self.poseSetter.setToolButtonStyle(2)

    def dropdown_setup(self):
        """Control method for the sim/real robot"""
        # NOTE: Change when adding novel control messages
        def on_select(option_name):
            if option_name=='by panel':
                
                return
            
            if option_name=='by hand':

                return
            
            if option_name=='by image':

                return
            # then send topic name to the subscriber

        dropdown_menu = QtWidgets.QMenu(self)
        dropdown_selection = ['by hand', 'by panel', 'by image']
        for option in dropdown_selection:
            action = QtWidgets.QAction(option, self)
            action.triggered.connect(partial(on_select, option))
            dropdown_menu.addAction(action)

    def initButtons(self):
        """Initialise the button set."""
        self.loadView.clicked.connect(self.loadView_cb)
        self.saveView.clicked.connect(self.saveView_cb)

        self.setUpMotionControl()
        self.vectorSpaceSetter.clicked.connect(self.vectorSpaceSetter_)

        self.pose_motion_handler = Poses_and_Motion(self.filepath)
        self.robot_pose = None
        self.joint_pose = None
        self.joint_pose_sub = rospy.Subscriber('/joint_states',JointState,self.updateJointPose)

        self.root_link = 'table_base'
        self.tip_link = '0tool0'
        self.listener = tf.TransformListener()
        self.logPose.clicked.connect(self.recordPose)

        self.sensorRefresh()
        self.sensorDropdown()

        # motion setters initialise;
        self.poseDropdown()
        self.motionDropDown()
        self.addMotionBttn.clicked.connect(self.saveMotionsButton)

        # motion testers & savers;
        self.path_planner = None
        self.runButton.clicked.connect(self.testMotions_cb)
        self.saveAllMotions.clicked.connect(self.saveAllMotions_)

        return

    def setUpMotionControl(self):
        """"""
        self.motionControlMethod = None
        self.motionControl.setPopupMode(QtWidgets.QToolButton.MenuButtonPopup)
        motionSelectMenu = QMenu(self.motionControl)
        controls = ['Widget', 'Hand Guidance', 'Freedrive', 'Joystick']
        for option in controls:
            action = QtWidgets.QAction(option, self)
            action.triggered.connect(lambda checked, opt=option: self.motconptionSelected(opt))
            motionSelectMenu.addAction(action)
        self.motionControl.setMenu(motionSelectMenu)

    def motconptionSelected(self,option):
        """Callback function for the selected motion type"""
        if self.motionControlMethod is not None:
            self.motionControlMethod.close()

        elif option == 'Hand Guidance':
            self.motionControlMethod = HandGestureCommand()

        elif option == 'Widget':
            self.popup_cb()

        elif option == 'Joystick':
            # TODO
            return



    ###############################################################################################
    # Set up visualisation;
    ###############################################################################################
    
    def setupVis(self):
        """
        Set up the RVIZ window for visualising the robot
        """
        self.frame = rviz.bindings.VisualizationFrame()
        self.frame.setSplashPath( "" )
        self.frame.initialize()
        reader = rviz.bindings.YamlConfigReader()
        config = rviz.bindings.Config()
        reader.readFile( config, self.filepath2+ "/rviz/gui1_3.rviz" )
        self.frame.load( config )
        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget( self.frame )
        self.rvizView.setLayout( layout )


    ###############################################################################################
    # Start Streaming Topics
    ###############################################################################################

    def sensorRefresh(self):
        img = QPixmap(self.filepath + '/imgs/refresh.png')
        icon= QIcon(img)
        self.refreshSensors.setIcon(icon)
        self.refreshSensors.setIconSize(QSize(31,31))
        self.refreshSensors.clicked.connect(self.sensorMenuRefresh)

    def sensorDropdown(self):
        self.sensorDisplayWidgetsTags = [['sensorView1', self.sensorViewLabel1],['sensorView2', self.sensorViewLabel2],['sensorView3', self.sensorViewLabel3]]
        self.curr_it = 0

        self.currentSensorsSelected = None
        self.newSensorView.setPopupMode(QtWidgets.QToolButton.MenuButtonPopup)
        self.sensorSelectmenu = QMenu(self.newSensorView)
        self.sensorMenuRefresh()
        self.newSensorView.setMenu(self.sensorSelectmenu)

    def sensorMenuRefresh(self):
        self.sensorSelectmenu.clear()
        self.get_topics.refresh()
        img_topics = self.get_topics.get_image_topics()
        for option in img_topics:
            action = QtWidgets.QAction(option, self)
            action.triggered.connect(lambda checked, opt=option: self.handleSensorOptionDetected(opt))
            self.sensorSelectmenu.addAction(action)

    def handleSensorOptionDetected(self,option):
        self.add_vis_widget(option,self.sensorDisplayWidgetsTags[self.curr_it][0], self.sensorDisplayWidgetsTags[self.curr_it][1])
        self.curr_it = np.mod(self.curr_it + 1,len(self.sensorDisplayWidgetsTags))
        return


    ###############################################################################################
    # Pose Buttons/Functions
    ###############################################################################################

    def recordPose(self):
        """Record the current pose, replacing named duplicates"""
        self.listener.waitForTransform(self.root_link, self.tip_link, rospy.Time(), rospy.Duration(1.0))
        (trans, rot) = self.listener.lookupTransform(self.root_link, self.tip_link, rospy.Time(0))
        # NOTE: sending/saving quaternions
        pose = np.hstack([trans,rot]).flatten()
        pose_name = self.logPoseName.toPlainText()
        if self.vectorSpaceSetter.text() == 'Cartesian':
            self.pose_motion_handler.add_pose(pose_name,pose,'cart',self.root_link)
        else:
            self.pose_motion_handler.add_pose(pose_name,pose,'joint',self.root_link, self.joint_pose)
        # refresh menu for choosing pose coupled to next motion;
        self.menuRefresh()
        
    def vectorSpaceSetter_(self):
        if self.vectorSpaceSetter.text() == 'Cartesian':
            self.vectorSpaceSetter.setText('Joint')
        else:
            if self.vectorSpaceSetter.text() == 'Joint':
                self.vectorSpaceSetter.setText('Cartesian')

    def updateJointPose(self,msg):
        self.joint_pose = np.array(list(msg.position))


    ###############################################################################################
    # Motion Buttons/Functions
    ###############################################################################################

    def poseDropdown(self):
        self.currentPoseSelected = None
        self.pointSelected1.setPopupMode(QtWidgets.QToolButton.MenuButtonPopup)
        self.poseSelectmenu = QMenu(self.pointSelected1)
        self.menuRefresh()
        self.pointSelected1.setMenu(self.poseSelectmenu)

    def menuRefresh(self):
        self.poseSelectmenu.clear()
        poses = ['None']
        for points in list(self.pose_motion_handler.pose_set.keys()):
            poses.append(points)
        for option in poses:
            action = QtWidgets.QAction(option, self)
            action.triggered.connect(lambda checked, opt=option: self.handleOptionSelected(opt))
            self.poseSelectmenu.addAction(action)

    def handleOptionSelected(self,option):
        if option == 'None':
            self.currentPoseSelected = None
        else:
            self.currentPoseSelected = option

    def motionDropDown(self):
        self.currentPoseSelected = None
        self.motionTypeSet.setPopupMode(QtWidgets.QToolButton.MenuButtonPopup)
        motionSelectmenu = QMenu(self.motionTypeSet)
        self.motionSelected = None
        # NOTE: for other robots - e.g. the KUKA IIWA, we can have a purely force based controller. This can be an option in the future when loading in a custom robot, creating a coupled config file.
        standard_motions = ['standard','standard with force','standard with grip', 'standard with release', 'custom']
        for motion in standard_motions:
            action = QtWidgets.QAction(motion, self)
            action.triggered.connect(lambda checked, opt=motion: self.handleMotionSelected(opt))
            motionSelectmenu.addAction(action)
        self.motionTypeSet.setMenu(motionSelectmenu)

    def handleMotionSelected(self,option): 
        """Select a motion from the drop-down menu"""       
        if option == 'custom':
            widget = CustomActionWidget(self.filepath)
            if widget.exec_() == QtWidgets.QDialog.Accepted:
                self.motionSelected =  widget.action
            else:
                rospy.logerr('Setter closed or rejected due to ambiguity.')
        else:
            self.motionSelected = option
      
    def saveMotionsButton(self):
        """Saves motion details"""
        additional = None
        name = self.motionNameSelected.toPlainText()
        command_type = self.motionSelected
        pose_name = self.currentPoseSelected
        topic_check = self.topicChecks.toPlainText()
        # box unedited;
        if topic_check == 'topic checks':
            topic_check = ''
        topic_check = topic_check.split(',')
        if isinstance(command_type,dict):
            self.additional = command_type
            command_type = 'custom'
            
        if 'force' in command_type:
            # set the force/torque params;
            fmw = ForceMotionWidget(self.filepath)
            if fmw.exec_() == QtWidgets.QDialog.Accepted:
                additional =  fmw.action
            else:
                rospy.logerr('Force setter closed or rejected - rejecting command.')
                return


        if name not in list(self.pose_motion_handler.motion_sets.keys()):
            self.motionsBrowser.setText(self.motionsBrowser.toPlainText() + '   -' + name +'\n')

        self.pose_motion_handler.add_motion(name, command_type, pose_name, topic_check, additional)
        
        # if custom, we probably want to reset the motion types. Otherwise, we can always engage a series of lin motions.
        if command_type == 'custom':
            self.motionSelected = None
            self.additional = None


    ###############################################################################################
    # Save/test motions
    ###############################################################################################


    def testMotions_cb(self):
        """This sends command sets to the path planner for testing"""
        motions = self.pose_motion_handler.motion_sets
        service_proxy = rospy.ServiceProxy('/path_plan_req', PathPlanRequest)
        cp = None
        for key in list(motions.keys()):
            element = motions[key]
            
            if element['cmmd'] == 'cart':
                if cp is None:
                    cp = element['pose_info']['Cpose']
                else:
                    cp = np.vstack([cp,element['pose_info']['Cpose']])

            elif element['cmmd']=='joint':
                d = Float64MultiArray()
                d.data = element['pose_info']['Jpose']
                service_proxy('joint_path',d)

        msg = PathPlanRequest()
        msg.command = 'cartesian_path'
        d = Float64MultiArray()
        d.data = cp.flatten()
        service_proxy('cartesian_path',d)

    def update_progress(self,value):
        self.runProgress.setValue(int(100*value))

    def saveAllMotions_(self):
        file_suffix = self.saveName.toPlainText()
        self.pose_motion_handler.saveMotionSet(file_suffix)


    ###############################################################################################
    # ROS Graph
    ###############################################################################################

    def add_widget(self, widget):
        """
        Adds ROS-graph widget to the UI.
        """
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(widget)
        self.graphView.setLayout(layout)

    def serial_number(self):
        return 0


    ###############################################################################################
    # Topic Display Widgets;
    ###############################################################################################

    def add_vis_widget(self, name, element_name,label_, data_type = 'img', n_var = None):
        """ adds a visualisation widget: get the widget based on its name, then inserts the different viewing widgets"""
        if isinstance(element_name,list):
            try:
                elements = [self.findChildren(QtWidgets.QWidget, el_n)[0] for el_n in element_name]
            except:
                return False
        else:
            try:
                element = self.findChildren(QtWidgets.QWidget, element_name)[0]
            except:
                return False
        if name not in self.vis_topics:
            self.widgetCont.add_topic(name, element_name, data_type,label_, n_var)
            if data_type=='img':
                self.vis_sub.append(ImageViewer(name,element,self))
            if data_type=='plt':
                self.vis_sub.append(MplCanvas(name=name,parent = self))
                if data_type=='pltm':
                    for i in range(n_var):
                        canvas_cont = MultipleCanvas(name,n_var,self)
                        widget_list = canvas_cont.widget_list
                        for widget in widget_list:
                            self.vis_sub.append(widget)
                            layout = QtWidgets.QVBoxLayout(elements[i])
                            layout.addWidget(self.vis_sub[-1])
                            elements[i].setLayout(layout)
                    return
            # Setting the title of the window;
            if label_ is not None:
                label_.setText(name)
            layout = QtWidgets.QVBoxLayout(element)
            layout.addWidget(self.vis_sub[-1])
            element.setLayout(layout)

    def loadView_cb(self):
        filename = self.viewNameLoad.toPlainText()
        if len(filename)==0:
            filename = 'default_config'
        self.widgetCont.load_topics(filename)
        for element_name in list(self.widgtCont.topic_layout.keys()):
            topic = self.widgtCont.topic_layout[element_name]['name']
            label = self.widgtCont.topic_layout[element_name]['label']
            type_ = self.widgtCont.topic_layout[element_name]['type']
            n_var = self.widgtCont.topic_layout[element_name]['nvar']
            self.add_vis_widget(topic,element_name,label,type_,n_var)

    def saveView_cb(self):
        filename = self.viewNameSave.toPlainText()
        if len(filename)==0:
            filename = 'default_config'
        self.widgetCont.save_topics()



if __name__=='__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = teach_ui()
    app.exec_()

