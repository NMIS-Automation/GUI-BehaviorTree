import rospy
from rpl_msgs.msg import SensorData
from rpl_msgs.srv import VisualisationRequest
from std_msgs.msg import Float64MultiArray
from PyQt5.QtWidgets import QMenu
from json_class import *
from addVis import *

from rpl_msgs.srv import PathPlanRequest

"""
Handles motion add/visualise and save/planning requests
Author: Alastair Poole 
Email: alastair.poole@strath.ac.uk
"""

class Poses_and_Motion:
    def __init__(self, filename):
        self.filename = filename
        self.pose_set = {}
        self.motion_sets = {}
        self.filehandler = json_()
        self.add_visualisation = VisualMarkers()
        self.motions_tested = False

    def add_pose(self,pose_name,pose_value,pose_type,frame_id, joint_pose = None):
        """sets the end-point for the motion"""
        self.pose_set[pose_name] = {'name':pose_name, 'Cpose':pose_value, 'type': pose_type, 'frame': frame_id, 'Jpose': joint_pose}
        data = Float64MultiArray()
        data.data = list(pose_value)
        self.add_visualisation.new_for(pose_value,frame_id,pose_name)

    def add_motion(self, name, command_type, pose_name, topic_check = None, additional = None):
        """sets the motion"""
        if pose_name is None and command_type == 'custom':
            new_motion = {'pose_info': 'None', 'cmmd':command_type, 'checks':topic_check, 'custom':additional}
            self.motion_sets[name] = new_motion
            return
        if command_type=='cart':
            # need to re-plan the motion
            self.motions_tested = False

        if pose_name not in list(self.pose_set.keys()):
            return
        pose_all = self.pose_set[pose_name]
        if command_type != 'custom':
            # so we know if it's linear in joint or cartesian space, and if we've asked to grip or move with force;
            command_type = pose_all['type']
        new_motion = {'pose_info': pose_all, 'cmmd':command_type, 'checks':topic_check, 'custom':additional}
        self.motion_sets[name] = new_motion

    def save_motions(self,filename):
        """save motions to config folder"""
        file = self.filename + '/config/' + filename
        self.filehandler.save(file,self.motion_sets)

    def load_motions(self,filename):
        """loads motions from the config folder"""
        file = self.filename + '/config/' + filename
        try:
            new_motions = self.filehandler(file)
        except:
            return
        for motion_name in list(new_motions.keys()):
            motion = new_motions[motion_name]
            if motion['pose_info']['name'] not in self.available_poses:
                self.add_pose(motion['pose_info']['name'],motion['pose_info']['pose'],motion['pose_info']['type'],motion['pose_info']['frame'])
                self.add_motion(motion_name,motion['cmmd'],motion['pose_info']['name'],motion['checks'],motion['custom'])

    def variable_selected(self, variable):
        val = self.pose_values[self.available_poses.index(variable)]
        type = self.pose_types[self.available_poses.index(variable)]
        return variable, val, type

    def saveMotionSet(self,file_suffix):
        fn = self.filename + '/configs/' + file_suffix
        if not self.filehandler.save(fn,self.motion_sets):
            rospy.logerr('Could not save file. Check the motion types and organisation of the folder in order to correctly run.')
        
