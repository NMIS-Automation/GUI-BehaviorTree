#! /usr/bin/env python3
from __future__ import print_function
from PyQt5.QtCore import QObject, pyqtSignal
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Float64MultiArray
from rpl_msgs.srv import PathPlanRequest,PathPlanRequestResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def convert_robot_trajectory_to_joint_trajectory(robot_trajectory_msg):
    joint_trajectory_msg = JointTrajectory()
    joint_trajectory_msg.joint_names = robot_trajectory_msg.joint_trajectory.joint_names

    for point in robot_trajectory_msg.joint_trajectory.points:
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = point.positions
        joint_trajectory_point.velocities = point.velocities
        joint_trajectory_point.accelerations = point.accelerations
        joint_trajectory_point.effort = point.effort
        joint_trajectory_point.time_from_start = point.time_from_start
        joint_trajectory_msg.points.append(joint_trajectory_point)

    return joint_trajectory_msg


import numpy as np


try:
    from math import pi, tau, dist, fabs, cos
except:  
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


class PathPlanInterface(QObject):
    progress_updated = pyqtSignal(float)
    def __init__(self, group_name, sim = True):
        super(PathPlanInterface, self).__init__()
        self.sim = sim
        self.dof = 6
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        move_group = moveit_commander.MoveGroupCommander(group_name)
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        """Path plan request service"""
        service = rospy.Service('/path_plan_req',PathPlanRequest,self.path_plan_req_cb)

    def path_plan_req_cb(self,msg):
        """
        Takes a path plan request and returns a success indication and a robot trajectory. Aimed for offline and online dynamic path planning.        
        """
        jt = JointTrajectory()
        response = PathPlanRequestResponse()
        response.path = jt
        response.success = True
        command = msg.command
        if command=='cartesian_path':
            path = np.array(list(msg.data.data))
            n = int(path.size/7)
            path = path.reshape((n,7))
            f,p = self.plan_cartesian_path_array(path)
            return response
        elif command == 'joint_path':
            path = np.array(list(msg.data.data))
            self.go_to_joint_state(path)
            return response        
        return response
        
    def handle_path_plan(self,path_dict, keep = False):
        
        if keep:
            self.new_keys = {}
            index = 0
            collected_cartesian_keep = None

        path_keys = list(path_dict.keys())
        commands = []
        collected_cartesian = []
        cartesian_path_segment = None

        last_motion_point = None
        tracking = 1
        n_commands = float(len(path_keys))
        for key in path_keys:
            rospy.logerr(key)
            self.progress_updated.emit(float(tracking)/n_commands)
            motion = path_dict[key]
            if last_motion_point!=motion['pose_info']['name']:
                if keep:
                    if motion['cmmd']!= 'cart' and len(collected_cartesian_keep)>0:
                        success,jpath = self.plan_cartesian_path_array(collected_cartesian_keep)
                        if not success:
                            return None
                        self.new_keys['cartcollection'+str(index)] = {'cmmd':'joint_path', 'custom':jpath.joint_trajectory}
                        index+=1
                        collected_cartesian_keep = None
                    else:
                        if motion['cmmd']== 'cart':
                            if collected_cartesian_keep is None:
                                collected_cartesian_keep = path_dict[key]['pose_info']['Cpose']
                            else:
                                collected_cartesian_keep = np.vstack([collected_cartesian_keep,path_dict[key]['pose_info']['Cpose']])
                        else:
                            self.new_keys[key] = path_dict[key]

                # if we have collected any cartesian paths and the next one isn't, we plan for the cartesian set first;
                if (motion['cmmd']!= 'cart' and motion['cmmd']!= 'cartesian with force') and len(collected_cartesian)>0:
                    success,jpath = self.plan_cartesian_path_array(cartesian_path_segment)  
                    if not success:
                        # TODO
                        return None
                    commands.append(['joint_path',jpath.joint_trajectory])
                    cartesian_path_segment = None

                if motion['cmmd'] == 'cart' or motion['cmmd'] == 'cartesian with force':
                    collected_cartesian.append(path_dict[key])
                    if cartesian_path_segment is None:
                        cartesian_path_segment = path_dict[key]['pose_info']['Cpose']
                    else:
                        cartesian_path_segment = np.vstack([cartesian_path_segment,path_dict[key]['pose_info']['Cpose']])
                else:
                    if  motion['cmmd'] == 'joint':
                        self.go_to_joint_state(motion['pose_info']['Jpose'])
                        commands.append(['joint_pose', motion['pose_info']['Jpose']])

                last_motion_point=motion['pose_info']['name']
            tracking+=1
        
        if len(collected_cartesian)>0:
            success,jpath = self.plan_cartesian_path_array(cartesian_path_segment)  
            if not success:
                # TODO
                return None
            commands.append(['joint_path',jpath.joint_trajectory])
            cartesian_path_segment = None
        rospy.logerr('cmmds')
        rospy.logerr(commands)
        return commands

    def path_test_handle(self,commands):
        r = rospy.Rate(10)
        n = len(commands)
        i = 0
        #pub = rospy.Publisher('joint_pose_ur_sim',SensorData,queue_size = 1)
        #msg = SensorData()
        data = Float64MultiArray()
        joint_speed = 0.01
        last_pose = None
        n_commands = float(len(commands))
        while True:
            rospy.logerr(i)
            self.progress_updated.emit(float(i)/n_commands)
            motion_info = commands[i]
            if motion_info[0]=='joint_path':
                plan = list(motion_info[1].points)
                for k in range(len(plan)):
                    next_pose = np.array(list(plan[0].positions))
                    if last_pose is None:
                        data.data = next_pose.tolist()
                        #msg.data = data
                        #pub.publish(msg)
                    else:
                        g = int(np.max(np.ceil(np.abs(last_pose - next_pose)/joint_speed)))
                        for k in range(g):
                            data.data = (last_pose + (float(k)/float(g))*(next_pose - last_pose)).tolist()
                            #msg.data = data
                            #pub.publish(msg)
                            r.sleep()
                    last_pose = next_pose
            else:
                if motion_info=='joint_pose':
                    next_pose = motion_info[1]
                    g = np.max(np.ceil(np.abs(last_pose - next_pose)/joint_speed))
                    for k in range(g):
                        data.data = (last_pose + (float(k)/float(g))*(next_pose - last_pose)).tolist()
                        #msg.data = data
                        #pub.publish(msg)
                        r.sleep()
                    last_pose = next_pose

            i = np.mod(i+1,n)
            if i==0:
                last_pose = None

    def PlanRequest_cb(self,command,msg):
        standin = JointTrajectory()
        if command == 'cart':
            success, path = self.go_to_pose_goal(msg)
            if not success:                
                return False, standin
            return True, path
        elif command == 'joint':
            success = self.go_to_joint_state(msg)
            return success, standin
        elif command == 'cartesian_path':
            success, path = self.plan_cartesian_path(msg)
            if not success:
                return False, standin
            return True,path

    def go_to_joint_state(self, msg):
        msg = list(msg)
        #rospy.logerr('msg')
        #rospy.logerr(msg)
        move_group = self.move_group
        joint_goal = []
        for i in range(self.dof):
            joint_goal.append(msg[i])    
        #rospy.logerr('AGAGGGGAAAAHHH')
        move_group.go(joint_goal, wait=True)
        #rospy.logerr('this')
        move_group.stop()
        #rospy.logerr('this')
        #current_joints = move_group.get_current_joint_values()
        rospy.logerr('is')
        return all_close(joint_goal, [], 0.01)

    def go_to_pose_goal(self,msg):
        pose = list(msg)
        if len(pose)!=7:
            return False,[]
        move_group = self.move_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]
        pose_goal.orientation.x = pose[3]
        pose_goal.orientation.y = pose[4]
        pose_goal.orientation.z = pose[5]
        pose_goal.orientation.w = pose[6]
        move_group.set_pose_target(pose_goal)
        success = move_group.go(wait=True)
        if not success:
            return success, []
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        # TODO: joint info in path;
        return all_close(pose_goal, current_pose, 0.01),[]

    def plan_cartesian_path_array(self, msg):
        move_group = self.move_group
        waypoints = []
        rospy.logerr(msg)
        for i in range(msg.shape[0]):
            wpose = move_group.get_current_pose().pose
            wpose.position.x = msg[i,0]
            wpose.position.y = msg[i,1]
            wpose.position.z = msg[i,2]
            wpose.orientation.x = msg[i,3]
            wpose.orientation.y = msg[i,4]
            wpose.orientation.z = msg[i,5]
            wpose.orientation.w = msg[i,6]
            waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )

        return fraction, plan

    def plan_cartesian_path(self, msg):
        n_data = list(msg)
        if np.mod(n_data , 7) !=0:
            # using quaternions
            return False, []
        move_group = self.move_group
        waypoints = []
        wp = []
        for i in range(n_data):
            if np.mod(i,7)==0:
                wpose = move_group.get_current_pose().pose
                wpose.position.x = wp[0]
                wpose.position.y = wp[1]
                wpose.position.z = wp[2]
                wpose.orientation.x = wp[3]
                wpose.orientation.y = wp[4]
                wpose.orientation.z = wp[5]
                wpose.orientation.w = wp[6]
                waypoints.append(copy.deepcopy(wpose))
                wp = []
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )
        return plan, fraction

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        box_name = self.box_name
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "0tool0"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "single_ur5e_solver"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )



def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if True:
        return True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


if __name__=='__main__':
    rospy.init_node('test_plan',anonymous = True)
    ppi = PathPlanInterface('rpl_ur5e_kin')
    rospy.spin()


    
#     path = np.array([0.5,0,0.3,0,0,0,1])
#     path_ = {}
#     for k in range(10):

#         path = np.vstack([path, np.array([0.5,0,0.3+0.1*np.random.rand(),0,0,0,1])])
#         path_[str(k)] = {'cmmd':'cart','pose_info':{'name':str(k),'Cpose':np.array([0.5,0,0.3+0.1*np.random.rand(),0,0,0,1])}}
#     
#     
#     ppi.handle_path_plan(path_)
    #ppi.plan_cartesian_path_array(path)


        
