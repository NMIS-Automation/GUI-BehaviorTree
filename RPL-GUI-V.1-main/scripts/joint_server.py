#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState
from rpl_msgs.msg import SensorData
from urdf_parser_py.urdf import URDF

"""
Functions as either a fake server for a simulated robot, or converts between the reported robotic joint position and re-broadcasts it as a joint state.
Necessary for robot parameter server to broadcast the state and transformations between links.
Author: Alastair Poole
Email: alastair.poole@strath.ac.uk
"""


class joint_server:
    def __init__(self):
        rospy.init_node('joint_parameter_server_custom', anonymous = True)
        self.sim = rospy.get_param('~sim', default = True)
        if self.sim:
            self.joint_vel_sub = rospy.Subscriber('/diff_kin/joint_vel_ur_sim', SensorData, self.joint_vel_cb)
            self.joint_pose_sub = rospy.Subscriber('joint_pose_ur_sim', SensorData, self.joint_pose_cb)
        else:
            self.joint_pose_sub = rospy.Subscriber('joint_pose_ur', SensorData, self.joint_pose_cb)
        # NOTE: Refresh rate of the server - can change. 10Hz seemed suitable for experimentation.        
        self.dt = 1/10
        self.setup_joint_param_server()

    def joint_pose_cb(self,msg):
        self.joint_msg.position = list(msg.data.data)

    def joint_vel_cb(self,msg):
        self.vel = list(msg.data.data)

    def setup_joint_param_server(self):
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.joint_msg = JointState()
        self.joint_msg.header.stamp = rospy.Time.now()
        self.joint_msg.name = []
        self.joint_msg.position = []
        # NOTE: for custom robot parameter names, can insert the name into the from_parameter_server(name) function
        self.robot = URDF.from_parameter_server()
        self.dof = 0
        self.vel = []
        self.root_name = None
        self.tip_name = None
        for joint in self.robot.joints:
            if joint.type != 'fixed':
                self.joint_msg.name.append(joint.name)
                self.joint_msg.position.append(-0.75)
                self.vel.append(0)
                self.dof+=1
        self.root_name = self.joint_msg.name[0]
        
        self.jointServer()

    def jointServer(self):
        r = rospy.Rate(1/self.dt)
        while not rospy.is_shutdown():
            for i in range(self.dof):
                self.joint_msg.position[i] += self.dt*self.vel[i]
            self.joint_msg.header.stamp = rospy.Time.now()
            self.joint_state_pub.publish(self.joint_msg)
            r.sleep()


if __name__=='__main__':
    js = joint_server()
    rospy.spin()

