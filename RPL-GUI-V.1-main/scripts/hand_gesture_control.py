# hand gesture libs:
import rospy
from rpl_msgs.msg import SensorData
import numpy as np
from geometry_msgs.msg import Twist
from add_action import *
import tf
import numpy as np


class HandGestureCommand:
    def __init__(self):
        self.pose0 = None
        self.pose_pub = rospy.Publisher('/diff_kin/cartesian_vel_sim',Twist, queue_size = 10)
        self.twist_msg = Twist()
        self.nn_subcriber = rospy.Subscriber('/gesture/hand_sign', SensorData, self.update_raw)
    
    def rodrigues_vec_to_rotation_mat(self,rodrigues_vec):
        theta = np.linalg.norm(rodrigues_vec)
        if theta == 0:              
            rotation_mat = np.eye(3, dtype=float)
        else:
            r = rodrigues_vec / theta
            I = np.eye(3, dtype=float)
            r_rT = np.array([
                [r[0]*r[0], r[0]*r[1], r[0]*r[2]],
                [r[1]*r[0], r[1]*r[1], r[1]*r[2]],
                [r[2]*r[0], r[2]*r[1], r[2]*r[2]]
            ])
            r_cross = np.array([
                [0, -r[2], r[1]],
                [r[2], 0, -r[0]],
                [-r[1], r[0], 0]
            ])
            rotation_mat = np.cos(theta) * I + (1 - np.cos(theta)) * r_rT + np.sin(theta) * r_cross
        return rotation_mat
    
    def rotation_matrix_to_axis_angle(self,R):
        # Compute the trace of the rotation matrix
        trace = np.trace(R)
        
        # Calculate the angle of rotation
        angle = np.arccos((trace - 1.0) / 2.0)
        
        # Check if the rotation angle is close to zero
        if np.isclose(angle, 0.0):
            # If angle is close to zero, return zero rotation (identity axis)
            return np.array([1.0, 0.0, 0.0]), 0.0
        else:
            # Compute the axis of rotation (unit vector)
            axis = (1.0 / (2.0 * np.sin(angle))) * np.array([
                R[2, 1] - R[1, 2],
                R[0, 2] - R[2, 0],
                R[1, 0] - R[0, 1]
            ])
            
            # Normalize the axis vector to ensure it is a unit vector
            axis = axis / np.linalg.norm(axis)
            
            return axis, angle

    def update_raw(self,msg):
        max_angular = 0.001
        max_linear = 0.01
        if msg.success:
            pose = msg.data.data
            T_new = np.identity(4)
            T_new[:3,:3] = self.rodrigues_vec_to_rotation_mat(pose[3:])
            T_new[:3,3] = np.array(pose[:3])
            if self.pose0 is None:
                self.pose0 = T_new
                self.twist_msg.linear.x = 0
                self.twist_msg.linear.y = 0
                self.twist_msg.linear.z = 0
                self.twist_msg.angular.x = 0
                self.twist_msg.angular.y = 0
                self.twist_msg.angular.z = 0
            else:
                dT = np.matmul(np.linalg.inv(self.pose0), T_new)
                dr,dth = self.rotation_matrix_to_axis_angle(dT[:3,:3])
                lin_fac = np.min([np.linalg.norm(dT[:3,3]),max_linear])
                rot_fac = np.min([np.abs(dth),max_angular])
                dth = rot_fac*dth/np.abs(dth)
                dT[:3,3] = dT[:3,3]*(lin_fac/np.linalg.norm(dT[:3,3]))
                self.twist_msg.linear.x = dT[0,3]
                self.twist_msg.linear.y = dT[1,3]
                self.twist_msg.linear.z = dT[2,3]
                self.twist_msg.angular.x = dr[0]*dth
                self.twist_msg.angular.y = dr[1]*dth
                self.twist_msg.angular.z = dr[2]*dth

            self.pose_pub.publish(self.twist_msg)
            return
        else:
            self.pose0 = None
            self.twist_msg.linear.x = 0
            self.twist_msg.linear.y = 0
            self.twist_msg.linear.z = 0
            self.twist_msg.angular.x = 0
            self.twist_msg.angular.y = 0
            self.twist_msg.angular.z = 0
            self.pose_pub.publish(self.twist_msg)

    def close(self):
        self.nn_subcriber.unregister()
        self.pose_pub.unregister()
        
