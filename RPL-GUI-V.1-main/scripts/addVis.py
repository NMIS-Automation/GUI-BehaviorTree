
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from stl import mesh
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_about_axis
from rpl_msgs.srv import VisualisationRequest, VisualisationRequestResponse 
import numpy as np


"""
Add visualisations of target points and meshes to the cell set-up.
Author: Alastair Poole
Email: alastair.poole@strath.ac.uk
"""


def quaternion_to_axis_angle(quaternion):
    # Extract quaternion components
    x, y, z, w = quaternion

    # Calculate the quaternion's magnitude (norm)
    magnitude = np.linalg.norm(quaternion)

    # Normalize the quaternion (unit quaternion)
    x /= magnitude
    y /= magnitude
    z /= magnitude
    w /= magnitude

    # Calculate angle of rotation (in radians)
    angle = 2.0 * np.arccos(w)

    # Calculate axis of rotation
    if angle == 0:
        axis = [1.0, 0.0, 0.0]  # Default axis (no rotation)
    else:
        axis = [x / np.sin(angle/2), y / np.sin(angle/2), z / np.sin(angle/2)]

    # Return axis-angle representation (axis, angle)
    return np.array(axis) * angle


def axis_angle_to_rotation_matrix(axis):
    """
    Convert axis-angle representation to a rotation matrix.

    Args:
        axis (list or np.array): Rotation axis as a 3D unit vector [x, y, z].
        angle (float): Angle of rotation in radians.

    Returns:
        np.array: 3x3 rotation matrix representing the rotation.
    """
    # Normalize the rotation axis (ensure it's a unit vector)
    axis = quaternion_to_axis_angle(axis)
    axis = np.array(axis)
    angle = np.linalg.norm(axis)
    axis /= angle

    # Components of the rotation axis
    x, y, z = axis

    # Compute the components of the rotation matrix using Rodrigues' formula
    c = np.cos(angle)
    s = np.sin(angle)
    t = 1 - c

    # Elements of the rotation matrix
    R = np.array([[t*x*x + c,    t*x*y - z*s,  t*x*z + y*s],
                  [t*x*y + z*s,  t*y*y + c,    t*y*z - x*s],
                  [t*x*z - y*s,  t*y*z + x*s,  t*z*z + c]])

    return R

class VisualMarkers:
    def __init__(self):
        self.marker_names = []
        self.marker_poses = []
        self.markerArray = MarkerArray()
        self.curr_count = 0
        self.pub_vals = rospy.Publisher('visual_markers', MarkerArray, queue_size = 1)
        self.id = 0

    def new_for(self,pose,frame_id,label):
        """
        New Frame of Reference for taught pose being generated.
        """

        sind = None
        if label in self.marker_names:
            idx = self.marker_names.index(label)
            sind = self.marker_poses[idx]
        else:
            self.marker_poses.append(self.id)
            

        R = axis_angle_to_rotation_matrix(pose[3:])        
        # Create a Marker message for the X axis (red)
        x_axis_marker = Marker()
        x_axis_marker.header.frame_id = frame_id
        x_axis_marker.header.stamp = rospy.Time.now()
        x_axis_marker.ns = "frame"
        if sind is not None:
            x_axis_marker.id = sind
        else:
            x_axis_marker.id = self.id
            self.id+=1
        
        x_axis_marker.type = Marker.ARROW
        x_axis_marker.action = Marker.ADD
        x_axis_marker.pose.orientation.w = 1.0
        x_axis_marker.scale = Vector3(0.002, 0.002, 0.002)  # Arrow dimensions (length, diameter, diameter)
        x_axis_marker.color.r = 1.0
        x_axis_marker.color.a = 1.0
        x_axis_marker.points = [Point(pose[0], pose[1], pose[2]), Point(pose[0] + 0.1*R[0,0], pose[1] + 0.1*R[1,0], pose[2] + 0.1*R[2,0])]  # Line endpoints (start, end)

        # Create a Marker message for the Y axis (green)
        y_axis_marker = Marker()
        y_axis_marker.header.frame_id = frame_id
        y_axis_marker.header.stamp = rospy.Time.now()
        y_axis_marker.ns = "frame"
        if sind is not None:
            y_axis_marker.id = sind + 1
        else:
            y_axis_marker.id = self.id
            self.id+=1
        y_axis_marker.type = Marker.ARROW
        y_axis_marker.action = Marker.ADD
        y_axis_marker.pose.orientation.w = 1.0
        y_axis_marker.scale = Vector3(0.002, 0.002, 0.002)  # Arrow dimensions (length, diameter, diameter)
        y_axis_marker.color.g = 1.0
        y_axis_marker.color.a = 1.0
        y_axis_marker.points = [Point(pose[0], pose[1], pose[2]), Point(pose[0] + 0.1*R[0,1], pose[1] + 0.1*R[1,1], pose[2] + 0.1*R[2,1])]  # Line endpoints (start, end)

        # Create a Marker message for the Z axis (blue)
        z_axis_marker = Marker()
        z_axis_marker.header.frame_id = frame_id
        z_axis_marker.header.stamp = rospy.Time.now()
        z_axis_marker.ns = "frame"
        if sind is not None:
            z_axis_marker.id = sind + 2
        else:
            z_axis_marker.id = self.id
            self.id+=1
        z_axis_marker.type = Marker.ARROW
        z_axis_marker.action = Marker.ADD
        z_axis_marker.pose.orientation.w = 1.0
        z_axis_marker.scale = Vector3(0.002, 0.002, 0.002)  # Arrow dimensions (length, diameter, diameter)
        z_axis_marker.color.b = 1.0
        z_axis_marker.color.a = 1.0
        z_axis_marker.points = [Point(pose[0], pose[1], pose[2]), Point(pose[0] + 0.1*R[0,2], pose[1] + 0.1*R[1,2], pose[2] + 0.1*R[2,2])] # Line endpoints (start, end)

        # Publish the Marker messages
        if label in self.marker_names:
            self.markerArray.markers[sind] = x_axis_marker
            self.markerArray.markers[sind+1] = y_axis_marker
            self.markerArray.markers[sind+2] = z_axis_marker
        else:
            self.markerArray.markers.append(x_axis_marker)
            self.markerArray.markers.append(y_axis_marker)
            self.markerArray.markers.append(z_axis_marker)

        # label
        label_marker = Marker()
        label_marker.header.frame_id = frame_id
        if sind is None:
            label_marker.id = self.id
            self.id+=1
        else:
            label_marker.id = sind + 3

        label_marker.type = Marker.TEXT_VIEW_FACING
        label_marker.action = Marker.ADD
        #label_marker.pose.position = point
        label_marker.pose.orientation.w = 1.0
        label_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        label_marker.text = label
        label_marker.scale.z = 0.1 / 2.0
        label_marker.pose.position.x = pose[0]
        label_marker.pose.position.y =  pose[1]
        label_marker.pose.position.z = pose[2]#+= 0.1 / 2.0
        if label in self.marker_names:
            self.markerArray.markers[sind+3] = label_marker
        else:
            self.markerArray.markers.append(label_marker)

        if label not in self.marker_names:
            self.marker_names.append(label)
        self.pub_vals.publish(self.markerArray)

    def cont_pub(self):        
        rate = rospy.Rate(1)        
        while not rospy.is_shutdown():
            self.pub_vals.publish(self.markerArray)
            rate.sleep()            

    def publish_marker_array(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            marker_array_msg = MarkerArray()
            # Create a Marker message for the X axis (red)
            x_axis_marker = Marker()
            x_axis_marker.header.frame_id = "table_base"
            x_axis_marker.header.stamp = rospy.Time.now()
            x_axis_marker.ns = "frame"
            x_axis_marker.id = 0
            x_axis_marker.type = Marker.ARROW
            x_axis_marker.action = Marker.ADD
            x_axis_marker.pose.orientation.w = 1.0
            x_axis_marker.scale = Vector3(0.002, 0.002, 0.002)  # Arrow dimensions (length, diameter, diameter)
            x_axis_marker.color.r = 1.0
            x_axis_marker.color.a = 1.0
            x_axis_marker.points = [Point(0, 0, 0), Point(0.1, 0, 0)]  # Line endpoints (start, end)

            # Create a Marker message for the Y axis (green)
            y_axis_marker = Marker()
            y_axis_marker.header.frame_id = "table_base"
            y_axis_marker.header.stamp = rospy.Time.now()
            y_axis_marker.ns = "frame"
            y_axis_marker.id = 1
            y_axis_marker.type = Marker.ARROW
            y_axis_marker.action = Marker.ADD
            y_axis_marker.pose.orientation.w = 1.0
            y_axis_marker.scale = Vector3(0.002, 0.002, 0.002)  # Arrow dimensions (length, diameter, diameter)
            y_axis_marker.color.g = 1.0
            y_axis_marker.color.a = 1.0
            y_axis_marker.points = [Point(0, 0, 0), Point(0, 0.1, 0)]  # Line endpoints (start, end)

            # Create a Marker message for the Z axis (blue)
            z_axis_marker = Marker()
            z_axis_marker.header.frame_id = "table_base"
            z_axis_marker.header.stamp = rospy.Time.now()
            z_axis_marker.ns = "frame"
            z_axis_marker.id = 2
            z_axis_marker.type = Marker.ARROW
            z_axis_marker.action = Marker.ADD
            z_axis_marker.pose.orientation.w = 1.0
            z_axis_marker.scale = Vector3(0.002, 0.002, 0.002)  # Arrow dimensions (length, diameter, diameter)
            z_axis_marker.color.b = 1.0
            z_axis_marker.color.a = 1.0
            z_axis_marker.points = [Point(0, 0, 0), Point(0, 0, 0.1)]  # Line endpoints (start, end)

            # Publish the Marker messages
            marker_array_msg.markers.append(x_axis_marker)
            marker_array_msg.markers.append(y_axis_marker)
            marker_array_msg.markers.append(z_axis_marker)

            

            # label
            label_marker = Marker()
            label_marker.header.frame_id = "table_base"
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            #label_marker.pose.position = point
            label_marker.pose.orientation.w = 1.0
            label_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            label_marker.text = "this"
            label_marker.scale.z = 0.1 / 2.0
            label_marker.pose.position.z += 0.1 / 2.0
            marker_array_msg.markers.append(label_marker)

            self.pub_vals.publish(marker_array_msg)

            rate.sleep()

