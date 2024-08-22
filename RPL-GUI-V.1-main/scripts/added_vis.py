#! /usr/bin/env python3

# visualisation libs
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from stl import mesh
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_about_axis
from rpl_msgs.srv import VisualisationRequest, VisualisationRequestResponse 
import numpy as np

class VisualMarkers:
    def __init__(self):
        rospy.init_node("added_visualisations",anonymous = True)
        self.marker_names = []
        self.marker_poses = []
        self.markerArray = MarkerArray()
        self.curr_count = 0
        self.pub_vals = rospy.Publisher('visual_markers', MarkerArray, queue_size = 1)
        rospy.Service('addvis_server', VisualisationRequest, self.update_markers_cb)

    def update_markers_cb(self,msg):
        print(msg.name)
        response = VisualisationRequestResponse()
        response.success = True
        try:
            label = msg.info
            type = msg.name
            frame_id = msg.frame
            pose = list(msg.data.data)
            point = Point(pose[0,pose[1],pose[2]])
            axis_angle = np.array(pose[3:])
            theta = np.linalg.norm(axis_angle)
            quat = [1,0,0,0]
            if theta>0:
                axis_angle = axis_angle/theta
                quat = quaternion_about_axis((theta,axis_angle[0],axis_angle[1],axis_angle[2]), (1, 0, 0))
            if label in self.marker_names:
                points = self.marker_poses[self.marker_names.index(label)]
                for p in points:
                    self.markerArray.markers[p].pose.position = point
                    self.markerArray.markers[p].orientation = quat
                    self.pub_vals.publish(self.markerArray)
                return response
            if type == 'for':
                self.marker_poses.append([self.curr_count,self.curr_count + 1, self.curr_count + 2, self.curr_count + 3])
                self.vis_coords(point,quat,frame_id)
                self.vis_label(frame_id,point,axis_name=label)
                self.marker_names.append(label)
            if type == 'stl':
                self.marker_poses.append([self.curr_count,self.curr_count + 1])
                filename = msg.filename
                self.vis_stl(filename,point,quat,frame_id)
                self.vis_label(frame_id,point,label)
            self.pub_vals.publish(self.markerArray)
            return response
        except:
            response.success = False
            return response
            
    def vis_stl(self,filename,point,orientation,frame):
        try:
            my_mesh = mesh.Mesh.from_file(filename)
        except:
            # TODO: error message return from request
            return
        marker_msg = Marker()
        marker_msg.header.frame_id = frame
        marker_msg.type = Marker.TRIANGLE_LIST
        marker_msg.action = Marker.ADD
        marker_msg.pose.orientation = orientation
        marker_msg.pose.position = point
        # mm -> m
        marker_msg.scale.x = 0.001
        marker_msg.scale.y = 0.001
        marker_msg.scale.z = 0.001

        marker_msg.color.a = 1.0
        marker_msg.color.r = 0.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 0.0
        for triangle in my_mesh.vectors:
            for vertex in triangle:
                pt = Point()
                pt.x, pt.y, pt.z = vertex
                marker_msg.points.append(pt)

        marker_msg.id = self.curr_count
        self.curr_count+=1
        self.markerArray.markers.append(marker_msg)
        self.markerArray.header.stamp = rospy.Time.now()

    def vis_coords(self,point,orientation,frame_id,scale=0.1):
        axis_names = ['X', 'Y', 'Z']
        for idx, axis_name in enumerate(axis_names):
            axis_marker = Marker()
            axis_marker.header.frame_id = frame_id
            axis_marker.type = Marker.ARROW
            axis_marker.action = Marker.ADD
            axis_marker.pose.position = point
            axis_marker.pose.orientation = orientation
            axis_marker.scale = Vector3(scale, scale / 5.0, scale / 5.0)
            if axis_name == 'X':
                axis_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red (X-axis)
            elif axis_name == 'Y':
                axis_marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Green (Y-axis)
            elif axis_name == 'Z':
                axis_marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # Blue (Z-axis)
            axis_end = Point(scale, 0.0, 0.0) if axis_name == 'X' else \
                    Point(0.0, scale, 0.0) if axis_name == 'Y' else \
                    Point(0.0, 0.0, scale)  # Z-axis
            axis_marker.points.append(point)  # Start at origin
            axis_marker.points.append(axis_end)  # End at axis endpoint
            self.markerArray.markers.append(axis_marker)
            self.curr_count+=1

    def vis_label(self,frame_id,point,axis_name = ''):
        # Create a Marker message for the axis label
        label_marker = Marker()
        label_marker.header.frame_id = frame_id
        label_marker.type = Marker.TEXT_VIEW_FACING
        label_marker.action = Marker.ADD
        label_marker.pose.position = point
        label_marker.pose.orientation.w = 1.0  # Face text toward the camera
        # Set color and text for the axis label
        label_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # White color
        label_marker.text = axis_name  # Text label (e.g., X, Y, Z)
        # Scale and z-offset for the text label
        label_marker.scale.z = self.scale / 2.0  # Text size
        label_marker.pose.position.z += label_marker.scale.z  # Offset above axis
        self.markerArray.markers.append(label_marker)
        self.curr_count+=1


if __name__=='__main__':
    vm = VisualMarkers()
    rospy.spin()

