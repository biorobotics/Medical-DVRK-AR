#!/usr/bin/env python
from __future__ import division, print_function
import rospy
import rospkg
import os.path
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
import json
import vtk
from tf_conversions import posemath
import PyKDL
import numpy as np
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, Float64
import math
import copy

def clean_resource_path(path):
    new_path = path
    if path.find("package://") == 0:
        new_path = new_path[len("package://"):]
        pos = new_path.find("/")
        if pos == -1:
            rospy.logfatal("%s Could not parse package:// format", path)
            quit(1)

        package = new_path[0:pos]
        new_path = new_path[pos:]
        package_path = rospkg.RosPack().get_path(package)

        if package_path == "":
            rospy.logfatal("%s Package [%s] does not exist",
                           path.c_str(),
                           package.c_str())
            quit(1)

        new_path = package_path + new_path
    elif path.find("file://") == 0:
        new_path = new_path[len("file://"):]

    if not os.path.isfile(new_path):
        rospy.logfatal("%s file does not exist", new_path)
        quit(1)
    return new_path

def message_from_dict(message, dictionary):
    for key, value in dictionary.items():
        if type(value) is dict:
            a = getattr(message, key)
            setattr(message, key, message_from_dict(a, value))
        elif type(value) is str:
             setattr(message, key, value.encode('utf-8'))
        else:
            setattr(message, key, value)
    return message

def make_obb(stl_file, position=(0,0,0), orientation=(0,0,0,1), scale=(1,1,1)):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(stl_file)
    # 'update' the reader i.e. read the .stl file
    reader.Update()

    angle, axis = PyKDL.Rotation.Quaternion(orientation[0],
                                            orientation[1],
                                            orientation[2],
                                            orientation[3]).GetRotAngle()
    print("angle", angle)
    print("angle / 180", angle/np.pi*180)
    print("axis", axis)

    transform = vtk.vtkTransform()
    transform.Scale(scale)
    transform.RotateWXYZ(angle / np.pi * 180, axis.x(), axis.y(), axis.z())
    transform.Translate(position)

    transformFilter = vtk.vtkTransformPolyDataFilter()
    transformFilter.SetInputConnection(reader.GetOutputPort())
    transformFilter.SetTransform(transform)
    transformFilter.Update()

    mesh = transformFilter.GetOutput()
    # print("number of points", mesh.GetNumberOfPoints())
    # If there are no points in 'vtkPolyData' something went wrong
    if mesh.GetNumberOfPoints() == 0:
        raise ValueError(
            "No point data could be loaded from '" + stl_file)
        return None

    obbTree = vtk.vtkOBBTree()
    obbTree.SetDataSet(mesh)
    obbTree.SetTolerance(0.01)
    obbTree.BuildLocator()

    return obbTree

class BlaserSim(object):
    def __init__(self, json_config):
        print("Reading from json file %s" % json_config)
        with open(json_config, 'r') as json_file:
            data = json.load(json_file)
            # Set up subscribers
            self.robot_sub = rospy.Subscriber(data['robot_topic'], PoseStamped, self.pose_cb)
            self.marker_pub = rospy.Publisher('collison_markers', MarkerArray, queue_size=1)
            self.cloud_pub = rospy.Publisher('blaser', PointCloud2, queue_size=1)
            self.blaser_pub = rospy.Publisher('blaser_loc_test', Marker, queue_size=1)
            # Get blaser parameters
            self.blaser_range =      data['blaser_range']
            self.blaser_nrays =      data['blaser_nrays']
            self.blaser_noise =      data['blaser_noise']
            self.blaser_view_angle = data['blaser_view_angle']
            # Set up marker messages and create colliders for each
            self.robot_frame = data['robot_base_frame']
            self.marker_array = MarkerArray()
            self.colliders = []
            self.amp = 0.02
            self.freq = 0.5
            self.sim_start_time = 0
            self.received_points = []
            self.old_move = 0
            for ob in data['objects']:
                self.marker_array.markers.append(message_from_dict(Marker(), ob))
                file_path = clean_resource_path(ob['mesh_resource'])
                pos =   (ob['pose']['position']['x'],
                         ob['pose']['position']['y'],
                         ob['pose']['position']['z'])
                rot =   (ob['pose']['orientation']['x'],
                         ob['pose']['orientation']['y'],
                         ob['pose']['orientation']['z'],
                         ob['pose']['orientation']['w'])
                scale = (ob['scale']['x'], ob['scale']['y'], ob['scale']['z'])
                self.colliders.append(make_obb(file_path, pos, rot, scale))
            self.move_liver()
    
    def move_liver(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            run_time = rospy.Time.now().to_sec()
            offset_z = self.amp * math.sin(self.freq * run_time)
            visualize_marker_array = copy.deepcopy(self.marker_array)
            

            for i in range(len(visualize_marker_array.markers)):
                visualize_marker_array.markers[i].pose.position.z += offset_z

            self.marker_pub.publish(visualize_marker_array)
            rate.sleep()


    def collide(self, start_vec, end_vecs):
        intersection_vtk = vtk.vtkPoints()
        collisions = end_vecs
        colors = [np.uint32(0xff0000) for vec in end_vecs]
        total_points = 0
        for idx, end in enumerate(end_vecs):
            points_intersected = []
            for collider in self.colliders:
                code = collider.IntersectWithLine(start_vec, end, intersection_vtk, None)
                point_data = intersection_vtk.GetData()
                n_points = point_data.GetNumberOfTuples()

                for i in range(n_points):
                    point = point_data.GetTuple3(i)
                    points_intersected.append(point)
                    self.received_points.append(np.array(point))
                    total_points +=1
            if points_intersected:                
                colors[idx] = np.uint32(0x00ff00)
                min_dist = float('inf')
                min_collision = collisions[idx]
                for pt in points_intersected:
                    dist = np.linalg.norm(np.subtract(start_vec, pt))
                    if dist < min_dist:
                        min_collision = pt
                collisions[idx] = min_collision
        return collisions, colors

    def pose_cb(self, msg):
        # Get current time for synchronizing markers and pointclouds
        stamp = rospy.Time.now()
        # Get blaser vectors
        frame = posemath.fromMsg(msg.pose)
        frameid = msg.header.frame_id
        run_time = stamp.to_sec() - self.sim_start_time
        offset_z = self.amp * math.sin(self.freq * run_time)
        offset = PyKDL.Vector(0, 0, offset_z)
        frame.p = frame.p - offset
        start = [frame.p.x(), frame.p.y(), frame.p.z()]
        ends = []
        noise = []
        rand = np.random.randn(self.blaser_nrays) * self.blaser_noise
        for i in range(self.blaser_nrays):
            angle = (i - (self.blaser_nrays - 1)/2) / self.blaser_nrays * self.blaser_view_angle / 180 * np.pi
            vec = PyKDL.Rotation().RotY(angle) * frame.M.UnitZ()
            noise.append([vec.x() * rand[i],
                          vec.y() * rand[i],
                          vec.z() * rand[i]])
            ends.append([start[0] + vec.x() * self.blaser_range,
                         start[1] + vec.y() * self.blaser_range,
                         start[2] + vec.z() * self.blaser_range])
        
        collisions, colors = self.collide(start, ends)
        points = [[v[0] + n[0], v[1] + n[1], v[2] + n[2], c] for v, n, c in zip(collisions, noise, colors)]
        
        #Create pointcloud message
        # this is the blaser collision
        header = Header()
        header.stamp = stamp
        header.frame_id = self.robot_frame
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgb', 12, PointField.UINT32, 1)]
        cloud_msg = point_cloud2.create_cloud(header, fields, points)

        self.cloud_pub.publish(cloud_msg)

        # uncomment this section if you want to see where the blaser is
        m = Marker()
        m.header.frame_id = self.robot_frame
        m.header.stamp = stamp
        m.pose.position.x = frame.p.x()
        m.pose.position.y = frame.p.y()
        m.pose.position.z = frame.p.z()
        m.scale.x = 0.01
        m.scale.y = 0.01
        m.scale.z = 0.01
        m.color.a = 1.0
        self.blaser_pub.publish(m)

        
        np.save('./blaser_results.npy', self.received_points)




if __name__ == '__main__':
    import argparse
    rospy.init_node('blaser_sim', anonymous=True)
    parser = argparse.ArgumentParser(description='Simulated blaser output.')
    parser.add_argument('-j', '--json_config', type=str, help='json configuration file for scene', required=True)
    args = parser.parse_args()
    blaser = BlaserSim(args.json_config)

    rospy.spin()

