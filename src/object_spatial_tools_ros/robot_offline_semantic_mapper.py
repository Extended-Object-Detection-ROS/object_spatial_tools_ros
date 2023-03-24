#!/usr/bin/env python
# coding: utf-8
import rospy
from extended_object_detection.msg import SimpleObjectArray
import numpy as np
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
import tf2_ros
from object_spatial_tools_ros.utils import obj_transform_to_pose, get_common_transform
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import seaborn as sns
import struct
from std_msgs.msg import Header

class OneClassObject(object):
    
    def __init__(self, first_pose):        
        # each object: x, y, z, ts
        #self.poses = np.array((0, 4))
        #self.add_object(first_pose)
        self.poses = np.array([first_pose])
        #print('init',self.poses.shape)
                
            
    def add_object(self, new_pose):        
        self.poses = np.concatenate((self.poses, [new_pose]), axis = 0)
        #print('add',self.poses.shape)
        
    

class OfflineSemanticMapper(object):
    
    
    def __init__(self):
        
        rospy.init_node('offline_semantic_mapper')
        
        # for params
        self.target_frame = rospy.get_param('target_frame', "map")
        
        
        
        # storage
        
        ## name: OneClassObject
        self.detected_objects = {}
        
        
        # ros stuff
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.pc_fields = [PointField('x', 0, PointField.FLOAT32, 1),
                          PointField('y', 4, PointField.FLOAT32, 1),
                          PointField('z', 8, PointField.FLOAT32, 1),          
                          PointField('rgba', 12, PointField.UINT32, 1),
                          ]
        self.pc_palette = sns.color_palette("tab10", 10)
        
        ## publishers
        self.point_cloud_pub = rospy.Publisher('~object_cloud', PointCloud2, queue_size = 1)
        
        
        ## subscribers & timers
        rospy.Subscriber('simple_objects', SimpleObjectArray, self.so_cb)
        
        rospy.Timer(rospy.Duration(1), self.publish_cloud)
        
    
    def so_cb(self, msg):
            
        if len(msg.objects) > 0:
            transform = get_common_transform(self.tf_buffer, msg.header, self.target_frame)
            
        for base_obj in msg.objects:
            # skip objects with unit transform (distance unknown)
            if base_obj.transform.translation.z == 1:
                continue
            
            # transform object to target frame            
            ps = obj_transform_to_pose(base_obj.transform, msg.header)                
            ps_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform).pose
            
            new_pose = [ps_transformed.position.x,
                        ps_transformed.position.y,
                        ps_transformed.position.z,
                        msg.header.stamp.to_sec()]
            if base_obj.type_name in self.detected_objects:
                self.detected_objects[base_obj.type_name].add_object(new_pose)
            else:
                self.detected_objects[base_obj.type_name] = OneClassObject(new_pose)
                
    def publish_cloud(self, event):
        points = []
        for i, obj in enumerate( self.detected_objects.values()):
            xyz = obj.poses[:,0:3]
            rgb = self.pc_palette[i % len(self.pc_palette)] # typle of 0-1 values
            r = int(rgb[0]*255.0)
            g = int(rgb[1]*255.0)
            b = int(rgb[2]*255.0)
            rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
            #print(type(rgb), rgb)
            for pose in xyz.tolist():
                points.append(pose + [rgb])        
        #print(points)
        if len(points):
            header = Header()
            header.frame_id = self.target_frame
            header.stamp = rospy.Time.now()
            pc2 = point_cloud2.create_cloud(header, self.pc_fields, points)
                
            self.point_cloud_pub.publish(pc2)
            
    def run(self):
        rospy.spin()
        
        
