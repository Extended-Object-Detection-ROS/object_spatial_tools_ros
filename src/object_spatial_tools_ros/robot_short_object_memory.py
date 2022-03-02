#!/usr/bin/env python
import rospy
from extended_object_detection.msg import SimpleObjectArray, ComplexObjectArray
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import numpy as np
from visualization_msgs.msg import Marker
from object_spatial_tools_ros.utils import obj_transform_to_pose, get_common_transform

class RobotShortObjectMemory(object):
    
    def __init__(self):
        
        rospy.init_node('robot_short_object_memory')
        
        self.target_frame = rospy.get_param('~target_frame', 'odom')
        
        #self.score_coefficients = np.array(rospy.get_param('~score_coefficients', [1,1,1,1]))
        self.score_coefficients = np.array([1,1,1,1])
        
        self.score_multiplyer = rospy.get_param('~score_multiplyer', 2)
        update_rate_hz = rospy.get_param('~update_rate_hz', 5)
        self.forget_time = rospy.Duration(rospy.get_param('~forget_time', 10))
        
        self.update_count_thresh = rospy.get_param('~update_count_thresh', 0)
               
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        '''
        object:
            type: str
            sub_type: str
            pose: geometry_msgs/Pose
            np_pose: np.array([x,y,z])
            volume: (approx cylinder) {radius:float, height:float}
            occurr: int
            stamp: ros time
            changed: bool
            forgoten: bool
        '''
        self.memory = []
        
        self.marker_pub = rospy.Publisher('~memory_map', Marker, queue_size = 10)
        
        rospy.Timer(rospy.Duration(1/update_rate_hz), self.update_cb)
        
        rospy.Subscriber('simple_objects', SimpleObjectArray, self.sobject_cb)
        rospy.Subscriber('complex_objects', ComplexObjectArray, self.cobject_cb)
        
    def update_cb(self, event):
        now = rospy.Time.now()
        
        del_indexes = []
        
        for i, obj in enumerate(self.memory):
            if obj['forgoten']:
                obj['occurr'] = obj['occurr'] - 1
                if obj['occurr'] <= 0:
                    del_indexes.append(i)
                #obj['changed'] = True
            else:
                if (now - obj['stamp']) > self.forget_time:
                    obj['forgoten'] = True
                    #obj['changed'] = True
            
                
        
        for index in sorted(del_indexes, reverse=True):
            del self.memory[index]
                        
        self.publish_memory_as_markers()
        
    def publish_memory_as_markers(self):
        now = rospy.Time.now()
        for i, obj in enumerate(self.memory):
            #if obj['changed']:
                #obj['changed'] = False
                
            marker_msg = Marker()
            marker_msg.header.frame_id = self.target_frame
            marker_msg.header.stamp = now
            
            marker_msg.id = i
            marker_msg.ns = 'cylinder'
            marker_msg.action = Marker.ADD
            marker_msg.type = Marker.CYLINDER
            marker_msg.pose.position = obj['pose'].position
            marker_msg.pose.orientation.w = 1
            #marker_msg.color.b = 1
            if obj['forgoten']:
                marker_msg.color.r = 1
                marker_msg.color.g = 1
                marker_msg.color.b = 1
                marker_msg.color.a = 0.25
            else:
                marker_msg.color.g = 1
                marker_msg.color.a = 1
            marker_msg.scale.x = obj['volume']['radius'] * 2
            marker_msg.scale.y = obj['volume']['radius'] * 2
            marker_msg.scale.z = obj['volume']['height']
            
            self.marker_pub.publish(marker_msg)
            
            marker_msg = Marker()
            marker_msg.header.frame_id = self.target_frame
            marker_msg.header.stamp = now
            marker_msg.id = i
            marker_msg.ns = 'text'
            marker_msg.action = Marker.ADD
            marker_msg.type = Marker.TEXT_VIEW_FACING
            
            marker_msg.pose.position.x = obj['pose'].position.x
            marker_msg.pose.position.y = obj['pose'].position.y
            marker_msg.pose.position.z = obj['pose'].position.z + obj['volume']['height'] * 4
            #marker_msg.pose.position.z = marker_msg.pose.position.z + obj['volume']['height'] * 4
            marker_msg.pose.orientation.w = 1
            
            if obj['forgoten']:
                marker_msg.color.r = 1
                marker_msg.color.g = 1
                marker_msg.color.b = 1
                marker_msg.color.a = 0.25
            else:
                marker_msg.color.g = 1
                marker_msg.color.a = 1
            
            marker_msg.scale.z = 0.1
            marker_msg.text = f"{obj['type']}({obj['sub_type']}) {obj['occurr']}"
            
            self.marker_pub.publish(marker_msg)
                                        
    def sobject_cb(self, msg):
        pass
        
    def cobject_cb(self, msg):
        transform = get_common_transform(self.tf_buffer, msg.header, self.target_frame)
        for obj in msg.complex_objects:
            self.proceed_object(msg.header, obj, transform)
        
    
    def proceed_object(self, header, object_, transform):
        
        new_object = {}
        new_object['type'] = object_.type_name
        if len(object_.extracted_info.values) > 0:
            new_object['sub_type'] = '_'.join(object_.extracted_info.values)
        else:
            new_object['sub_type'] = ""
        
        ps = obj_transform_to_pose(object_.transform, header)
        new_object['pose'] = tf2_geometry_msgs.do_transform_pose(ps, transform).pose
        new_object['np_pose'] = np.array([new_object['pose'].position.x, new_object['pose'].position.y, new_object['pose'].position.z])
        r = np.abs(object_.rect.cornerTranslates[1].x - object_.rect.cornerTranslates[2].x)/2
        h = np.abs(object_.rect.cornerTranslates[0].y - object_.rect.cornerTranslates[1].y)
        new_object['volume'] = {'radius': r, 'height': h}
        new_object['occurr'] = 1
        new_object['stamp'] = header.stamp
        new_object['changed'] = True
        new_object['forgoten'] = False
        
        #print(new_object)
        self.add_object_to_memory(new_object)        
        #print(len(self.memory))
        
    def add_object_to_memory(self, new_object):
        if len(self.memory) == 0:
            self.memory.append(new_object)
        else:
            same_types_obj = [obj for obj in self.memory if obj['type'] == new_object['type']]
            if len(same_types_obj) == 0:
                self.memory.append(new_object)
            else:
                same_sub_types = [obj for obj in same_types_obj if obj['sub_type'] == new_object['sub_type'] and obj['stamp'] != new_object['stamp']]
                if len(same_sub_types) == 0:
                    self.memory.append(new_object)
                else:
                    np_poses = np.array([obj['np_pose'] for obj in same_sub_types])
                    volumes = np.array([(obj['volume']['radius'],obj['volume']['height']) for obj in same_sub_types])
                                                        
                    dr = np_poses - new_object['np_pose']
                    dxy = np.hypot(dr[:,0], dr[:,1])
                    dz = np.abs(dr[:,2])
                    
                    volume = np.array((new_object['volume']['radius'], new_object['volume']['height']))
                    dv = volumes - volume
                    
                    #print(dxy.shape, dz.shape, dv.shape)
                    d = np.concatenate([np.expand_dims(dxy,1), np.expand_dims(dz,1), dv], axis = 1)
                    
                    score = np.dot(d, self.score_coefficients)
                    
                    min_score_ind = np.argmin(score)
                    min_score = score[min_score_ind]
                    
                    thresh = (new_object['volume']['radius'] * 2 + new_object['volume']['height']) * self.score_multiplyer
                    
                    if thresh > min_score:
                        
                        
                        if self.update_count_thresh != 0:
                            num = self.update_count_thresh
                        else:
                            num = same_sub_types[min_score_ind]['occurr']
                        
                        same_sub_types[min_score_ind]['stamp'] = new_object['stamp']
                        same_sub_types[min_score_ind]['np_pose'] = (same_sub_types[min_score_ind]['np_pose'] * num + new_object['np_pose'])/(num+1)
                        same_sub_types[min_score_ind]['pose'].position.x = same_sub_types[min_score_ind]['np_pose'][0]
                        same_sub_types[min_score_ind]['pose'].position.y = same_sub_types[min_score_ind]['np_pose'][1]
                        same_sub_types[min_score_ind]['pose'].position.z = same_sub_types[min_score_ind]['np_pose'][2]
                                                                        
                        for el in ['radius', 'height']:
                            same_sub_types[min_score_ind]['volume'][el] = (same_sub_types[min_score_ind]['volume'][el] * num + new_object['volume'][el])/(num+1)                            
                            
                        same_sub_types[min_score_ind]['changed'] = True
                        same_sub_types[min_score_ind]['occurr']+=1
                        same_sub_types[min_score_ind]['forgoten'] = False
                        
                    else:
                        self.memory.append(new_object)
                    
    
    def run(self):
        rospy.spin()
