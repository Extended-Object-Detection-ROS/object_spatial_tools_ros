#!/usr/bin/env python
import rospy
from extended_object_detection.msg import SimpleObjectArray, ComplexObjectArray
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import numpy as np

class RobotShortObjectMemory(object):
    
    def __init__(self):
        
        rospy.init_node('robot_short_object_memory')
        
        self.target_frame = rospy.get_param('~target_frame', 'odom')
        
        self.score_coefficients = ros.get_param('~score_coefficients', np.array([1,1,1,1]))
               
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
        '''
        self.memory = []
        
        
        
        rospy.Subscriber('simple_objects', SimpleObjectArray, self.sobject_cb)
        rospy.Subscriber('complex_objects', ComplexObjectArray, self.cobject_cb)
        
    def sobject_cb(self, msg):
        pass
        
    def cobject_cb(self, msg):
        transform = self.get_common_transform(msg.header)
        for obj in msg.complex_objects:
            self.proceed_object(msg.header, obj, transform)
    
    def proceed_object(self, header, object_, transform):
        
        new_object = {}
        new_object['type'] = object_.type_name
        if len(object_.extracted_info.values) > 0:
            new_object['sub_type'] = '_'.join(object_.extracted_info.values)
        else:
            new_object['sub_type'] = None
        
        ps = obj_transform_to_pose(object_.transform, header)
        new_object['pose'] = tf2_geometry_msgs.do_transform_pose(ps, transform).pose
        new_object['np_pose'] = np.array([new_object['pose'].x, new_object['pose'].y, new_object['pose'].z])
        r = np.abs(object_.rect.cornerTranslates[1].x - object_.rect.cornerTranslates[2].x)/2
        h = np.abs(object_.rect.cornerTranslates[0].y - object_.rect.cornerTranslates[1].y)
        new_object['volume'] = {'radius': r, 'height': h}
        new_object['occurr'] = 1
        new_object['stamp'] = header.stamp
        
        #print(new_object)
        self.add_object_to_memory(new_object)
        
    def add_object_to_memory(self, new_object):
        if len(self.memory) == 0:
            self.memory.append(new_object)
        else:
            same_types_obj = [obj for obj in self.memory if obj['type'] == new_object['type']]
            if len(same_types_obj) == 0:
                self.memory.append(new_object)
            else:
                same_sub_types = [obj for obj in same_sub_types if obj['sub_type'] == new_object['sub_type'] and obj['stamp'] != new_object['stamp']]
                if len(same_sub_types) == 0:
                    self.memory.append(new_object)
                else:
                    np_poses = np.array([obj['np_pose'] for obj in same_sub_types])
                    volumes = np.array([(obj['volume']['radius'],obj['volume']['height']) fro obj in same_sub_types])
                                                        
                    dr = np_poses - new_object['np_pose']
                    dxy = np.hypot(d[:,0], d[:,1])
                    dz = np.abs(d[:,2])
                    
                    dv = volumes - np.array((new_object['volume']['radius'], new_object['volume']['height']))
                    
                    d = np.concatenate([dxy, dz, dv], axis = 1)
                    
                    score = np.dot(d, self.score_coefficients)
                    
                    min_score_ind = np.argmin(score)
                    min_score = score[min_score_ind]
                    
                    thresh
                    
                    
            
        
    def get_common_transform(self, msg_header):
        transform = self.tf_buffer.lookup_transform(self.target_frame,
                                       # source frame:
                                       msg_header.frame_id,
                                       # get the tf at the time the pose was valid
                                       msg_header.stamp,
                                       # wait for at most 1 second for transform, otherwise throw
                                       rospy.Duration(1.0))
        #print(transform)
        return transform
    
    def run(self):
        rospy.spin()
        
def obj_transform_to_pose(transform, header):
    #print(transform)
    ps = PoseStamped()
    ps.header = header
    ps.pose.orientation = transform.rotation
    ps.pose.position.x = transform.translation.x
    ps.pose.position.y = transform.translation.y
    ps.pose.position.z = transform.translation.z
    return ps
    
        
        
        
        
        
        
        
