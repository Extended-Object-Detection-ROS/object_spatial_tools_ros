#!/usr/bin/env python
import rospy
from extended_object_detection.msg import SimpleObjectArray, ComplexObjectArray
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

class RobotShortObjectMemory(object):
    
    def __init__(self):
        
        rospy.init_node('robot_short_object_memory')
        
        self.target_frame = rospy.get_param('~target_frame', 'odom')
               
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        '''
        object:
            type
            sub_type
            pose
            volume (approx cylinder)
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
        
        ps = obj_transform_to_pose(object_.transform, header)
        new_object['pose'] = tf2_geometry_msgs.do_transform_pose(ps, transform).pose
        
        #print(new_object)
        
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
    
        
        
        
        
        
        
        
