#!/usr/bin/env python
import rospy
from extned_object_detection.msg import SimpleObjectArray, ComplexObjectArray

class RobotShortObjectMemory(object):
    
    def __init__(self):
        
        rospy.init_node('robot_short_object_memory')
        
        self.target_frame = rospy.get_param('~target_frame', 'odom')
                
        
        '''
        object:
            type
            sub_type
            pose
            volume (approx cylinder)
        '''
        self.memory = []
        
        rospy.Subscriber('complex_objects', SimpleObjectArray, self.sobject_cb)
        rospy.Subscriber('complex_objects', ComplexObjectArray, self.cobject_cb)
        
    def sobject_cb(self, msg):
        pass
        
    def cobject_cb(self, msg):
        pass
    
    def proceed_object(self, object_):
        
        new_object = {}
        new_object['type'] = object_.type_name
        if len(object_.extracted_info.values) > 0:
            new_object['sub_type'] = '_'.join(object_.extracted_info.values)
        
        new_object['pose']
        
        
        
        
        
        
        
