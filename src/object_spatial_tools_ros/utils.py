#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
import rospy

def obj_transform_to_pose(transform, header):
    #print(transform)
    ps = PoseStamped()
    ps.header = header
    ps.pose.orientation = transform.rotation
    ps.pose.position.x = transform.translation.x
    ps.pose.position.y = transform.translation.y
    ps.pose.position.z = transform.translation.z
    return ps
    
    
def get_common_transform(tf_buffer, msg_header, target_frame):
    transform = tf_buffer.lookup_transform(target_frame,
                                    # source frame:
                                    msg_header.frame_id,
                                    # get the tf at the time the pose was valid
                                    msg_header.stamp,
                                    # wait for at most 1 second for transform, otherwise throw
                                    rospy.Duration(1.0))        
    return transform
