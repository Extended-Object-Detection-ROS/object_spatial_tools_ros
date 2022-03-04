#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped, Quaternion
import rospy
import numpy as np
import tf

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
    try:
        transform = tf_buffer.lookup_transform(target_frame,
                                    # source frame:
                                    msg_header.frame_id,
                                    # get the tf at the time the pose was valid
                                    msg_header.stamp,
                                    # wait for at most 1 second for transform, otherwise throw
                                    rospy.Duration(1.0))        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn(f"Can't find transform {target_frame} --> {msg_header.frame_id}")
        return None
    return transform


def get_cov_ellipse_params(cov_matrix):
    a = cov_matrix[0,0]
    b = cov_matrix[0,1]
    c = cov_matrix[1,1]
    
    d = 0.5 * (a + c)
    D = np.sqrt( (0.5*(a - c))**2 + b**2)
    
    l1 = d + D
    l2 = d - D
    
    if b == 0:
        if a >= c:
            theta = 0
        else:
            theta = np.pi /2
    else:
        theta = np.acrtan2(l1 -a, b)
        
    return np.sqrt(l1), np.sqrt(l2), theta
   
   
def quaternion_msg_from_yaw(yaw):
    qu = tf.transformations.quaternion_from_euler(0,0,yaw)    
    msg = Quaternion()
    msg.x = qu[0]
    msg.y = qu[1]
    msg.z = qu[2]
    msg.w = qu[3]
    return msg

'''
Sm is inv cov matrixes
'''
def multi_mahalanobis(x, y, Sm):
    
    #print(x.shape, y.shape)
    
    xx = np.tile(x, (y.shape[0],1))
    yy = np.repeat(y, x.shape[0], axis = 0)
            
    
    SSm = np.repeat(Sm, x.shape[0], axis = 0)
    
    #print(xx.shape, yy.shape)
    
    d = xx - yy
        
    de = np.expand_dims(d, 1)
    dee = np.expand_dims(d, 2)
    
    #print(de.shape, SSm.shape)
    
    D = np.matmul(de, SSm)
    #print(D.shape)
    #D = D.squeeze(1)
    
    
    D = np.sqrt( np.matmul( D, dee))
    
    D = D.reshape((x.shape[0], y.shape[0]))
    
    return D
    
    
    
