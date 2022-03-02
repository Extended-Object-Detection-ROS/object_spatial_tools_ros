#!/usr/bin/env python

import rospy
from extended_object_detection.msg import SimpleObjectArray, ComplexObjectArray
from object_spatial_tools_ros.utils import obj_transform_to_pose, get_common_transform
import tf2_geometry_msgs
import tf2_ros

class SingleKFUndirectedObjectTracker(object):
    
    '''        
    x_start - start pose state [x, y]
    t_start - time stamp for init
    Q_list - diagonale of Q matrix [Qx, Qy, Qvx, Qvy]
    R_diag - diagonale of R matrix [Rx, Ry]    
    k_decay - coefficien of speed reduction, default = 1    
    '''
    def __init__(self, x_start, t_start, Q_diag, R_diag, k_decay = 1):
        
        self.x = np.array([x_start[0], x_start[1], 0.0, 0.0])
        self.last_t = t_start
        self.last_upd_t = t_start
        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)
        self.k_decay = k_decay
        
        
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])
        
        self.P = np.eye(4)
        
        self.I = np.eye(4)                            
        
    '''
    t - time stamp for predict, seconds
    '''
    def predict(self, t):        
        dt = t - max(self.last_t, self.last_upd_t)
        self.last_t = t
        
        self.F = np.array([[1, 0, dt ,0],
                           [0, 1, 0 ,dt],
                           [0, 0, self.k_decay, 0],
                           [0, 0, 0, self.k_decay]])
        
        self.x = np.dot(self.F, self.x)
        
        self.P = np.dot( np.dot(self.F, self.P), self.F.T) + self.Q
        
            
    '''
    z - measured x, y values
    t - time stamp for update, seconds
    '''
    def update(self, z, t):
        self.last_upd_t = t
                
        y = z - np.dot(self.H, self.x)
        
        S = np.dot( np.dot(self.H, self.P), self.H.T) + self.R
        
        K = np.dot( np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        self.x = self.x + np.dot(K, y)
        
        self.P = np.dot((self.I - np.dot(K, self.H)), self.P)

class RobotKFUndirectedObjectTracker(object):
    
    def __init__(self):
        
        rospy.init_node('robot_kf_undirected_object_tracker')
        
        self.target_frame = rospy.get_param('~target_frame', 'odom')
        
        tracked_objects_type_names = rospy.get_param('~tracked_objects_type_names', [])
        
        self.objects_to_KFs = {}
        for type_obj in tracked_objects_type_names:
            self.objects_to_KFs[type_obj] = []            
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.Qdiag = rospy.get_param('~Qdiag', [0.1, 0.1, 0.1, 0.1])
        self.Rdiag = rospy.get_param('~Rdiag', [0.1, 0.1])
        self.k_decay = rospy.get_param('~k_decay', 1)
        
        update_rate_hz = rospy.get_param('~update_rate_hz', 5)
        
        rospy.Subscriber('simple_objects', SimpleObjectArray, self.sobject_cb)
        rospy.Subscriber('complex_objects', ComplexObjectArray, self.cobject_cb)
        
        rospy.Timer(rospy.Duration(1/update_rate_hz), self.process)
        
    def process(self, event):
        now = rospy.Time.now().to_sec()
        for name, kfs in self.objects_to_KFs.items():
            for i, kf in enumerate(kfs):
                kf.predict(now)
        
                rospy.logwarn(f"{name} {i} {kf.x} {kf.P}")
                
        
        
    def cobject_cb(self, msg):
        now = rospy.Time.now().to_sec()
        transform = get_common_transform(self.tf_buffer, msg.header, self.target_frame)
        
        # collect objects of interests
        detected_objects = {}
        for obj in msg.complex_objects:            
            if obj.type_name in self.objects_to_KFs:                                
                
                ps = obj_transform_to_pose(obj.transform, msg.header)
                
                ps_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform).pose
                
                ps_np = np.array([ps_transformed.position.x, ps_transformed.y])
                
                if obj.type_name in detected_objects:
                    detected_objects[obj.type_name].append(ps_np)
                else:
                    detected_objects[obj.type_name] = [ps_np]
                
                break # TEMP
                
        for obj_name, poses in detected_objects.items():
            
            if len(self.objects_to_KFs[obj_name]) == 0:
                for pose in poses:                
                    self.objects_to_KFs[obj.type_name].append(SingleKFUndirectedObjectTracker(pose, now, self.Qdiag, self.Rdiag, self.k_decay))
            else:
                self.objects_to_KFs[obj.type_name].update(poses[0], now)
                        
        
    def run(self):
        rospy.spin()
