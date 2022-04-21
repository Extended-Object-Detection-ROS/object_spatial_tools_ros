#!/usr/bin/env python

import rospy
from extended_object_detection.msg import SimpleObjectArray, ComplexObjectArray
from object_spatial_tools_ros.utils import obj_transform_to_pose, get_common_transform, get_cov_ellipse_params, quaternion_msg_from_yaw, multi_mahalanobis
import tf2_geometry_msgs
import tf2_ros
import tf
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from threading import Lock
from geometry_msgs.msg import TransformStamped

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
        
        F = np.array([[1, 0, dt ,0],
                           [0, 1, 0 ,dt],
                           [0, 0, self.k_decay, 0],
                           [0, 0, 0, self.k_decay]])
        
        self.x = np.matmul(F, self.x)
        
        self.P = np.matmul( np.matmul(F, self.P), F.T) + self.Q
        
            
    '''
    z - measured x, y values
    t - time stamp for update, seconds
    '''
    def update(self, z, t):
        self.last_upd_t = t
                
        y = z - np.matmul(self.H, self.x)
        
        S = np.matmul( np.matmul(self.H, self.P), self.H.T) + self.R
        
        K = np.matmul( np.matmul(self.P, self.H.T), np.linalg.inv(S))
        
        self.x = self.x + np.matmul(K, y)
        
        self.P = np.matmul((self.I - np.matmul(K, self.H)), self.P)

class RobotKFUndirectedObjectTracker(object):
    
    def __init__(self):
        
        rospy.init_node('robot_kf_undirected_object_tracker')
        
        self.target_frame = rospy.get_param('~target_frame', 'odom')
        self.tf_pub_prefix = rospy.get_param('~tf_pub_prefix', '')
        if self.tf_pub_prefix != '':
            self.tf_pub_prefix = self.tf_pub_prefix + '/'
        
        tracked_objects_type_names = rospy.get_param('~tracked_objects_type_names', [])
        
        self.objects_to_KFs = {}
        self.KFs_prev_elements = {}
        for type_obj in tracked_objects_type_names:
            self.objects_to_KFs[type_obj] = []            
            self.KFs_prev_elements[type_obj] = 0
        
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        self.Qdiag = rospy.get_param('~Qdiag', [0.1, 0.1, 0.1, 0.1])
        self.Rdiag = rospy.get_param('~Rdiag', [0.1, 0.1])
        self.k_decay = rospy.get_param('~k_decay', 1)
        self.lifetime = rospy.get_param('~lifetime', 0)
        self.mahalanobis_max = rospy.get_param('~mahalanobis_max', 1)
        
        self.mutex = Lock()
        
        update_rate_hz = rospy.get_param('~update_rate_hz', 5)
        
        self.vis_pub = rospy.Publisher('~vis', MarkerArray, queue_size = 1)
        
        rospy.Subscriber('simple_objects', SimpleObjectArray, self.sobject_cb)
        rospy.Subscriber('complex_objects', ComplexObjectArray, self.cobject_cb)
        
        rospy.Timer(rospy.Duration(1/update_rate_hz), self.process)
        
    def process(self, event):
        self.mutex.acquire()
        now = rospy.Time.now().to_sec()
        for name, kfs in self.objects_to_KFs.items():
            self.KFs_prev_elements[name] = len(kfs)
            remove_index = []
            for i, kf in enumerate(kfs):
                if self.lifetime == 0 or (now - kf.last_upd_t) < self.lifetime:
                    kf.predict(now)
                else:
                    remove_index.append(i)
        
                #rospy.logwarn(f"{name} {i} {kf.x} {kf.P}")
            for index in sorted(remove_index, reverse=True):
                del kfs[index]
                
        self.to_marker_array()
        self.to_tf()
        self.mutex.release()
        
    def to_tf(self):
        for name, kfs in self.objects_to_KFs.items():
            for i, kf in enumerate(kfs):
                
                t = TransformStamped()
                
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = self.target_frame
                t.child_frame_id = self.tf_pub_prefix+name+f'_{i}'
                
                t.transform.translation.x = kf.x[0]
                t.transform.translation.y = kf.x[1]
                
                t.transform.rotation.w = 1
                
                self.tf_broadcaster.sendTransform(t)                
                
                
                
    def to_marker_array(self):
        now = rospy.Time.now()
        marker_array = MarkerArray()
        for name, kfs in self.objects_to_KFs.items():
            i = 0
            for i, kf in enumerate(kfs):
                # POSE AND SPEED
                marker = Marker()
                
                marker.header.stamp = now
                marker.header.frame_id = self.target_frame
                
                marker.ns = name+"_pose"
                marker.id = i
                
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                
                marker.pose.position.x = kf.x[0]
                marker.pose.position.y = kf.x[1]                                                
                
                marker.pose.orientation = quaternion_msg_from_yaw(np.arctan2(kf.x[3], kf.x[2]))
                                
                marker.color.r = 1
                marker.color.a = 1
                
                marker.scale.x = np.hypot(kf.x[3], kf.x[2])
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                
                marker_array.markers.append(marker)
                
                # COV ELLIPSE
                marker = Marker()
                
                marker.header.stamp = now
                marker.header.frame_id = self.target_frame
                
                marker.ns = name+"_el"
                marker.id = i
                
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                
                marker.pose.position.x = kf.x[0]
                marker.pose.position.y = kf.x[1]                
                                
                marker.color.r = 1
                marker.color.b = 1
                marker.color.a = 0.5
                
                r1, r2, th = get_cov_ellipse_params(kf.P[:2,:2])
                
                marker.scale.x = r1
                marker.scale.y = r2
                marker.scale.z = 0.01
                marker.pose.orientation = quaternion_msg_from_yaw(th)
                
                marker_array.markers.append(marker)
            for j in range(i+1, self.KFs_prev_elements[name]):
                for t in ["_el","_pose"]:
                    marker = Marker()                
                    marker.header.stamp = now
                    marker.ns = name+t
                    marker.id = j
                    marker.action = Marker.DELETE
                    marker_array.markers.append(marker)
                
        self.vis_pub.publish(marker_array)
    
    def sobject_cb(self, msg):
        self.mutex.acquire()
        now = rospy.Time.now().to_sec()
        transform = get_common_transform(self.tf_buffer, msg.header, self.target_frame)
        if transform is None:
            self.mutex.release()
            return
        
        # collect objects of interests
        detected_objects = {}
        for obj in msg.objects:            
            if obj.type_name in self.objects_to_KFs:                                
                
                ps = obj_transform_to_pose(obj.transform, msg.header)
                
                ps_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform).pose
                
                ps_np = np.array([ps_transformed.position.x, ps_transformed.position.y])
                
                if obj.type_name in detected_objects:
                    detected_objects[obj.type_name].append(ps_np)
                else:
                    detected_objects[obj.type_name] = [ps_np]                                
                
        for obj_name, poses in detected_objects.items():
            
            if len(self.objects_to_KFs[obj_name]) == 0:
                for pose in poses:                
                    self.objects_to_KFs[obj.type_name].append(SingleKFUndirectedObjectTracker(pose, now, self.Qdiag, self.Rdiag, self.k_decay))
            else:
                
                m_poses_np = np.array(poses) # [[x, y]]
                
                kf_poses_np = np.array([[kf.x[0], kf.x[1]] for kf in self.objects_to_KFs[obj_name]])
                
                S_np = np.array([np.linalg.inv(kf.P[:2,:2]) for kf in self.objects_to_KFs[obj_name]]) # already inv!
                
                D = multi_mahalanobis(m_poses_np, kf_poses_np, S_np)
                
                #print('D', D, D.shape)
                
                extra_poses = list(range(len(poses)))
                while not rospy.is_shutdown():
                    
                    closest = np.unravel_index(np.argmin(D, axis=None), D.shape)
                    #print(closest, D[closest])
                                    
                    if D[closest] > self.mahalanobis_max:
                        break
                    
                    self.objects_to_KFs[obj.type_name][closest[1]].update(poses[closest[0]], now)
                    
                    D[closest[0],:] = np.inf
                    D[:,closest[1]] = np.inf
                    extra_poses.remove(closest[0])
                                                        
                #for i in range(D.shape[0]):
                for i in extra_poses:                
                    self.objects_to_KFs[obj.type_name].append(SingleKFUndirectedObjectTracker(poses[i], now, self.Qdiag, self.Rdiag, self.k_decay))
        self.mutex.release()
        
    def cobject_cb(self, msg):
        self.mutex.acquire()
        now = rospy.Time.now().to_sec()
        transform = get_common_transform(self.tf_buffer, msg.header, self.target_frame)
        if transform is None:
            self.mutex.release()
            return
        
        # collect objects of interests
        detected_objects = {}
        for obj in msg.complex_objects:            
            if obj.type_name in self.objects_to_KFs:                                
                
                ps = obj_transform_to_pose(obj.transform, msg.header)
                
                ps_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform).pose
                
                ps_np = np.array([ps_transformed.position.x, ps_transformed.position.y])
                
                if obj.type_name in detected_objects:
                    detected_objects[obj.type_name].append(ps_np)
                else:
                    detected_objects[obj.type_name] = [ps_np]                                
                
        for obj_name, poses in detected_objects.items():
            
            if len(self.objects_to_KFs[obj_name]) == 0:
                for pose in poses:                
                    self.objects_to_KFs[obj.type_name].append(SingleKFUndirectedObjectTracker(pose, now, self.Qdiag, self.Rdiag, self.k_decay))
            else:
                
                m_poses_np = np.array(poses) # [[x, y]]
                
                kf_poses_np = np.array([[kf.x[0], kf.x[1]] for kf in self.objects_to_KFs[obj_name]])
                
                S_np = np.array([np.linalg.inv(kf.P[:2,:2]) for kf in self.objects_to_KFs[obj_name]]) # already inv!
                
                D = multi_mahalanobis(m_poses_np, kf_poses_np, S_np)
                
                #print('D', D, D.shape)
                
                extra_poses = list(range(len(poses)))
                while not rospy.is_shutdown():
                    
                    closest = np.unravel_index(np.argmin(D, axis=None), D.shape)
                    #print(closest, D[closest])
                                    
                    if D[closest] > self.mahalanobis_max:
                        break
                    
                    self.objects_to_KFs[obj.type_name][closest[1]].update(poses[closest[0]], now)
                    
                    D[closest[0],:] = np.inf
                    D[:,closest[1]] = np.inf
                    extra_poses.remove(closest[0])
                                                        
                #for i in range(D.shape[0]):
                for i in extra_poses:                
                    self.objects_to_KFs[obj.type_name].append(SingleKFUndirectedObjectTracker(poses[i], now, self.Qdiag, self.Rdiag, self.k_decay))
        self.mutex.release()
                        
        
    def run(self):
        rospy.spin()
