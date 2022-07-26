#!/usr/bin/env python
# coding: utf-8
import rospy
import tf
import numpy as np
from scipy.cluster.hierarchy import dendrogram, linkage
from scipy.cluster.hierarchy import fcluster
from scipy.spatial import distance
import yaml
from std_srvs.srv import Empty
import os.path

from extended_object_detection.msg import SimpleObjectArray
from geometry_msgs.msg import PointStamped, PoseWithCovariance, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from object_spatial_tools_ros.msg import SemanticMap, SemanticObject
from object_spatial_tools_ros.utils import quaternion_msg_from_yaw

class SimpleMapObject(object):
    def __init__(self, type_id, type_name, data):
        self.type_id = type_id
        self.type_name = type_name
        
        # not clustered data
        if data is not None:
            self.all_poses = np.array([data])                
        else:
            self.all_poses = np.zeros((0,5))
        
        self.clusters = []
        self.saved_clusters = []        
            
        self.updated = True
            
    def appendPoint(self, data):
        global mh_thres
        
        if len(self.saved_clusters) > 0:
            dist = []
            for cluster in self.saved_clusters:                
                dist.append(distance.mahalanobis(data[:3], cluster['centroid'], np.linalg.inv(cluster['cov']) ))   
            
            # search for unmarked cluster
            while len(dist) > 0:
                nearest_cluster = dist.index(min(dist))
                if self.saved_clusters[nearest_cluster]['mark']:
                    del dist[nearest_cluster]
                else:
                    break
            
            
            if len(dist) > nearest_cluster and dist[nearest_cluster] < mh_thres:            
                self.saved_clusters[nearest_cluster]['centroid'] = (self.saved_clusters[nearest_cluster]['centroid'] * self.saved_clusters[nearest_cluster]['size'] + data[:3]) / (self.saved_clusters[nearest_cluster]['size']+1.)
                
                m = self.saved_clusters[nearest_cluster]['size']/(self.saved_clusters[nearest_cluster]['size']+1.)
                m2 = m / (self.saved_clusters[nearest_cluster]['size']+1.)
                
                d = (data[:3] - self.saved_clusters[nearest_cluster]['centroid']).reshape(3,1)                                
                
                self.saved_clusters[nearest_cluster]['cov'] = m * self.saved_clusters[nearest_cluster]['cov'] + m2 * np.dot(d, d.T)                                                      
                self.saved_clusters[nearest_cluster]['size'] += 1
                
                self.saved_clusters[nearest_cluster]['mark'] = True
                return
        
        self.all_poses = np.vstack((self.all_poses, data))                            
        self.updated = False                
    
    def unmark_all(self):
        for sc in self.saved_clusters:
            sc['mark'] = False
    
    def update(self):   
        global cluster_min_size
        global cluster_max_size
        
        global cluster_dist_thres
        if self.updated:
            return
        if self.all_poses.shape[0] < cluster_min_size:
            return        
        Z = linkage(self.all_poses, method='complete', metric='euclidean')  
        fclusters = fcluster(Z, cluster_dist_thres, criterion='distance')        
        
        self.clusters = []
        for id_ in set(fclusters):                        
            cluster = self.all_poses[fclusters == id_]            
            # filter by size
            if cluster.shape[0] > cluster_min_size:
                centroid = np.median(cluster[:,:3] , axis = 0)                
                
                new_cluster = {}
                new_cluster['centroid'] = centroid                
                new_cluster['size'] = cluster.shape[0]
                new_cluster['cov'] = np.cov(cluster[:,:3].T)
                new_cluster['height'] = np.mean(cluster[:,3])
                new_cluster['width'] = np.mean(cluster[:,4])    
                
                if cluster.shape[0] > cluster_max_size:                    
                    self.all_poses = np.delete(self.all_poses, np.argwhere(fclusters == id), axis = 0)                                        
                    self.saved_clusters.append(new_cluster)
                else:
                    self.clusters.append(new_cluster)        
        
        self.updated = True
        
    def validate(self):
        self.unmark_all()
        
        for i in range(len(self.saved_clusters)):
            if self.saved_clusters[i]['mark']:
                continue
            for j in range(i+1, len(self.saved_clusters)):
                if self.saved_clusters[j]['mark']:
                    continue
                if self.is_intersects(self.saved_clusters[i], self.saved_clusters[j]):
                    self.saved_clusters[i] = self.merge_clusters(self.saved_clusters[i], self.saved_clusters[j])
                    self.saved_clusters[j]['mark'] = True
                   
        temp_list = []
        for cl in self.saved_clusters:
            if not cl['mark']:
                temp_list.append(cl)
        self.saved_clusters = temp_list
                            
            
    def is_intersects(self, cluster1, cluster2):
        # Height intersection
        h1l = cluster1['centroid'][2] - cluster1['height']
        h1u = cluster1['centroid'][2] + cluster1['height']
        h2l = cluster2['centroid'][2] - cluster2['height']
        h2u = cluster2['centroid'][2] + cluster2['height']
        if min(h1u, h2u) < max(h1l, h2l):
            return False
        
        # circle intersection
        r1 = np.hypot( cluster1['centroid'][0] - cluster2['centroid'][0], cluster1['centroid'][1] - cluster2['centroid'][1])        
        r2 = cluster1['width'] + cluster2['width']        
        if r1 > r2:
            return False
        
        return True
    
    def merge_clusters(self, cluster1, cluster2):
        merged_cluster = {}
        alpha = cluster1['size'] / float(cluster1['size'] + cluster2['size'])
        
        merged_cluster['centroid'] = alpha * cluster1['centroid'] + (1 - alpha) * cluster2['centroid']
        merged_cluster['cov'] = alpha * cluster1['cov'] + (1 - alpha) * cluster2['cov']
        merged_cluster['size'] = cluster1['size'] + cluster2['size']
        merged_cluster['height'] = alpha * cluster1['height'] + (1 - alpha) * cluster2['height']
        merged_cluster['width'] = alpha * cluster1['width'] + (1 - alpha) * cluster2['width']
        merged_cluster['mark'] = False
        
        return merged_cluster
    
    
#------------------------
# MAPPER
#------------------------

class SimpleClusterObjectMapper(object):
    def __init__(self):
        global mh_thres
        global cluster_dist_thres
        global cluster_min_size
        global cluster_max_size
        
        # node init
        rospy.init_node("simple_cluster_object_mapper")
        
        # params
        self.map_frame = rospy.get_param("~map_frame","map")
                       
        mh_thres = rospy.get_param("~mh_thres", 3.)                       
        cluster_dist_thres = rospy.get_param("~cluster_dist_thres", 3.)
        cluster_min_size = rospy.get_param("~cluster_min_size", 10)
        cluster_max_size = rospy.get_param("~cluster_max_size", 100)
        
        self.tfl = tf.TransformListener()
            
        # MAP stuff
        self.MAP = {}        
        
        ## load map from file
        self.map_file = rospy.get_param("~map_file","/tmp/semantic_map.yaml")
        
        ## clear map on start
        self.clear_map_on_start = rospy.get_param("~clear_map_on_start", False)
        
        self.update_map = rospy.get_param("~update_map", False)
        
        if not self.clear_map_on_start:
            ## update existing map or not
            
            if not self.update_map:
                pub_rate_sec = rospy.get_param("~pub_rate_sec",1.0)
        
        ## check if map file really exist if not update write it anyway
        if not self.clear_map_on_start:
            if not os.path.exists(self.map_file):
                
                if not self.update_map:
                    rospy.logerr("[simple_object_mapper] could not found map {}! Exit.".format(self.map_file))
                else:
                    rospy.logwarn("[simple_object_mapper] could not found map {}! Created a new one.".format(self.map_file))
            else:
                with open(self.map_file, 'rb') as infile:
                    self.MAP = self.dictionary_to_map(yaml.load(infile, Loader=yaml.FullLoader))
        
        ## semantic format publisher
        self.map_pub = rospy.Publisher('semantic_map', SemanticMap, queue_size = 1)                
        
        ## marker array format publisher
        self.publish_map = rospy.get_param('~publish_map_as_markers', True)        
        if self.publish_map:
            self.map_pub_markers = rospy.Publisher('semantic_object_map_as_markers',MarkerArray,queue_size = 1)
        
        ## point cloud format publisher
        self.publish_cloud = rospy.get_param('~publish_cloud', False)
        
        if self.publish_cloud:
            self.cloud_pub = rospy.Publisher("object_cloud",PointCloud2,queue_size=1)
            self.points = []
            self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1)]
        
        # Services 
        rospy.Service('~save_semantic_map', Empty, self.save_map_srv)
        rospy.Service('~clear_map', Empty, self.clear_map_srv)
        
        if self.update_map:
            # Subscribers
            rospy.Subscriber('detected_objects', SimpleObjectArray, self.detected_objects_cb)
        else:
            rospy.Timer(rospy.Duration(pub_rate_sec), self.map_rate_pub)
        
    def map_rate_pub(self, event):
        
        if self.publish_map:
            self.map_pub_markers.publish(self.map_to_marker_array())
        
        self.map_pub.publish(self.map_to_semantic())
        
    def detected_objects_cb(self, msg):                                
        # unmark all clusters to prevent addition of more than one points to cluster
        for t_object in self.MAP.values():
            t_object.unmark_all()
        
        # new points addition
        for detected_object in msg.objects:
            # get point in map frame
            ps = PointStamped()
            ps.header.frame_id = msg.header.frame_id
            ps.header.stamp = rospy.Time(0)#.now()#msg.header.stamp
            ps.point.x = detected_object.transform.translation.x
            ps.point.y = detected_object.transform.translation.y
            ps.point.z = detected_object.transform.translation.z
            try:
                poseSt = self.tfl.transformPoint( self.map_frame, ps)                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("No transfrom from {} to {}".format( self.map_frame, msg.header.frame_id ))
                return
                       
            # calc width and height
            v1 = detected_object.rect.cornerTranslates[0]
            v2 = detected_object.rect.cornerTranslates[1]
            v3 = detected_object.rect.cornerTranslates[2]            
            distance = detected_object.transform.translation.z       
            if distance == 1:
                continue
            width = np.sqrt((v1.x - v2.x)**2 + (v1.y - v2.y)**2 + (v1.z - v2.z)**2)# * distance
            height = np.sqrt((v3.x - v2.x)**2 + (v3.y - v2.y)**2 + (v3.z - v2.z)**2)# * distance
            
            data = np.array([poseSt.point.x, poseSt.point.y, poseSt.point.z, width, height])
            
            # add point
            if detected_object.type_id in self.MAP:
                self.MAP[detected_object.type_id].appendPoint(data)
            else:
                self.MAP[detected_object.type_id] = SimpleMapObject(detected_object.type_id, detected_object.type_name, data)
                
        for t_object in self.MAP.values():
            t_object.update()
            t_object.validate()            
        
        if self.publish_map:
            self.map_pub_markers.publish(self.map_to_marker_array())
        
        if self.publish_cloud:
            self.cloud_pub.publish(self.map_to_pointcloud2())
            
        self.map_pub.publish(self.map_to_semantic())

    def map_to_semantic(self):
        map_msg = SemanticMap()
        map_msg.header.frame_id = self.map_frame
        map_msg.header.stamp = rospy.Time.now()
        
        for type_id, type_object in self.MAP.items():
            for obj in type_object.clusters + type_object.saved_clusters:
                obj_msg = SemanticObject()
                
                obj_msg.type_name = type_object.type_name
                obj_msg.pose.pose.position.x = obj['centroid'][0]
                obj_msg.pose.pose.position.y = obj['centroid'][1]
                obj_msg.pose.pose.position.z = obj['centroid'][2]
                obj_msg.pose.pose.orientation.w = 1
                
                obj_msg.accepted = False
                
                obj_msg.diameter = obj['width']
                obj_msg.height = obj['height']
                
                if obj['cov'] is not None:
                    #obj_msg.pose.covariance = obj['cov'].flatten().tolist()
                    cov = np.zeros(36)
                    cov[0] = obj['cov'][0,0] # xx
                    cov[1] = obj['cov'][0,1] # xy
                    cov[2] = obj['cov'][0,2] # xz
                    cov[6] = obj['cov'][1,0] # yx
                    cov[7] = obj['cov'][1,1] # yy
                    cov[8] = obj['cov'][1,2] # yz
                    cov[12] = obj['cov'][2,0] # zx
                    cov[13] = obj['cov'][2,1] # zy
                    cov[14] = obj['cov'][2,2] # zz
                    obj_msg.pose.covariance = cov.tolist()
                
                obj_msg.number_observations = obj['size']
                
                map_msg.objects.append(obj_msg)
                
            for obj in type_object.saved_clusters:
                obj_msg = SemanticObject()
                
                obj_msg.type_name = type_object.type_name
                obj_msg.pose.pose.position.x = obj['centroid'][0]
                obj_msg.pose.pose.position.y = obj['centroid'][1]
                obj_msg.pose.pose.position.z = obj['centroid'][2]
                obj_msg.pose.pose.orientation.w = 1
                
                obj_msg.accepted = True
                
                obj_msg.diameter = obj['width']
                obj_msg.height = obj['height']
                
                if obj['cov'] is not None:
                    #obj_msg.pose.covariance = obj['cov'].flatten().tolist()
                    cov = np.zeros(36)
                    cov[0] = obj['cov'][0,0] # xx
                    cov[1] = obj['cov'][0,1] # xy
                    cov[2] = obj['cov'][0,2] # xz
                    cov[6] = obj['cov'][1,0] # yx
                    cov[7] = obj['cov'][1,1] # yy
                    cov[8] = obj['cov'][1,2] # yz
                    cov[12] = obj['cov'][2,0] # zx
                    cov[13] = obj['cov'][2,1] # zy
                    cov[14] = obj['cov'][2,2] # zz
                    obj_msg.pose.covariance = cov.tolist()
                
                obj_msg.number_observations = obj['size']
                
                map_msg.objects.append(obj_msg)
                        
        return map_msg
    
    def map_to_marker_array(self):
        global mh_thres
        markers_msg = MarkerArray()        
        
        cluster_color = [1, 1, 0] # yellow
        saved_cluster_color = [1, 0.5, 0] # orange
        
        
        counter = 0
        for type_id, type_object in self.MAP.items():
            for i, obj in enumerate(type_object.clusters + type_object.saved_clusters):
                #
                # cylinders
                #
                marker_msg = Marker()
                
                marker_msg.header.frame_id = self.map_frame
                marker_msg.header.stamp = rospy.Time.now()
                marker_msg.id = counter
                marker_msg.action = Marker.ADD
                marker_msg.type = Marker.CYLINDER
                marker_msg.lifetime = rospy.Duration(0)
                
                marker_msg.scale.x = obj['width']
                marker_msg.scale.y = obj['width']
                marker_msg.scale.z = obj['height']
                
                if i < len(type_object.clusters):
                    # not saved
                    marker_msg.color.r = cluster_color[0]
                    marker_msg.color.g = cluster_color[1]
                    marker_msg.color.b = cluster_color[2]
                    marker_msg.color.a = 0.5                     
                else:
                    # saved
                    marker_msg.color.r = saved_cluster_color[0]
                    marker_msg.color.g = saved_cluster_color[1]
                    marker_msg.color.b = saved_cluster_color[2]
                    marker_msg.color.a = 0.5                   
                
                marker_msg.pose.position.x = obj['centroid'][0]
                marker_msg.pose.position.y = obj['centroid'][1]
                marker_msg.pose.position.z = obj['centroid'][2]
                
                marker_msg.pose.orientation.w = 1
                
                markers_msg.markers.append(marker_msg)
                counter+=1
                
                #
                # text
                #
                marker_msg = Marker()
        
                marker_msg.header.frame_id = self.map_frame
                marker_msg.header.stamp = rospy.Time.now()
                marker_msg.id = counter 
                counter+=1
                            
                marker_msg.action = Marker.ADD
                marker_msg.type = Marker.TEXT_VIEW_FACING
                marker_msg.lifetime = rospy.Duration(0)
                
                marker_msg.pose.orientation.w = 1                
                marker_msg.scale.z = 0.5
                
                marker_msg.pose.position.x = obj['centroid'][0]
                marker_msg.pose.position.y = obj['centroid'][1]
                marker_msg.pose.position.z = obj['centroid'][2] + 0.5 * obj['height'] + marker_msg.scale.z
                                            
                marker_msg.text = type_object.type_name
                
                #if obj in type_object.saved_clusters:
                if i < len(type_object.clusters):
                    marker_msg.color.r = cluster_color[0]
                    marker_msg.color.g = cluster_color[1]
                    marker_msg.color.b = cluster_color[2]
                    marker_msg.color.a = 1     
                else:
                    marker_msg.color.r = saved_cluster_color[0]
                    marker_msg.color.g = saved_cluster_color[1]
                    marker_msg.color.b = saved_cluster_color[2]
                    marker_msg.color.a = 1
                    
                markers_msg.markers.append(marker_msg)
                
                #
                # covariance
                #
                eig_val, eig_vec = np.linalg.eig(obj['cov'][0:2,0:2])
                
                if eig_val[0] >= eig_val[1]:
                    big_ind = 0
                    small_ind = 1
                else:
                    big_ind = 1
                    small_ind = 0
                    
                try:
                    a = np.sqrt(eig_val[big_ind])
                except ValueError:
                    a = 0
                try:
                    b = np.sqrt(eig_val[small_ind])
                except ValueError:
                    b = 0
                    
                angle = np.arctan2(eig_vec[big_ind, 1], eig_vec[big_ind, 0])
        
                marker_msg = Marker()
                
                marker_msg.header.frame_id = self.map_frame
                marker_msg.header.stamp = rospy.Time.now()
                marker_msg.id = counter
                marker_msg.action = Marker.ADD
                marker_msg.type = Marker.CYLINDER
                marker_msg.lifetime = rospy.Duration(0)
                
                marker_msg.scale.x = a * mh_thres
                marker_msg.scale.y = b * mh_thres
                marker_msg.scale.z = 0.01
                
                #if obj in type_object.saved_clusters:
                if i < len(type_object.clusters):
                    marker_msg.color.r = cluster_color[0]
                    marker_msg.color.g = cluster_color[1]
                    marker_msg.color.b = cluster_color[2]
                    marker_msg.color.a = 0.5     
                else:
                    marker_msg.color.r = saved_cluster_color[0]
                    marker_msg.color.g = saved_cluster_color[1]
                    marker_msg.color.b = saved_cluster_color[2]
                    marker_msg.color.a = 0.5   
                
                marker_msg.pose.position.x = obj['centroid'][0]
                marker_msg.pose.position.y = obj['centroid'][1]
                marker_msg.pose.position.z = obj['centroid'][2]
                
                marker_msg.pose.orientation = quaternion_msg_from_yaw(-angle)
                
                markers_msg.markers.append(marker_msg)
                counter+=1
        
        return markers_msg
    
    def map_to_pointcloud2(self):
        header = Header()
        header.frame_id = self.map_frame
        header.stamp = rospy.Time.now()
        
        points = []
        for type_id, type_object in self.MAP.items():            
            for i in range(type_object.all_poses.shape[0]):
                points.append([type_object.all_poses[i,0], type_object.all_poses[i,1], type_object.all_poses[i,2]])
        
        pointcloud_msg = point_cloud2.create_cloud(header, self.fields, points)
        return pointcloud_msg
        
    def dictionary_to_map(self, loaded_dict):
        LMAP = {}
        for type_id, type_object in loaded_dict.items():
            simple_map_object = SimpleMapObject(type_object["type_id"], type_object["type_name"], None )                        
            simple_map_object.saved_clusters = []
            for l_cluster in type_object["saved_clusters"]:
                cluster = {}
                cluster["cov"] = np.array(l_cluster["cov"]).reshape((3,3))
                cluster["centroid"] = np.array(l_cluster["centroid"])
                
                cluster["size"] = l_cluster["size"]
                cluster["height"] = l_cluster["height"]
                cluster["width"] = l_cluster["width"]
                
                cluster["mark"] = False
                
                simple_map_object.saved_clusters.append(cluster)
            
            LMAP[type_id] = simple_map_object
        return LMAP
        
    def map_to_full_dictionary(self):
        MAP_DICT = {}
        for type_id, type_object in self.MAP.items():
            simple_map_object = {}
            simple_map_object["type_id"] = type_object.type_id
            simple_map_object["type_name"] = type_object.type_name
            simple_map_object["saved_clusters"] = []
            for cluster in type_object.saved_clusters:
                listed_cluster = {}
                listed_cluster["cov"] = cluster["cov"].flatten().tolist()
                listed_cluster["centroid"] = cluster["centroid"].tolist()
                listed_cluster["size"] = int(cluster["size"])
                listed_cluster["height"] = float(cluster["height"])
                listed_cluster["width"] = float(cluster["width"])
                
                simple_map_object["saved_clusters"].append(listed_cluster)
            
            MAP_DICT[type_id] = simple_map_object
        return MAP_DICT        
        
    def map_to_file(self):                
        with open(self.map_file, 'w') as outfile:
            fd = self.map_to_full_dictionary()
            rospy.loginfo("[simple_object_mapper] map saved to {}.".format(self.map_file))
            yaml.dump(fd, outfile)

    def save_map_srv(self, req):
        self.map_to_file()
        return []
    
    def clear_map_srv(self, req):
        if self.publish_map:
            self.clear_markers()            
        self.MAP = {}
        return []
    
    def clear_markers(self):
        markers_msg = MarkerArray()   
        marker_msg = Marker()
        marker_msg.action = Marker.DELETEALL
        markers_msg.markers.append(marker_msg)
        self.map_pub_markers.publish(markers_msg)
                    
    def run(self):
        rospy.spin()        





