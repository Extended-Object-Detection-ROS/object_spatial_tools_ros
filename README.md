__THIS REPOSITORY IS ON EARLY STAGE OF DEVELOPMENT__

#  object_spatial_tools_ros
Nodes to work with results of [Extended Object Detection node](https://github.com/Extended-Object-Detection-ROS/extended_object_detection).  
All objects should be detected with distance estimation to it.

 - [X] [Short Object Memory](https://github.com/Extended-Object-Detection-ROS/object_spatial_tools_ros#1-robot_short_object_memory_nodepy) - remembers position of detected objects for some period of time
 - [X] [Semantic Map Creator](https://github.com/Extended-Object-Detection-ROS/object_spatial_tools_ros/blob/devel-noetic/README.md#2-robot_semantic_map_processor_nodepy) - marks detected objects on map, estimatin their sizes
 - [X] [Kalman Filter Tracker for Unoriented Objects](https://github.com/Extended-Object-Detection-ROS/object_spatial_tools_ros/blob/devel-noetic/README.md#3-robot_kf_undirected_object_tracker_nodepy) - tracks detected objects without orientation, estimating their speed
 - [ ] Exteneded Kalman Filter for Oriented Objects - tracks detected objects with orientation, estimating their speed


## 1. robot_short_object_memory_node.py
Remembers objects in moving frame for short period of time.  
Simplified algorithm to add new object:  
```mermaid
graph LR
    classDef box fill:#FFFFFF, stroke:#000, stroke-width:2px;

    A[get new detected object]:::box --> B{type exists?}
    B --> |NO|C[add to memory as new]:::box
    C --> J[occurance++, forgotten = false]:::box
    B --> |YES|D[calc match scores, calc thresh]:::box
    D --> E{best match score < thresh}:::box
    E --> |NO|C:::box
    E --> |YES|I[append to best match]:::box
    I --> J:::box
```
Simplified algorithm to update objects:  
```mermaid
graph LR
    classDef box fill:#FFFFFF, stroke:#000, stroke-width:2px;
    
    A{forgotten == true}:::box -->|YES| B{occurance--}
    B --> C[occurance == 0]:::box
    C --> |YES|D[delete obj]:::box
    C --> |NO|E[do nothing]:::box
    A --> |NO|F{now - obj_stamp < forget_time}:::box
    F -->|NO|E:::box
    F -->|YES|I[forgotten = true]:::box
```
### Params
 - __~target_frame__ (string, default: odom) frame for remembered objects
 - __~score_multiplyer__ (double, default: 2) multiplier for score, to check similarity of objects
 - __~update_rate_hz__ (double, default: 5 [hz]) rate of update algorithm (see below)
 - __~forget_time__ (double, default: 10 [sec]) time to remove object if not seen
 - __~update_count_thresh__ (double, default: 0) limit for previous position used for update, if 0 - no limit

### Subscribed topics
- __simple_objects__ (extended_object_detection/SimpleObjectArray) input result of detection
- __complex_objects__ (extended_object_detection/ComplexObjectArray) input result of detection

### Published topics
- __~memory_map__ (visualization_msgs/MarkerArray) visualization of results
- TODO: results itself!

## 2. robot_semantic_map_processor_node.py
Creates an 'semantic map layer' which contains position, names and sizes of objects.

```mermaid
graph TD
  classDef box fill:#FFFFFF, stroke:#000, stroke-width:2px;

  A[new objects recieved]:::box
  A-->B{type exists}:::box

  B --> b{Is min MH dist to saved cluster centroid less thresh?}:::box

  b -->|NO| C[Append point to container unmerged data]:::box
  b -->|YES| c[Add point to saved cluster]:::box
  B -->|NO| D[Create new object container]:::box

  c --> E
  C --> E
  D --> E
  E[Free temp clusters, and recluster all unmerged data]:::box
  E --> F{Cluster size > min_size}:::box
  F -->|YES| G{Cluster size > max_size}:::box
  F -->|NO| H[Ignore it]:::box

  G -->|YES| J[Add cluster to saved clusters, remove its data from unmerged data]:::box
  G -->|NO| K[Add clusters to temp clusters]:::box
  J --> L:::box
  K --> L:::box
  L[Merge intersecting saved clusters]
```

### Params
 - __~map_frame__ (string, default: "map") TF frame, used for mapping.
 - __~map_file__ (string, "/tmp/semantic_map.yaml") Path of map to be loaded.
 - __~clear_map_on_start__ (bool, false) If true creates new map, and _save_semantic_map_ serice will overwrite saved map if exists.
 - __~update_map__ (bool, default: False) If _clear_map_on_start_ is false. If true, loaded map will be updated if exist, else will be crated a new one. If false, that loaded map only will be republished to topics is exist, else programm will be terminated.
 - __~pub_rate_sec__ (float, default: 1.0) If _update_map_ is false, it will be published with such rate.
 - __~publish_map_as_markers__ (bool, default: True) If true marker representation of map is published.
 - __~publish_cloud__ (bool, default: False) If true raw object position is published.
 - __~mh_thres__ (float, default: 3.0) Mahalanobis threshold when new points is added to existing cluster.
 - __~cluster_dist_thres__ (float, default: 3.0) Threshold (in meters?) of distancem which is used to form new cluster by hierarhical clustering.
 - __~cluster_min_size__ (int, default: 10) If cluster size is less, it is ignored.
 - __~cluster_max_size__ (int, default: 100) When cluster size overgrown that value, it is saved and only its params are updated.

### Subscibed topics
- __detected_objects__ (extended_object_detection/SimpleObjectArray) Detected objects to be mapped. If _update_map_ is false, node doesn't subscribe to it.

### Published topics
- __~semantic_map__ (object_mapping/SematicMap) Full map information for external usage.
- __~semantic_object_map_as_markers__ (visualiation_msgs/MarkerArray) Map represented for rviz visualization, only published if _publish_map_as_markers_ param is set.

### Provided services
- __~save_semantic_map__ (std_srvs/Empty) Updates map on it's path, or saves new one if not exist.
- __~clear_map__ (std_srvs/Empty) Fully clears existed map.

## 3. robot_kf_undirected_object_tracker_node.py
Tracks visually detected objects in 2d space. Works with unoriented objects. Kalman Filter estimates `x,y, vx, vy` parameters.  
Simplified algorithm to add new object:  
```mermaid
graph LR
    classDef box fill:#FFFFFF, stroke:#000, stroke-width:2px;
    
    A[get new object]:::box --> B{type exists?}:::box
    
    Z[Reject object]:::box
    C[start new KF]:::box   
    D[calc mahalanobis]:::box
    E{min maxalanobis < thresh}:::box
    e{score > min_score}:::box
    f{score > min_score_soft}:::box

    B -->|NO|e:::box
    B --> |YES|D:::box
    D --> E:::box
    E --> |NO|e:::box
    e --> |YES|C:::box
    E --> |YES|f:::box
    f --> |YES|F[update KF with object]:::box
    f --> |NO|Z:::box
    e --> |NO|Z:::box
```
Simplified algorithm to handle existing filters:  
```mermaid
graph LR
    classDef box fill:#FFFFFF, stroke:#000, stroke-width:2px;
    
    A{lifetime > now - last_update} --> |YES|B[remove KF]:::box
    A --> |NO|C[predict KF]:::box
```
### Params
 - __~target_frame__ (string, default: odom) frame for tracking
 - __~tf_pub_prefix__ (string, default: "") is set, prefix will be added to broadcasted tf frames
 - __~tracked_objects_type_names__ (list, default: []) object names from object base to track
 - __~Qdiag__ (list, default: [0.1, 0.1, 0.1, 0.1]) diagonal values of Q matrix
 - __~Rdiag__ (list, default: [0.1, 0.1]) diagonale values of R matrix
 - __~k_decay__ (double, default: 1) track speed reducer, new step speed will be `k * speed_prev`
 - __~lifetime__ (double, default: 0) how long to perform tracking when objects disappears, if 0 - infinite
 - __~mahalanobis_max__ (double, default: 1) Mahalanobis dist when new object might be added to existing track
 - __~update_rate_hz__ (double, default: 5 [hz]) rate of tracker
 - __~min_score__ (double, default: 0.0) threshold for score of detected objects
 - __~min_score_soft__ (double, default: __~min_score__) threshold for soft-mode traking, to disable set >= __~min_score__
 
### Subscribed topics
- __simple_objects__ (extended_object_detection/SimpleObjectArray) input result of detection
- __complex_objects__ (extended_object_detection/ComplexObjectArray) input result of detection

### Published topics
- __~vis__ (visualization_msgs/MarkerArray) visualization of results
- __~tracked_objects__ ([object_spatial_tools_ros/TrackedObjectArray](https://github.com/Extended-Object-Detection-ROS/object_spatial_tools_ros/blob/devel-noetic/msg/TrackedObjectArray.msg)) out results, containg poses and speeds
