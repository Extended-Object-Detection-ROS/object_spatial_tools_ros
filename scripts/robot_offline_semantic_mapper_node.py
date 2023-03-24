#!/usr/bin/env python3
from object_spatial_tools_ros.robot_offline_semantic_mapper import OfflineSemanticMapper

if __name__ == '__main__' :
    osm = OfflineSemanticMapper()
    osm.run() 
