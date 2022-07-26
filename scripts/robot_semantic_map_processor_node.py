#!/usr/bin/env python3
from object_spatial_tools_ros.robot_semantic_map_processor import SimpleClusterObjectMapper

if __name__ == '__main__' :
    scom = SimpleClusterObjectMapper()
    scom.run() 
