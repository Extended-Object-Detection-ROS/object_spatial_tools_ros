import xml.etree.cElementTree as ET
import yaml

if __name__ == '__main__':
    
    yaml_file_path = '/home/anton/wheelchair_ws/src/wheelchair2/wheelchair2_config/config/localization/maps/gazebo_hall/semantic_map.yaml'
    xml_file_path = '/home/anton/wheelchair_ws/src/wheelchair2/wheelchair2_config/config/localization/maps/gazebo_hall/semantic_map.xml'
    
    with open(yaml_file_path, 'rb') as infile:
        yaml_dict = yaml.load(infile, Loader=yaml.FullLoader)
    
    xml_root = ET.Element("SceneBase")
    
    xml_base = ET.SubElement(xml_root, "Scene", ID="", Name="")
    
    for id, info in yaml_dict.items():
        class_name = info["type_name"]
        for i, so in enumerate(info["saved_clusters"]):
            x = round(so["centroid"][0],3)
            y = round(so["centroid"][1],3)
            z = round(so["centroid"][2],3)
            h = round(so["height"],3)
            r = round(so["width"]/2,3) # ??
            so_xml = ET.SubElement(xml_base, "SimpleObject", Name=f"{class_name}{i}", Class=class_name, x=f"{x}", y=f"{y}", z=f"{z}", h=f"{h}", r=f"{r}")
        
        
    xml_tree = ET.ElementTree(xml_root)
    xml_tree.write(xml_file_path)
