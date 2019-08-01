import rospy


ws = rospy.get_param("bwi_scavenger/ws_path", "/home/bwilab/scavenger_hunt")
dn = ws + "/src/darknet_ros/darknet"
dnros = ws + "/src/darknet_ros/darknet_ros"
dnros_weights = dnros + "/yolo_network_config/weights"
dnros_cfg = dnros + "/yolo_network_config/cfg"
dnnode_meta = ws + "/darknet.dat"
objmem = ws + "/objmem"
templates = ws + "/src/bwi_scavenger/scripts/templates"
