#ifndef BWI_SCAVENGER_FIND_OBJECT_TOPICS_H
#define BWI_SCAVENGER_FIND_OBJECT_TOPICS_H

const char TPC_TASK_COMPLETE[] = "/bwi_scavenger/task_complete";

const char TPC_MOVE_NODE_STOP[] = "/bwi_scavenger/move_node/stop";
const char TPC_MOVE_NODE_GO[] = "/bwi_scavenger/move_node/go";
const char TPC_MOVE_NODE_FINISHED[] = "/bwi_scavenger/move_node/finished";
const char TPC_MOVE_NODE_CLOSEST_WAYPOINT[] = "/bwi_scavenger/move_node/closest_waypoint";

const char TPC_YOLO_NODE_TARGET_SEEN[] = "/bwi_scavenger/yolo_node/target_seen";
const char TPC_YOLO_NODE_TARGET_IMAGE[] = "/bwi_scavenger/yolo_node/target_image";
const char TPC_YOLO_NODE_TARGET[] = "/bwi_scavenger/yolo_node/target";

const char TPC_MAIN_NODE_TASK_START[] = "/bwi_scavenger/main_node/task_start";

#endif
