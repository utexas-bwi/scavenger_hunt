#ifndef BWI_SCAVENGER_GLOBAL_TOPICS_H
#define BWI_SCAVENGER_GLOBAL_TOPICS_H

const char TPC_TASK_COMPLETE[] = "/bwi_scavenger/task_complete";

const char TPC_MOVE_NODE_STOP[] = "/bwi_scavenger/move_node/stop";
const char TPC_MOVE_NODE_GO[] = "/bwi_scavenger/move_node/go";
const char TPC_MOVE_NODE_FINISHED[] = "/bwi_scavenger/move_node/finished";
const char TPC_MOVE_NODE_REQUEST_POSE[] = "/bwi_scavenger/move_node/request_pose";
const char TPC_MOVE_NODE_ROBOT_POSE[] = "/bwi_scavenger/move_node/robot_pose";

const char TPC_YOLO_NODE_TARGET_SEEN[] = "/bwi_scavenger/yolo_node/target_seen";
const char TPC_YOLO_NODE_TARGET_IMAGE[] = "/bwi_scavenger/yolo_node/target_image";
const char TPC_YOLO_NODE_TARGET[] = "/bwi_scavenger/yolo_node/target";

const char TPC_MAIN_NODE_TASK_START[] = "/bwi_scavenger/main_node/task_start";

const char TPC_PERCEPTION_NODE_MOMENT[] = "/bwi_scavenger/perception_node/moment";

const char TPC_DATABASE_NODE_UPDATE_PROOF[] = "/bwi_scavenger/database_node/update_proof";
const char TPC_DATABASE_NODE_DONE_PARSE[] = "/bwi_scavenger/database_node/done_parse";
const char TPC_DATABASE_NODE_GET_INFO[] = "/bwi_scavenger/database_node/get_info";
const char TPC_DATABASE_NODE_INCORRECT[] = "/bwi_scavenger/database_node/incorrect";


#endif
