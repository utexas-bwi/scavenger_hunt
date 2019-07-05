#ifndef GLOBALS_H
#define GLOBALS_H

#include <string>

/*------------------------------------------------------------------------------
GENERAL
------------------------------------------------------------------------------*/

const std::string PROOF_DATABASE_PATH = "/home/bwilab/scavenger_hunt/BWI_SCAVENGER_PROOF_DB.dat";
const std::string PROOF_MATERIAL_PATH = "/home/bwilab/scavenger_hunt/BWI_SCAVENGER_IMAGE_PROOF.jpeg";

/*------------------------------------------------------------------------------
NODES
------------------------------------------------------------------------------*/

const char NODE_FIND_OBJECT = "find_object_node";
const char NODE_CONCLUDE = "conclude_node";

/*------------------------------------------------------------------------------
TASKS
------------------------------------------------------------------------------*/

const std::string TASK_FIND_OBJECT = "Find Object";
const std::string TASK_CONCLUDE = "Conclude";

/*------------------------------------------------------------------------------
TOPICS
------------------------------------------------------------------------------*/

const char TPC_TASK_START[] = "/bwi_scavenger/general/task_start";
const char TPC_TASK_END[] = "/bwi_scavenger/general/task_end";

const char TPC_MOVE_NODE_STOP[] = "/bwi_scavenger/move_node/stop";
const char TPC_MOVE_NODE_GO[] = "/bwi_scavenger/move_node/go";
const char TPC_MOVE_NODE_FINISHED[] = "/bwi_scavenger/move_node/finished";

const char TPC_PERCEPTION_NODE_MOMENT[] = "/bwi_scavenger/perception_node/moment";

const char TPC_DATABASE_NODE_UPDATE_PROOF[] = "/bwi_scavenger/database_node/update_proof";
const char TPC_DATABASE_NODE_DONE_PARSE[] = "/bwi_scavenger/database_node/done_parse";
const char TPC_DATABASE_NODE_GET_INFO[] = "/bwi_scavenger/database_node/get_info";
const char TPC_DATABASE_NODE_INCORRECT[] = "/bwi_scavenger/database_node/incorrect";

/*------------------------------------------------------------------------------
SERVICES
------------------------------------------------------------------------------*/

const char SRV_POSE_REQUEST[] = "/bwi_scavenger/services/pose_request";

#endif
