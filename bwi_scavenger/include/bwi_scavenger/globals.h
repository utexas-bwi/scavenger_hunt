#ifndef GLOBALS_H
#define GLOBALS_H

#include <string>

/*------------------------------------------------------------------------------
GENERAL
------------------------------------------------------------------------------*/

const std::string PROOF_DATABASE_FNAME = "BWI_SCAVENGER_PROOF_DB.dat";
const std::string PROOF_MATERIAL_FNAME = "BWI_SCAVENGER_IMAGE_PROOF.jpeg";
const std::string PROOF_MATERIALS_REPO_DNAME = "proofs";

/*------------------------------------------------------------------------------
NODES
------------------------------------------------------------------------------*/

const char* NODE_FIND_OBJECT = "find_object_node";
const char* NODE_CONCLUDE = "conclude_node";

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
const char TPC_DATABASE_NODE_LOCATIONS[] = "/bwi_scavenger/database_node/locations";

const char TPC_DARKNET_NODE_ADD_TRAINING_FILE[] = "/bwi_scavenger/darknet_node/add_training_file";
const char TPC_DARKNET_NODE_START_TRAINING[] = "/bwi_scavenger/darknet_node/start_training";

const char TPC_TRANSFER_NODE_SEND_FILE[] = "/bwi_scavenger/transfer_node/send_file";


/*------------------------------------------------------------------------------
SERVICES
------------------------------------------------------------------------------*/

const char SRV_POSE_REQUEST[] = "/bwi_scavenger/services/pose_request";
const char SRV_DATABASE_INFO_REQUEST[] = "/bwi_scavenger/services/database_info_request";

#endif
