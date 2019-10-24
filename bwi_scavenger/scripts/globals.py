
TPC_DARKNET_NODE_ADD_TRAINING_FILE = '/bwi_scavenger/darknet_node/add_training_file'
TPC_DARKNET_NODE_START_TRAINING = '/bwi_scavenger/darknet_node/start_training'

TPC_TRANSFER_NODE_SEND_FILE = "/bwi_scavenger/transfer_node/send_file"

TPC_OBJMEM_NODE_ADD = "/bwi_scavenger/objmem_node/add"

SRV_PROOFDB_NODE_SEND_PROOF = "/bwi_scavenger/services/send_proof"
SRV_CONFIRM_OBJECT = "/bwi_scavenger/services/confirm_object"
SRV_GET_PRIORITY_POINTS = "/bwi_scavenger/services/get_priority_points"
SRV_OBJMEM_DUMP = "/bwi_scavenger/services/objmem_dump"
SRV_GET_NEXT_LOCATION = "/bwi_scavenger/services/get_next_location"
SRV_SAVE_WORLD = "/bwi_scavenger/services/save_world"

# This is a really important variable!
#
# When CLUSTER_ANY is true, the status clustering process will include EVERY
# proof in its priority assessment, regardless of status. It will also weigh
# correct, incorrect, and unverified proofs the same when scoring clusters.
#
# When CLUSTER_ANY if false, only verified proofs are clustered, and
# FEAR_OF_FAILURE is enforced.
CLUSTER_ANY = True
