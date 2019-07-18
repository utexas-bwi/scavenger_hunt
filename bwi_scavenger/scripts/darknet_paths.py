import os


DARKNET_BIN_LOCATION = "/home/bwilab/darknet"
DARKNET_METADATA_LOCATION = "/home/bwilab/scavenger_hunt"
DARKNET_METADATA_FILENAME = "darknet.dat"
METADATA_FILE_PATH = os.path.join(
    DARKNET_METADATA_LOCATION, DARKNET_METADATA_FILENAME
)
BWI_SCAVENGER_LOCATION = os.path.join(os.getcwd(), "src", "bwi_scavenger")
SCRIPTS_LOCATION = os.path.join(BWI_SCAVENGER_LOCATION, "scripts")
TEMPLATES_LOCATION = os.path.join(SCRIPTS_LOCATION, "templates")
