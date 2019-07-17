#!/usr/bin/env python
"""A node that manages configuration and training for multiple Darknetworks
mapped by name.

FQPNs for important files are in darknet_paths.py.
"""
import os
import os.path as osp
import rospy
import shutil
from bwi_scavenger_msgs.msg import DarknetAddTrainingFile
from bwi_scavenger_msgs.msg import DarknetStartTraining
from darknet_paths import *
from darknet_structure import *
from globals import *
from util import Logger


METADATA_FILE_PATH = osp.join(
    DARKNET_METADATA_LOCATION, DARKNET_METADATA_FILENAME
)

METADATA_PAIRING_DELIMITER = "="
METADATA_VALUES_DELIMITER = ","

METADATA_KEY_NETS = "nets"

log = Logger("darknet_node")
nets = {}


def load_metadata():
    """Parses darknet.dat and rebuilds the Darknetwork containers from disk.
    """
    src = force_open_read(METADATA_FILE_PATH)

    for line in src.readlines():
        tokens = line.strip().split(METADATA_PAIRING_DELIMITER)
        key = tokens[0]
        values = [x.strip() for x in tokens[1].split(METADATA_VALUES_DELIMITER)]

        # Create network wrappers
        if key == METADATA_KEY_NETS:
            for value in values:
                net = Darknetwork(value)
                nets[value] = net
                log.info(
                    'Parsed network "%s" with %s labels and %s training '
                    "files."
                    % (
                        net.name,
                        len(net.labels),
                        net.training_file_count,
                    )
                )

    src.close()


def update_metadata():
    """Updates the metadata file with network names, etc.
    """
    src = open(METADATA_FILE_PATH, "w")

    # Update list of network names
    src.write(METADATA_KEY_NETS + METADATA_PAIRING_DELIMITER)
    net_names = nets.keys()
    if len(net_names) > 0:
        src.write(net_names[0])
        for name in net_names[1:]:
            src.write(METADATA_VALUES_DELIMITER + name)

    src.close()


def add_network(name):
    """Adds a new network to the database.

    Parameters
    ----------
    name : str
        network name
    """
    # Replace spaces to avoid path problems
    name_clean = name.replace(" ", "_")
    net = Darknetwork(name_clean)
    nets[name_clean] = net
    update_metadata()


def add_training_file(net_name, src_path, label):
    """Adds a training file to a network of some name. The network is created
    if it doesn't exist.

    Parameters
    ----------
    net_name : str
        name of network to add to
    src_path : str
        file path to training file
    label : str
        training file label
    """
    if net_name not in nets:
        add_network(net_name)

    nets[net_name].add_training_file(src_path, label)


def add_training_file_cb(msg):
    """Callback for adding a new traiing file.

    Parameters
    ----------
    msg : DarknetAddTrainingFile
        training file metadata
    """
    add_training_file(msg.network_name, msg.file_path, msg.label)


def start_training_cb(msg):
    """Callback for kicking off training for some network.

    Parameters
    ----------
    msg : DarknetStartTraining
        start command
    """
    if msg.network_name not in nets:
        log.err(
            "Client tried to train nonexistent network: %s"
            % msg.network_name
        )
        return
    else:
        log.info('Training "%s"...' % msg.network_name)

    # Send system command
    net = nets[msg.network_name]
    cmd = '%s classifier train "%s" "%s"' % (
        osp.join(DARKNET_BIN_LOCATION, "darknet"),
        net.dat_path,
        net.cfg_path,
    )
    os.system(cmd)

    # Training ended for one reason or another; try to ship it off
    ship_network(net)


def ship_network(net):
    """Ships a network to darknet_ros by copying over the .cfg and .weights
    files as well as autogenerating the model YAML file.

    Parameters
    ----------
    net : Darknetwork
        network to ship
    """
    # Copy network config and weights
    dnros_path = osp.join(os.getcwd(), "src", "darknet_ros", "darknet_ros")
    weights_path = osp.join(dnros_path, "yolo_network_config", "weights")
    cfg_path = osp.join(dnros_path, "yolo_network_config", "cfg")
    weight_file_path = osp.join(
        DARKNET_BIN_LOCATION,
        "backup",
        net.name + ".weights",
    )

    try:
        shutil.copyfile(
            weight_file_path,
            osp.join(weights_path, net.name + ".weights"),
        )
        shutil.copyfile(
            net.cfg_path,
            osp.join(cfg_path, net.name + ".cfg"),
        )
    except IOError:
        log.warn(
            'Failed to ship network "%s". Possible training abort?'
            % net.name
        )
        return

    # Generate model YAML file
    yaml_location = osp.join(dnros_path, "config", net.name + ".yaml")
    yaml = open(yaml_location, "w")

    yaml.write(net.name + "_model:\n")
    yaml.write("  config_file:\n    name: " + net.name + ".cfg\n")
    yaml.write("  weight_file:\n    name: " + net.name + ".weights\n")
    yaml.write("  threshold:\n    value: 0.3\n")
    yaml.write("  detection_classes:\n    names:\n")

    for label in net.labels:
        yaml.write("      - " + label + "\n")

    yaml.close()

    log.info(
        'Network "' + net.name + '" shipped to darknet_ros!'
    )


if __name__ == "__main__":
    rospy.init_node(log.tag_name)
    load_metadata()
    log.info("Standing by.")

    rospy.Subscriber(
        TPC_DARKNET_NODE_ADD_TRAINING_FILE,
        DarknetAddTrainingFile,
        add_training_file_cb,
    )
    rospy.Subscriber(
        TPC_DARKNET_NODE_START_TRAINING,
        DarknetStartTraining,
        start_training_cb,
    )
    rospy.spin()
