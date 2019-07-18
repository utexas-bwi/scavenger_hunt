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


def clean_name(name):
    """Purges a network name of illegal characters.

    Parameters
    ----------
    name : str
        name to clean

    Return
    ------
    str
        clean name
    """
    return name.replace(" ", "_")


def add_network(name):
    """Adds a new network to the database.

    Parameters
    ----------
    name : str
        network name
    """
    # Replace spaces to avoid path problems
    name = clean_name(name)
    net = Darknetwork(name)
    nets[name] = net
    update_metadata()


def add_training_file(net_name, src_path, label, bbox, image_size):
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
    bbox : 4-tuple
        bounding box corners (xmin, xmax, ymin, ymax)
    image_size : 2-tuple
        image dimensions (width, height)
    """
    net_name = clean_name(net_name)

    if net_name not in nets:
        add_network(net_name)

    nets[net_name].add_training_file(src_path, label, bbox, image_size)


def add_training_file_cb(msg):
    """Callback for adding a new traiing file.

    Parameters
    ----------
    msg : DarknetAddTrainingFile
        training file metadata
    """
    add_training_file(
        msg.network_name,
        msg.file_path,
        msg.label,
        (msg.xmin, msg.xmax, msg.ymin, msg.ymax),
        (msg.image_width, msg.image_height)
    )


def start_training_cb(msg):
    """Callback for kicking off training for some network.

    Parameters
    ----------
    msg : DarknetStartTraining
        start command
    """
    msg.network_name = clean_name(msg.network_name)

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
    files as well as autogenerating the YAML and launch files needed by
    darknet_ros.

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
            'Could not find network files for "%s". '
            "Possible training abort?"
            % net.name
        )
        return

    NETWORK_PARAMS_FNAME = net.name + ".yaml"
    ROS_PARAMS_FNAME = net.name + "-ros.yaml"
    LAUNCH_FNAME = "darknet_ros_" + net.name + ".launch"

    # Generate model YAML file
    model_yaml_path = osp.join(dnros_path, "config", NETWORK_PARAMS_FNAME)

    with open(model_yaml_path, "w") as f:
        f.write(net.name + "_model:\n")
        f.write("  config_file:\n    name: " + net.name + ".cfg\n")
        f.write("  weight_file:\n    name: " + net.name + ".weights\n")
        f.write("  threshold:\n    value: 0.3\n")
        f.write("  detection_classes:\n    names:\n")

        for label in net.labels:
            f.write("      - " + label + "\n")

    # Generate ROS YAML file
    template_path = osp.join(
        TEMPLATES_LOCATION, "darknet_ros_yaml_template.yaml"
    )
    ros_yaml_path = osp.join(dnros_path, "config", ROS_PARAMS_FNAME)

    try:
        with open(template_path, "r") as fin:
            with open(ros_yaml_path, "w") as fout:
                for line in fin.readlines():
                    fout.write(
                        line.replace("darknet_ros", "darknet_ros_" + net.name)
                    )
    except IOError:
        log.err("Failed to generate ROS YAML file. Aborting network ship.")
        return
    pass

    # Generate launch file
    template_blanks = {
        "{ROS_PARAMS_FNAME}" : ROS_PARAMS_FNAME,
        "{NETWORK_PARAMS_FNAME}" : NETWORK_PARAMS_FNAME,
        "{NODE_NAME}" : "darknet_ros_" + net.name,
        "{WEIGHTS_FNAME}" : net.name + ".weights",
        "{MODEL_FNAME}" : net.name + ".cfg"
    }
    template_path = osp.join(
        TEMPLATES_LOCATION, "darknet_launch_template.launch"
    )

    try:
        with open(template_path, "r") as f:
            template_data = f.read()

            for key in template_blanks:
                template_data = template_data.replace(key, template_blanks[key])

            launch_path = osp.join(dnros_path, "launch", LAUNCH_FNAME)

            f = open(launch_path, "w")
            f.write(template_data)
            f.close()
    except IOError:
        log.err("Failed to generate launch file. Aborting network ship.")
        return

    log.info('Network "' + net.name + '" shipped to darknet_ros!')


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

    # a = DarknetStartTraining()
    # a.network_name = "Find Object"
    # start_training_cb(a)

    rospy.spin()
