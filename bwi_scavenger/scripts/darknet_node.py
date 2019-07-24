#!/usr/bin/env python
"""A node that manages configuration and training for multiple Darknetworks
mapped by name.

FQPNs for important files are in darknet_paths.py.
"""
import os
import os.path as osp
import paths
import rospy
import shutil
from bwi_scavenger_msgs.msg import DarknetAddTrainingFile
from bwi_scavenger_msgs.msg import DarknetStartTraining
from bwi_scavenger_msgs.msg import DatabaseFile
from darknet_structure import *
from globals import *
from util import Logger
from sensor_msgs.msg import Image


METADATA_PAIRING_DELIMITER = "="
METADATA_VALUES_DELIMITER = ","
METADATA_KEY_NETS = "nets"

log = Logger("darknet_node")
pub_send_file = None
nets = {}


def load_metadata():
    """Parses darknet.dat and rebuilds the Darknetwork containers from disk.
    """
    src = force_open_read(paths.dnnode_meta)

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
    src = open(paths.dnnode_meta, "w")

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


def add_training_file(net_name, image, label, bbox, image_size):
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

    nets[net_name].add_training_file(image, label, bbox, image_size)

    log.info(
        "Added training file with label \"%s\" to network \"%s\""              \
        % (label, net_name)
    )


def add_training_file_cb(msg):
    """Callback for adding a new traiing file.

    Parameters
    ----------
    msg : DarknetAddTrainingFile
        training file metadata
    """
    add_training_file(
        msg.network_name,
        msg.image,
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
    cmd = '%s detector train "%s" "%s"' % (
        paths.dn + "/darknet",
        net.dat_path,
        net.cfg_path,
    )
    # os.system(cmd)

    # Training ended for one reason or another; try to ship it off
    ship_network(net)


def send_file(path, tag):
    """Sends a file via the transfer node.

    Parameters
    ----------
    path : str
        path to file
    tag : str
        file metadata tag (see file_receive.cpp)
    """
    fname = os.path.basename(path)
    f = DatabaseFile()
    f.name = fname
    f.tag = tag
    with open(path, "r") as fin:
        f.data = fin.read()
    pub_send_file.publish(f)


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
    weight_file_path = paths.dn + "/backup/" + net.name + ".weights"

    try:
        send_file(weight_file_path, "dnros_weights")
        send_file(net.cfg_path, "dnros_cfg")
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
    model_yaml_path = paths.dnros + "/config/" + NETWORK_PARAMS_FNAME

    with open(model_yaml_path, "w") as f:
        f.write("yolo_model:\n")
        f.write("  config_file:\n    name: " + net.name + ".cfg\n")
        f.write("  weight_file:\n    name: " + net.name + ".weights\n")
        f.write("  threshold:\n    value: " + str(net.detection_threshold) + "\n")
        f.write("  detection_classes:\n    names:\n")

        for label in net.labels:
            f.write("      - " + label + "\n")

    send_file(model_yaml_path, "dnros_model_yaml")

    # Generate ROS YAML file
    template_path = paths.templates + "/darknet_ros_yaml_template.yaml"
    ros_yaml_path = paths.dnros + "/config/" + ROS_PARAMS_FNAME

    try:
        with open(template_path, "r") as fin:
            with open(ros_yaml_path, "w") as fout:
                for line in fin.readlines():
                    fout.write(
                        line.replace("darknet_ros", "darknet_ros_" + net.name)
                    )

        send_file(ros_yaml_path, "dnros_ros_yaml")
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
    template_path = paths.templates + "/darknet_launch_template.launch"

    try:
        with open(template_path, "r") as f:
            template_data = f.read()

            for key in template_blanks:
                template_data = template_data.replace(key, template_blanks[key])

            launch_path = paths.dnros + "/launch/" + LAUNCH_FNAME

            f = open(launch_path, "w")
            f.write(template_data)
            f.close()

            send_file(launch_path, "dnros_launch")
    except IOError:
        log.err("Failed to generate launch file. Aborting network ship.")
        return

    log.info('Network "' + net.name + '" shipped to darknet_ros!')


if __name__ == "__main__":
    rospy.init_node(log.tag_name)
    load_metadata()
    log.info("Standing by.")

    pub_send_file = rospy.Publisher(
        TPC_TRANSFER_NODE_SEND_FILE,
        DatabaseFile,
        queue_size=1
    )

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

    # a = DarknetAddTrainingFile()
    # b = Image()
    # b.encoding = "bgr8"
    # a.image = b
    # a.network_name = "Find Object"
    # a.label = "can"
    # a.xmin = 0
    # a.xmax = 10
    # a.ymin = 0
    # a.ymax = 10
    # a.image_width = 100
    # a.image_height = 100
    # add_training_file_cb(a)
    #
    # a = DarknetStartTraining()
    # a.network_name = "Find Object"
    # start_training_cb(a)

    rospy.spin()
