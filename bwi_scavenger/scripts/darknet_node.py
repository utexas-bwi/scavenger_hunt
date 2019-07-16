#!/usr/bin/env python
"""A node that manages building and training for multiple Darknetworks.
"""
import os
import rospy
from bwi_scavenger_msgs.msg import DarknetAddTrainingFile
from bwi_scavenger_msgs.msg import DarknetStartTraining
from darknet_paths import *
from darknet_structure import *
from topics import *
from util import Logger


METADATA_FILE_PATH = os.path.join(DARKNET_METADATA_LOCATION,
                                  DARKNET_METADATA_FILENAME)

METADATA_PAIRING_DELIMITER = '='
METADATA_VALUES_DELIMITER = ','

METADATA_KEY_NETS = 'nets'

log = Logger('darknet_node')
nets = {}


def load_metadata():
    """Loads network metadata from disk.
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
                log.info("Parsed network \"%s\" with %s labels and %s training files." % (net.name, len(net.labels), net.training_file_count))

    src.close()

def update_metadata():
    """Updates the metadata file with network names, etc.
    """
    src = open(METADATA_FILE_PATH, 'w')

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
    """
    net = Darknetwork(name)
    nets[name] = net
    update_metadata()

def add_training_file(net_name, src_path, label):
    """Adds a training file to a network of some name. The network is created
    if it doesn't exist.
    """
    if net_name not in nets:
        add_network(net_name)

    nets[net_name].add_training_file(src_path, label)

def add_training_file_cb(msg):
    """Callback for adding a new traiing file.
    """
    add_training_file(msg.network_name, msg.file_path, msg.label)

def start_training_cb(msg):
    """Callback for kicking off training for some network.
    """
    if msg.network_name not in nets:
        log.err("Client tried to train nonexistent network: " +                \
                msg.network_name)
        return
    else:
        log.info("Training \"" + msg.network_name + "\"...")

    net = nets[msg.network_name]
    cmd = '%s classifier train %s %s' %                                        \
          (os.path.join(DARKNET_BIN_LOCATION, 'darknet'),
           net.dat_path,
           net.cfg_path)
    os.system(cmd)


if __name__ == '__main__':
    rospy.init_node(log.tag_name)
    log.info("Loading metadata...")
    load_metadata()
    log.info("Standing by.")

    rospy.Subscriber(TPC_DARKNET_NODE_ADD_TRAINING_FILE,
                     DarknetAddTrainingFile,
                     add_training_file_cb)

    rospy.Subscriber(TPC_DARKNET_NODE_START_TRAINING,
                     DarknetStartTraining,
                     start_training_cb)

    a = DarknetStartTraining()
    a.network_name = 'cifar'
    start_training_cb(a)

    rospy.spin()
