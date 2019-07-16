import os
import shutil
from darknet_paths import *


def force_open_read(fname):
    """Creates a file if not already in existence and opens it for reading.
    """
    try:
        src = open(fname, 'r')
    except IOError:
        src = open(fname, 'w')
        src.close()
        src = open(fname, 'r')

    return src


class Darknetwork:
    """Wrapper for an on-disk Darknet network configuration.
    """
    def __init__(self, name):
        self.name = name

        # Directories
        self.data_path = os.path.join(DARKNET_BIN_LOCATION, 'data', name)
        self.train_path = os.path.join(self.data_path, 'train')

        self.populate()

        # Metadata files
        self.cfg_path = os.path.join(DARKNET_BIN_LOCATION, 'cfg', name + '.cfg')
        self.labels_path = os.path.join(self.data_path, 'labels.txt')
        self.train_list_path = os.path.join(self.data_path, 'train.list')

        self.labels = []

        # Get the number of training files already logged for numbering purposes
        src = force_open_read(self.train_list_path)
        self.training_file_count = len(src.readlines())
        src.close()

        self.load_labels()

    def populate(self):
        """Creates directories for storing network files if not already in
        existence.
        """
        dirs = [
            self.data_path,
            self.train_path
        ]

        for dir in dirs:
            try:
                os.mkdir(dir)
            except OSError:
                pass

    def load_labels(self):
        """Loads labels from labels.txt into local storage.
        """
        src = force_open_read(self.labels_path)
        self.labels = [line.strip() for line in src.readlines()]
        src.close()

    def add_label(self, label):
        """Adds a new label to the network if not already in existence.
        """
        if label not in self.labels:
            self.labels.append(label)
            src = open(self.labels_path, 'w')
            src.write(label + "\n")
            src.close()

    def add_training_file(self, src_path, label):
        """Adds a new training file to the network. New labels are handled.
        """
        # Copy file to train directory
        self.add_label(label)
        root, ext = os.path.splitext(src_path)
        fname = str(self.training_file_count) + "_" + label + ext
        fname_qual = os.path.join(self.train_path, fname)
        shutil.copyfile(src_path, fname_qual)
        self.training_file_count += 1

        # Log in train list
        src = open(self.train_list_path, 'a')
        src.write(fname_qual  + '\n')
        src.close()
