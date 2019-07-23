"""Wrappers for conveniently editing Darknet network configurations.
"""
import os
import paths
import shutil
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image

def force_open_read(fname):
    """Creates a file if not already in existence and opens it for reading.

    Parameters
    ----------
    fname : str
        file path
    """
    try:
        src = open(fname, "r")
    except IOError:
        src = open(fname, "w")
        src.close()
        src = open(fname, "r")

    return src


class Darknetwork:
    """Wrapper for an on-disk Darknet network configuration.
    """
    def __init__(self, name):
        """Creates a new network and populates it with metadata found on disk if
        it could be found.

        Parameters
        ----------
        name : str
            unique identifying name
        """
        self.name = name

        # Directories
        self.data_path = paths.dn + "/data/" + name
        self.train_path = self.data_path + "/train"
        self.test_path = self.data_path + "/test"

        self.populate()

        # Metadata files
        self.cfg_path = paths.dn + "/cfg/" + name + ".cfg"
        self.dat_path = paths.dn + "/cfg/" + name + ".dat"
        self.backup_path = paths.dn + "/backup"
        self.labels_path = self.data_path + "/labels.txt"
        self.train_list_path = self.data_path + "/train.list"
        self.test_list_path = self.data_path + "/test.list"

        self.labels = []
        self.detection_threshold = 0.3
        self.top_accuracy = 2

        # Image
        self.bridge = CvBridge()

        # Get the number of training files already logged for numbering purposes
        src = force_open_read(self.train_list_path)
        self.training_file_count = len(src.readlines())
        src.close()

        self.load_labels()
        self.make_dat()

    def make_dat(self):
        """Generates the .dat file used to convey network metadata to Darknet.
        """
        src = open(self.dat_path, 'w')
        contents = [
            "classes=" + str(len(self.labels)),
            "train=" + self.train_list_path,
            "valid=" + self.test_list_path,
            "labels=" + self.labels_path,
            "backup=" + self.backup_path,
            "top=" + str(self.top_accuracy)
        ]

        for line in contents:
            src.write(line + '\n')

    def populate(self):
        """Creates the directories required in the main Darknet directory
        used to store files for this network.
        """
        dirs = [
            self.data_path,
            self.train_path,
            self.test_path
        ]

        for dir in dirs:
            try:
                os.mkdir(dir)
            except OSError:
                pass

    def load_labels(self):
        """Loads labels from this network's labels.txt into local structures.
        """
        src = force_open_read(self.labels_path)
        self.labels = [line.strip() for line in src.readlines()]
        src.close()

    def add_label(self, label):
        """Adds a new label to the network if not already in existence.

        Parameters
        ----------
        label : str
            label
        """
        if label not in self.labels:
            self.labels.append(label)
            src = open(self.labels_path, 'w')
            src.write(label + "\n")
            src.close()

    def add_training_file(self, image, label, bbox, image_size):
        """Adds a new training file to the network. New labels are handled.

        Parameters
        ----------
        src_path : str
            path to training file
        label : str
            correct label for training file
        bbox : 4-tuple
            bounding box corners (xmin, xmax, ymin, ymax)
        image_size : 2-tuple
            image dimensions (width, height)
        """
        fname_stem = str(self.training_file_count) + "_" + label

        # Copy file to train directory
        self.add_label(label)
        # root, ext = os.path.splitext(src_path)
        # fname = fname_stem + ext

        fname = fname_stem + '.png'
        fname_qual = os.path.join(self.train_path, fname)
        # shutil.copyfile(src_path, fname_qual)
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        cv2.imwrite(fname_qual, cv_image)

        self.training_file_count += 1

        # Log in train list
        src = open(self.train_list_path, 'a')
        src.write(fname_qual  + '\n')
        src.close()

        # Generate annotation file
        annotation = open(
            os.path.join(self.train_path, fname_stem + ".txt"), "w"
        )

        xmin, xmax, ymin, ymax = bbox
        image_width, image_height = image_size
        x = (xmin + xmax) / 2
        y = (ymin + ymax) / 2
        width = xmax - xmin
        height = ymax - ymin
        class_index = self.labels.index(label)

        annotation.write(
            "%s %s %s %s %s\n" % (
                class_index,
                x * 1.0 / image_width,
                y * 1.0 / image_height,
                width * 1.0 / image_width,
                height * 1.0 / image_height
            )
        )

        annotation.close()
