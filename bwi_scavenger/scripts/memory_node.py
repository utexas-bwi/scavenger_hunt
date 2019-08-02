#!/usr/bin/env python
"""memory_node provides utilities for managing databases of task data. At the
moment there are two databases: submitted proofs and rigid objects. These
databases hold metadata used in task learning.

The node provides a proxy service for scavenger_hunt_msgs::SendProof with the
added functionality of recording the proof in the database and drawing
connections between objects submitted as proof and objects in the rigid object
database. These connections, once verified via website feedback, are used to
train darknet detectors on task-specific entities.
"""
import cv2
import enum
import objmem
import os
import paths
import rospy
import status_clustering

import scavenger_hunt_msgs.srv
import bwi_scavenger_msgs.srv

from bwi_scavenger_msgs.msg import ObjmemAdd, DarknetAddTrainingFile
from cv_bridge import CvBridge, CvBridgeError
from darknet_structure import force_open_read
from geometry_msgs.msg import Point
from globals import *
from scavenger_hunt import ProofStatus
from scavenger_hunt_msgs.msg import Hunt, Login, Parameter, Proof, Task
from sensor_msgs.msg import Image
from util import Logger
from visualization_msgs.msg import Marker


log = Logger("memory_node")
bridge = CvBridge()
scav_send_proof = None
proof_db = []
pub_add_training_file = None
pub_marker = None


class ProofDbEntry:
    """Metadata about a proof submitted to the website.
    """
    def __init__(
        self,
        id=0,
        task_name="",
        parameter="",
        robot_position=[0, 0, 0],
        status=ProofStatus.UNVERIFIED,
        objmem_id=-1):
        """
        Parameters
        ----------
        id: int
            proof ID received from the upload process
        task_name: str
            name of the task for which this proof was submitted
        parameter: str
            parameter of the task for which this proof was submitted
        robot_position: 3-tuple of float
            (x, y, z) of the robot at the time of task completion
        status: ProofStatus
            status of the proof; updates on database load
        objmem_id: int
            (optional) ID of associated object in objmem
        """
        self.id = id
        self.task_name = task_name
        self.parameter = parameter
        self.robot_position = robot_position
        self.status = status
        self.objmem_id = objmem_id

    def __str__(self):
        """To-string override.
        """
        return "(%s, %s, %s, %s, %s, %s)" % (
            self.id,
            self.task_name,
            self.parameter,
            self.robot_position,
            self.status,
            self.objmem_id
        )


################################################################################
# DATABASE MANAGEMENT
################################################################################


def send_proof(req):
    """Sends a proof to scavenger_hunt_node for upload and logs this action in
    the proof database.

    Parameters
    ----------
    req: bwi_scavenger_msgs.SendProofRequest
        upload request message
    """
    log.info("Delegating proof upload to scavenger_hunt_node...")

    # Create a scavenger_hunt_msgs upload request and get the response
    req2 = scavenger_hunt_msgs.srv.SendProofRequest()
    req2.proof = req.proof
    req2.task = req.task
    res = scav_send_proof(req2)

    log.info("Response received. Proof ID: %s" % res.id)

    # Create and log the new database entry
    param = None if len(req.task.parameters) == 0 else req.task.parameters[0].value
    entry = ProofDbEntry(
        res.id,
        req.task.name,
        param,
        (req.robot_position.x, req.robot_position.y, req.robot_position.z),
        ProofStatus.UNVERIFIED,
        -1
    )

    # Find_Object -- Attempt object memorization
    if req.task.name == "Find Object":
        add_msg = ObjmemAdd()
        add_msg.label = param
        add_msg.x = req.secondary_position.x
        add_msg.y = req.secondary_position.y
        add_msg.z = req.secondary_position.z
        add_msg.image = req.proof.image
        bbox = [int(x) for x in req.metadata.split(",")]
        add_msg.xmin = bbox[0]
        add_msg.xmax = bbox[1]
        add_msg.ymin = bbox[2]
        add_msg.ymax = bbox[3]
        uid = objmem_add(add_msg)
        entry.objmem_id = uid if uid is not None else -1

    proof_db.append(entry)

    # Reload the database
    save_proof_db()

    # Return the proof ID
    res2 = bwi_scavenger_msgs.srv.SendProofResponse()
    res2.id = res.id
    return res2


# def objmem_verify(msg):
#     status = None
#
#     # Determine status of the object's proof
#     if not msg.verified:
#         status = objmem.ProofStatus.UNVERIFIED
#     elif msg.correct:
#         status = objmem.ProofStatus.CORRECT
#     else:
#         status = objmem.ProofStatus.INCORRECT
#
#     # Locate the object in the db
#     for obj in objmem.bank:
#         if obj.uid == msg.uid:
#             # If the status changed to CORRECT, we need to send some training
#             # files to darknet
#             if obj.status != status:
#                 obj.status = status
#
#                 if status == objmem.ProofStatus.CORRECT:
#                     dir = paths.ws + "/objmem/obj" + str(obj.uid)
#
#                     # Send over every image in the object's metadata directory
#                     for file in os.listdir(dir):
#                         if file.endswith(".jpeg"):
#                             # Load image into sensor_msgs::Image
#                             cv_img = cv2.imread(file)
#                             ros_img = Image()
#                             ros_img.encoding = 'bgr8'
#                             ros_img.height = cv_img.shape[0]
#                             ros_img.width = cv_img.shape[1]
#                             ros_img.step = cv_img.shape[1] * 3
#                             ros_img.data = cv_img.tostring()
#
#                             # Parse bounding box from annotation file
#                             fname = os.path.splitext(file)[0]
#                             annot = open(fname + ".txt", "w")
#                             bbox = [
#                                 float(x) for x in                              \
#                                 annot.readlines()[0].strip().split(",")
#                             ]
#
#                             # Send file to Darknet
#                             add = DarknetAddTrainingFile()
#                             add.network_name = "Find_Object"
#                             add.image = ros_img
#                             add.label = obj.label
#                             add.xmin = bbox[0]
#                             add.xmax = bbox[1]
#                             add.ymin = bbox[2]
#                             add.ymax = bbox[3]
#                             add.image_width = ros_img.width
#                             add.image_height = ros_img.height
#
#                             pub_add_training_file.publish(add)
#
#             return


def objmem_add(msg):
    """Attempts memorizing a new object.

    Parameters
    ----------
    msg : bwi_scavenger_msgs.msg.ObjmemAdd
        message describing object
    """
    obj = objmem.Obj(
        len(objmem.bank),
        msg.label,
        (msg.x, msg.y, msg.z)
    )
    novel, obj = objmem.memorize_rigid(obj, metric='euclid')

    if obj is None:
        log.err("Object position was NaN")
        return

    if novel:
        # Update metadata file
        save_objmem_db()

    # Add image
    dir = paths.ws + "/objmem/obj" + str(obj.uid)
    image_count = len(os.listdir(dir)) // 2  # 2 files per entry (image, annot)
    img_cv = bridge.imgmsg_to_cv2(msg.image, "bgr8")
    cv2.imwrite(dir + "/" + str(image_count) + ".jpeg", img_cv)

    # Add annotation
    annot = open(dir + "/" + str(image_count) + ".txt", "w")
    annot.write("%s,%s,%s,%s" % (msg.xmin, msg.xmax, msg.ymin, msg.ymax))
    annot.close()

    log.info("A(n) %s passed through objmem (novel=%s)" % (msg.label, novel))

    return obj.uid


################################################################################
# SAVING AND LOADING DATABASES
################################################################################


def save_objmem_db():
    """Saves the objmem bank to disk.
    """

    dat = open(paths.objmem + "/objmem.dat", "w")

    for ind, obj in enumerate(objmem.bank):
        # Record object in main dat file
        dat.write("%s,%s,%s,%s,%s,%s\n" % (
                ind,
                obj.label,
                obj.pos[0],
                obj.pos[1],
                obj.pos[2],
                obj.status.name
            )
        )

        # Create a directory for that object's image data
        try:
            os.mkdir(paths.objmem + "/obj" + str(ind))
        except OSError:
            pass

    dat.close()


def load_objmem_db():
    """Loads the objmem bank from disk.
    """

    dat = force_open_read(paths.objmem + "/objmem.dat")

    for line in dat.readlines():
        tokens = line.strip().split(",")
        obj = objmem.Obj(
            int(tokens[0]),
            tokens[1],
            (float(tokens[2]), float(tokens[3]), float(tokens[4])),
            objmem.ProofStatus[tokens[5]]
        )
        objmem.bank.append(obj)

    dat.close()


def save_proof_db():
    """Saves the current proof database to disk.
    """
    for entry in proof_db:
        f = open(paths.ws + "/proofdb/" + str(entry.id) + ".dat", "w")
        lines = [
            "id=" + str(entry.id),
            "task_name=" + entry.task_name,
            "parameter=" + entry.parameter,
            "robot_position=%s,%s,%s" % tuple(entry.robot_position),
            "status=" + entry.status.name,
            "objmem_id=" + str(entry.objmem_id)
        ]

        for line in lines:
            f.write(line + "\n")


def load_proof_db():
    """Loads the proof database state saved on disk. This will also update the
    status of unverified proofs via Scavenger Hunt services.
    """
    scav_get_proof_status = rospy.ServiceProxy(
        "scavenger_hunt/get_proof_status",
        scavenger_hunt_msgs.srv.GetProofStatus
    )

    for file in os.listdir(paths.ws + "/proofdb"):
        f = open(paths.ws + "/proofdb/" + file, "r")
        entry = ProofDbEntry()

        for line in f.readlines():
            pair = line.strip().split("=")

            if len(pair) != 2:
                continue

            if pair[0] == "id":
                entry.id = int(pair[1])
            elif pair[0] == "task_name":
                entry.task_name = pair[1]
            elif pair[0] == "parameter":
                entry.parameter = pair[1]
            elif pair[0] == "robot_position":
                pos = [float(x) for x in pair[1].split(",")]
                entry.robot_position = pos
            elif pair[0] == "status":
                entry.status = ProofStatus[pair[1]]
            elif pair[0] == "objmem_id":
                entry.objmem_id = int(pair[1])

        # If yet to be verified, attempt verification
        if entry.status == ProofStatus.UNVERIFIED:
            req = scavenger_hunt_msgs.srv.GetProofStatusRequest()
            req.id = entry.id
            res = scav_get_proof_status(req)
            entry.status = ProofStatus(res.status)

            if entry.status != ProofStatus.UNVERIFIED:
                log.info("Updated status of proof %s to %s"
                    % (
                        entry.id,
                        entry.status.name
                    )
                )
                if entry.objmem_id > -1:
                    objmem.bank[entry.objmem_id].status = entry.status

        # Add to the status clustering database if verified
        if entry.status != ProofStatus.UNVERIFIED:
            status_clustering.expand_database(
                entry.task_name,
                entry.parameter,
                entry.status,
                entry.robot_position
            )

        proof_db.append(entry)

    # Statuses may have changed
    save_proof_db()
    save_objmem_db()


################################################################################
# QUERYING INFORMATION FROM DATABASES
################################################################################


def confirm_object(req):
    """A service for determining if an object was previously marked correct or
    incorrect by Scavenger Hunt.

    Parameters
    ----------
    req : bwi_scavenger_msgs.srv.ConfirmObjectRequest
        service request

    Return
    ------
    bwi_scavenger_msgs.srv.ConfirmObjectResponse
        service response
    """
    obj = objmem.Obj(
        -1,
        req.label,
        (req.position.x, req.position.y, req.position.z),
        ProofStatus.UNVERIFIED
    )
    novel, obj = objmem.memorize_rigid(obj, grow_bank=False)
    res = bwi_scavenger_msgs.srv.ConfirmObjectResponse()
    res.ok = novel or obj is None or obj.status != ProofStatus.INCORRECT

    return res


def get_priority_points(req):
    """A service for getting historically successful locations for a particular
    task.

    Parameters
    ----------
    req : bwi_scavenger_msgs.srv.GetPriorityPointsRequest
        service request with task name and parameter

    Return
    ------
    bwi_scavenger_msgs.srv.GetPriorityPointsResponse
        service response with (potentially empty) list of points
    """
    centroids = status_clustering.get_priority_centroids(
        req.task_name, req.task_parameter
    )
    res = bwi_scavenger_msgs.srv.GetPriorityPointsResponse()

    for c in centroids:
        point = Point()
        point.x = c[0][0]
        point.y = c[0][1]
        point.z = c[0][2]
        res.points.append(point)

    log.info("Served %s priority points." % len(centroids))

    return res


################################################################################
# VISUALIZATION
################################################################################


def visualize():
    """Populates rviz with useful markers for visualizing object locations,
    priority points, etc.
    """
    log.info("Sending visuals to rviz...")

    marker_id = 0

    # Visualize objmem
    for ind, obj in enumerate(objmem.bank):
        # Position marker
        msg = Marker()
        msg.header.frame_id = "level_mux_map"
        msg.header.stamp = rospy.get_rostime()
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD
        msg.scale.x = msg.scale.y = msg.scale.z = 0.5
        msg.pose.position.x = obj.pos[0]
        msg.pose.position.y = obj.pos[1]
        msg.pose.position.z = 0

        # Green -> correct
        # Red -> incorrect
        # Blue -> unverified
        if obj.status == ProofStatus.CORRECT:
            msg.color.r = 0
            msg.color.b = 0
            msg.color.g = 1
        elif obj.status == ProofStatus.INCORRECT:
            msg.color.r = 1
            msg.color.b = 0
            msg.color.g = 0
        else:
            msg.color.r = 0
            msg.color.b = 1
            msg.color.g = 0

        msg.color.a = 1
        msg.id = marker_id
        marker_id += 1

        # Label marker
        label = Marker()
        label.header.frame_id = "level_mux_map"
        label.header.stamp = rospy.get_rostime()
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.scale.x = label.scale.y = label.scale.z = 0.5
        label.text = str(obj.uid) + " " + obj.label
        label.pose.position.x = obj.pos[0]
        label.pose.position.y = obj.pos[1]
        label.pose.position.z = obj.pos[2] + 1
        label.color.r = 1
        label.color.b = 1
        label.color.g = 1
        label.color.a = 1
        label.id = marker_id
        marker_id += 1

        pub_marker.publish(msg)
        pub_marker.publish(label)

    # Visualize priority points
    for key in status_clustering.completion_clusters:
        clusters = status_clustering.completion_clusters[key]

        for cluster in clusters:
            pos = status_clustering.get_cluster_centroid(cluster)

            # Position marker
            msg = Marker()
            msg.header.frame_id = "level_mux_map"
            msg.header.stamp = rospy.get_rostime()
            msg.type = Marker.SPHERE
            msg.action = Marker.ADD
            msg.scale.x = msg.scale.y = msg.scale.z = 2
            msg.pose.position.x = pos[0]
            msg.pose.position.y = pos[1]
            msg.pose.position.z = 0

            if status_clustering.score_cluster(cluster) >= 0:
                msg.color.r = 0
                msg.color.b = 0
                msg.color.g = 1
            else:
                msg.color.r = 1
                msg.color.b = 0
                msg.color.g = 0

            msg.color.a = 0.5
            msg.id = marker_id
            marker_id += 1

            # Label marker
            label = Marker()
            label.header.frame_id = "level_mux_map"
            label.header.stamp = rospy.get_rostime()
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.scale.x = label.scale.y = label.scale.z = 0.5
            label.text = key
            label.pose.position.x = pos[0]
            label.pose.position.y = pos[1]
            label.pose.position.z = 1
            label.color.r = 1
            label.color.b = 1
            label.color.g = 1
            label.color.a = 1
            label.id = marker_id
            marker_id += 1

            pub_marker.publish(msg)
            pub_marker.publish(label)

    log.info("Done.")


################################################################################
# NODE ENTRYPOINT
################################################################################


if __name__ == "__main__":
    rospy.init_node(log.tag_name)

    # Create database folders if nonexistent
    try:
        os.mkdir(paths.ws + "/proofdb")
    except OSError:
        pass

    try:
        os.mkdir(paths.objmem)
    except OSError:
        pass

    # Load databases (objmem before proof, always)
    load_objmem_db()
    load_proof_db()
    status_clustering.cluster()

    log.info("Loaded %s proof(s)." % len(proof_db))
    log.info("Loaded %s object(s)." % len(objmem.bank))

    rospy.Subscriber(
        TPC_OBJMEM_NODE_ADD,
        ObjmemAdd,
        objmem_add
    )
    pub_add_training_file = rospy.Publisher(
        TPC_DARKNET_NODE_ADD_TRAINING_FILE,
        DarknetAddTrainingFile,
        queue_size=0
    )
    pub_marker = rospy.Publisher(
        "/bwi_scavenger/vis",
        Marker,
        queue_size=0
    )
    rospy.Service(
        SRV_PROOFDB_NODE_SEND_PROOF,
        bwi_scavenger_msgs.srv.SendProof,
        send_proof
    )
    rospy.Service(
        SRV_CONFIRM_OBJECT,
        bwi_scavenger_msgs.srv.ConfirmObject,
        confirm_object
    )
    rospy.Service(
        SRV_GET_PRIORITY_POINTS,
        bwi_scavenger_msgs.srv.GetPriorityPoints,
        get_priority_points
    )
    scav_send_proof = rospy.ServiceProxy(
        "/scavenger_hunt/send_proof",
        scavenger_hunt_msgs.srv.SendProof
    )

    rospy.sleep(3)
    visualize()
    log.info("Standing by.")
    rospy.spin()
