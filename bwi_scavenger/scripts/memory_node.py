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
import numpy as np
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
from geometry_msgs.msg import Pose
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
pub_objmem_markers = None
pub_priority_markers = None


class ProofDbEntry:
    """Metadata about a proof submitted to the website.
    """
    def __init__(
        self,
        id=0,
        task_name="",
        parameter="",
        robot_position=[0, 0, 0],
        robot_orientation=[1, 0, 0, 0],
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
        robot_orientation: 4-tuple of float
            (w, x, y, z) quaternion components
        status: ProofStatus
            status of the proof; updates on database load
        objmem_id: int
            (optional) ID of associated object in objmem
        """
        self.id = id
        self.task_name = task_name
        self.parameter = parameter
        self.robot_position = robot_position
        self.robot_orientation = robot_orientation
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
        (
            req.robot_pose.position.x,
            req.robot_pose.position.y,
            req.robot_pose.position.z
        ),
        (
            req.robot_pose.orientation.w,
            req.robot_pose.orientation.x,
            req.robot_pose.orientation.y,
            req.robot_pose.orientation.z
        ),
        ProofStatus.UNVERIFIED,
        -1
    )

    # Find_Object -- Attempt object memorization
    if req.task.name == "Find Object":
        add_msg = ObjmemAdd()
        add_msg.label = param
        add_msg.x = req.secondary_pose.position.x
        add_msg.y = req.secondary_pose.position.y
        add_msg.z = req.secondary_pose.position.z
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
            "robot_orientation=%s,%s,%s,%s" % tuple(entry.robot_orientation),
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
            elif pair[0] == "robot_orientation":
                quat = [float(x) for x in pair[1].split(",")]
                entry.robot_orientation = quat
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
        if entry.status != ProofStatus.UNVERIFIED or CLUSTER_ANY:
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
        pose = Pose()
        pose.position.x = c[0][0]
        pose.position.y = c[0][1]
        pose.position.z = c[0][2]
        res.poses.append(pose)
        res.scores.append(c[1])

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
        m_obj = Marker()
        m_obj.header.frame_id = "level_mux_map"
        m_obj.header.stamp = rospy.get_rostime()
        m_obj.type = Marker.SPHERE
        m_obj.action = Marker.ADD
        m_obj.scale.x = m_obj.scale.y = m_obj.scale.z = 0.5
        m_obj.pose.position.x = obj.pos[0]
        m_obj.pose.position.y = obj.pos[1]
        m_obj.pose.position.z = 0

        # Green -> correct
        # Red -> incorrect
        # Blue -> unverified
        if obj.status == ProofStatus.CORRECT:
            m_obj.color.r = 0
            m_obj.color.b = 0
            m_obj.color.g = 1
        elif obj.status == ProofStatus.INCORRECT:
            m_obj.color.r = 1
            m_obj.color.b = 0
            m_obj.color.g = 0
        else:
            m_obj.color.r = 0
            m_obj.color.b = 1
            m_obj.color.g = 0

        m_obj.color.a = 1
        m_obj.id = marker_id
        marker_id += 1

        pub_objmem_markers.publish(m_obj)

        # Label marker
        m_lab = Marker()
        m_lab.header.frame_id = "level_mux_map"
        m_lab.header.stamp = rospy.get_rostime()
        m_lab.type = Marker.TEXT_VIEW_FACING
        m_lab.action = Marker.ADD
        m_lab.scale.x = m_lab.scale.y = m_lab.scale.z = 0.5
        m_lab.text = str(obj.uid) + " " + obj.label
        m_lab.pose.position.x = obj.pos[0]
        m_lab.pose.position.y = obj.pos[1]
        m_lab.pose.position.z = obj.pos[2] + 1
        m_lab.color.r = 1
        m_lab.color.b = 1
        m_lab.color.g = 1
        m_lab.color.a = 1
        m_lab.id = marker_id
        marker_id += 1

        pub_objmem_markers.publish(m_lab)

    # Visualize priority points
    for key in status_clustering.completion_clusters:
        clusters = status_clustering.completion_clusters[key]

        for cluster in clusters:
            score = status_clustering.score_cluster(cluster)
            pos = status_clustering.get_cluster_centroid(cluster)

            # Visualize points in cluster
            dist_max = -1

            for item in cluster:
                dist = np.linalg.norm(pos - item.pos)

                if dist > dist_max:
                    dist_max = dist

                # Item marker
                m_item = Marker()
                m_item.header.frame_id = "level_mux_map"
                m_item.header.stamp = rospy.get_rostime()
                m_item.type = Marker.SPHERE
                m_item.action = Marker.ADD
                m_item.scale.x = m_item.scale.y = m_item.scale.z = 0.1
                m_item.pose.position.x = pos[0]
                m_item.pose.position.y = pos[1]
                m_item.pose.position.z = 0

                if item.status == ProofStatus.CORRECT:
                    m_item.color.r = 0
                    m_item.color.b = 0
                    m_item.color.g = 1
                else:
                    m_item.color.r = 1
                    m_item.color.b = 0
                    m_item.color.g = 0

                m_item.color.a = 0.5
                m_item.id = marker_id
                marker_id += 1

                pub_priority_markers.publish(m_item)

            # Radius marker
            m_rad = Marker()
            m_rad.header.frame_id = "level_mux_map"
            m_rad.header.stamp = rospy.get_rostime()
            m_rad.type = Marker.CYLINDER
            m_rad.action = Marker.ADD
            m_rad.scale.x = m_rad.scale.y = dist_max / 2
            m_rad.scale.z = 0.05
            m_rad.pose.position.x = pos[0]
            m_rad.pose.position.y = pos[1]
            m_rad.pose.position.z = 0

            if score >= 0:
                m_rad.color.r = 0
                m_rad.color.b = 0
                m_rad.color.g = 1
            else:
                m_rad.color.r = 1
                m_rad.color.b = 0
                m_rad.color.g = 0

            m_rad.color.a = 0.5
            m_rad.id = marker_id
            marker_id += 1

            pub_priority_markers.publish(m_rad)

            # Waypoint
            m_wayp = Marker()
            m_wayp.header.frame_id = "level_mux_map"
            m_wayp.header.stamp = rospy.get_rostime()
            m_wayp.type = Marker.SPHERE
            m_wayp.action = Marker.ADD
            m_wayp.scale.x = m_wayp.scale.y = m_wayp.scale.z = 0.33
            m_wayp.pose.position.x = pos[0]
            m_wayp.pose.position.y = pos[1]
            m_wayp.pose.position.z = 1
            m_wayp.color.r = m_rad.color.r
            m_wayp.color.b = m_rad.color.b
            m_wayp.color.g = m_rad.color.g
            m_wayp.color.a = 1
            m_wayp.id = marker_id
            marker_id += 1

            pub_priority_markers.publish(m_wayp)

            # Label marker
            m_lab = Marker()
            m_lab.header.frame_id = "level_mux_map"
            m_lab.header.stamp = rospy.get_rostime()
            m_lab.type = Marker.TEXT_VIEW_FACING
            m_lab.action = Marker.ADD
            m_lab.scale.x = m_lab.scale.y = m_lab.scale.z = 0.5
            m_lab.text = key + " (s=%s)" % score
            m_lab.pose.position.x = pos[0]
            m_lab.pose.position.y = pos[1]
            m_lab.pose.position.z = 1.33
            m_lab.color.r = 1
            m_lab.color.b = 1
            m_lab.color.g = 1
            m_lab.color.a = 1
            m_lab.id = marker_id
            marker_id += 1

            pub_priority_markers.publish(m_lab)

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
    pub_objmem_markers = rospy.Publisher(
        "/bwi_scavenger/objmem_vis",
        Marker,
        queue_size=0
    )
    pub_priority_markers = rospy.Publisher(
        "/bwi_scavenger/priority_vis",
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
