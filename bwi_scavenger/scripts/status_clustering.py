"""Status clustering is the robot's method of learning which locations in its
environment are suitable for different tasks. This module maintains a mapping of
(task, parameter) pairs to collections of (position, status) pairs, where
position was the position of the robot when submitting a particular proof and
status was the verified result of the proof.

Verified proofs and their associated locations are clustered with DBSCAN. Each
cluster is scored based on the relative number of successful and failed proofs
within them for a particular task. At the beginning of the task, the robot
can query a list of the top scored locations for the task it is about to
attempt. Ideally, the robot prioritizes visiting these locations.
"""
import math
import numpy as np
import objmem

from scavenger_hunt import ProofStatus
from dbscan import MyDBSCAN


completion_clusters = {}
completion_databases = {}

EPS = 2  # DBSCAN epsilon
MIN_SAMPLES = 2  # DBSCAN minimum samples per cluster

FEAR_OF_FAILURE = 1.25  # Weight of failed proofs over successful ones
PRIORITY_SCORE_THRESHOLD = 0  # Only clusters w/ a score > this are prioritized


class TaskCompletion:
    """Robot position and proof status pairing for some (task, parameter) pair.
    A tuple is better suited for this. Sue me.
    """
    def __init__(self, status, pos):
        self.status = status
        self.pos = pos


def get_key(task, param):
    """Creates a database key used to map a (task, parameter) pair to its
    priority clusters.

    Parameters
    ----------
    task : str
        task name
    param: str
        parameter name

    Return
    ------
    str
        key
    """
    return task + "/" + param


def expand_database(task, param, status, position):
    """Logs a new task completion in the database.

    Parameters
    ----------
    task : str
        task name
    param : str
        task parameter
    status : scavenger_hunt.ProofStatus
        status of the proof submitted for the task at this location (verified
        proofs ONLY)
    position : iterable
        3-tuple (x, y, z) of the robot's location at the time of task completion

    """
    key = get_key(task, param)

    if key not in completion_databases:
        completion_databases[key] = []

    comp = TaskCompletion(status, np.array(position))
    completion_databases[key].append(comp)


def cluster():
    """Cluster all databases. Call this after major changes to the database
    (initial load, etc.).
    """
    for key in completion_databases:
        # Look up the completion database for this task and parameter. If it's
        # empty, we don't care
        completion_database = completion_databases[key]

        if len(completion_database) == 0:
            continue

        # Assemble a database of the completion positions that sklearn can
        # understand
        position_database = []

        for item in completion_database:
            position_database.append(item.pos)

        # Perform the clustering
        labels = MyDBSCAN(position_database, EPS, MIN_SAMPLES)

        # Assemble actual cluster lists from the label vector
        cluster_count = len(set(labels)) - (1 if -1 in labels else 0)
        completion_clusters[key] = []
        key_clusters = completion_clusters[key]

        for i in range(cluster_count):
            key_clusters.append([])

        for index, comp in enumerate(completion_database):
            label = labels[index]

            if label == -1:
                continue

            key_clusters[label - 1].append(comp)


def get_cluster_centroid(cluster):
    """Computes the average position within a position cluster.

    Parameters
    ----------
    cluster : list
        list of np.array of the form (x, y, z)

    Return
    ------
    np.array
        centroid position (x, y, z)
    """
    centroid = np.array([0.0, 0.0, 0.0])

    for item in cluster:
        centroid += item.pos

    return centroid / len(cluster)


def score_cluster(cluster):
    """Scores a cluster based on the number of successful and failed proofs
    within it.

    Parameters
    ----------
    cluster : list
        list of TaskCompletion

    Return
    ------
    float
        cluster score (>0 good, <0 bad)
    """
    score = 0
    # Rating function
    F = lambda status : 1 if status == ProofStatus.CORRECT else -FEAR_OF_FAILURE

    for item in cluster:
        score += F(item.status)

    return score


def get_priority_centroids(task, param):
    """Gets a list of all the positively scored centroids for a task, sorted by
    descending score.

    Parameters
    ----------
    task : str
        task name
    param : str
        task parameter

    Return
    ------
    list
        list of (pos, score) tuples sorted by descending score
    """
    key = get_key(task, param)

    if key not in completion_clusters:
        return np.array([])

    clusters = completion_clusters[key]
    centroids = []

    for item in clusters:
        score = score_cluster(item)

        if score > PRIORITY_SCORE_THRESHOLD:
            centroids.append((get_cluster_centroid(item), score))

    centroids.sort(key=lambda x : x[1], reverse=True)

    return centroids
