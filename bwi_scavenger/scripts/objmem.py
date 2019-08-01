import enum
import math
import numpy as np
import os
import paths

from scavenger_hunt import ProofStatus

bank = []
rigid_euclid_threshold = 1


class Obj:
    """A memorized object with some metadata.
    """
    def __init__(self, uid, label, pos, status=ProofStatus.UNVERIFIED):
        self.uid = uid
        self.label = label
        self.pos = pos
        self.status = status

    def __str__(self):
        return "%s@(%s, %s, %s)%s" % (
            self.label, self.pos[0], self.pos[1], self.pos[2], self.status
        )


def memorize_rigid(obj, metric='euclid', grow_bank=True):
    """Attempt to memorize an object rigidly.

    Parameters
    ----------
    obj : objmem.Obj
        object to memorize
    metric : str
        novelty evaluation method
            euclid -> Euclidian distance < rigid_euclid_threshold
    grow_bank : bool
        whether or not to add novel objects to the memory bank

    Returns
    -------
    (bool, objmem.Obj)
        (if the object was deemed novel, object container)
    """
    if np.isnan(obj.pos[0]) or np.isnan(obj.pos[1]) or np.isnan(obj.pos[2]):
        return False, None

    for obj_i in bank:
        if obj_i.label != obj.label:
            continue

        if metric == 'euclid':
            dx = obj.pos[0] - obj_i.pos[0]
            dy = obj.pos[1] - obj_i.pos[1]
            dz = obj.pos[2] - obj_i.pos[2]
            dist = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

            if dist < rigid_euclid_threshold:
                return False, obj_i

    if grow_bank:
        bank.append(obj)

    return True, obj
