import rospy
import math

class Logger:
    def __init__(self, tag_name):
        self.tag_name = tag_name

    def info(self, data):
        rospy.loginfo(("[%s] " + data) % self.tag_name)

    def warn(self, data):
        rospy.logwarn(("[%s] " + data) % self.tag_name)

    def err(self, data):
        rospy.logerr(("[%s] " + data) % self.tag_name)


def approx(a, b, negl=1e-6):
    """Gets if two integral types are approximately equivalent.
    Parameters
    ----------
    a : int, float
        lhs
    b : int, float
        rhs
    negl : int, float
        maximum allowed difference
    Return
    ------
    bool
        if a ~= b
    """
    return math.fabs(a - b) <= negl
