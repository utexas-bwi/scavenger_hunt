import rospy

class Logger:
    def __init__(self, tag_name):
        self.tag_name = tag_name

    def info(self, data):
        rospy.loginfo(("[%s] " + data) % self.tag_name)

    def warn(self, data):
        rospy.logwarn(("[%s] " + data) % self.tag_name)

    def err(self, data):
        rospy.logerr(("[%s] " + data) % self.tag_name)
