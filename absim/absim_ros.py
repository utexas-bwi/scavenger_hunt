import bwi_scavenger_msgs.srv
import hunt
import rospy


agent_name = "dynamic"
agent = None
datfile = "/home/bwilab/exp.dat"


def update(req):
    """Step the algorithm once and return the location it traveled to.
    """
    res = bwi_scavenger_msgs.srv.GetNextLocationResponse()
    res.next_location = "done"
    if not agent.is_done():
        agent.objects_at_current_node = req.objects_found
        agent.run()
        res.next_Location = agent.current_node()
    return res


def load():
    """Load scavenger hunt from datfile.
    """
    cmd = [
        "hunt.py",
        datfile,
        "-a",
        agent_name,
        "-s"
    ]
    map, hunt, start_loc, params = hunt.parse(cmd)
    agent_lambda = hunt.agent_lookup[agent_name]
    agent = agent_lambda(map, hunt.copy(), start_loc)
    agent.epoch()


if __name__ == "__main__":
    rospy.init_node("/absim_node")
    load()

    proxy = rospy.Service(
        "/absim",
        bwi_scavenger_msgs.srv.GetNextLocation,
        update
    )

    rospy.sleep(3)
    rospy.spin()
