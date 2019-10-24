import rospy

from globals import *
from bwi_scavenger_msgs.srv import GetNextLocation, SaveWorld

def get_next_location(req):
  # Req takes in the current robot location and the objects that
  # have just been found at said location

  # Res returns the next location to travel to (based on the algorithm 
  # previously provided)

  return true

def save_world(req):
  # Saves the world / nodes in the world
  # Saves the algorithm to be used for the hunt
  # Saves the occurrence model
  # Saves the hunt to be accomplished

  return true

if __name__ == "__main__":
  rospy.init_node('robot_absim_services')

  service_get_next_location = rospy.Service(
    SRV_GET_NEXT_LOCATION,
    GetNextLocation,
    get_next_location
  )

  service_save_world = rospy.Service(
    SRV_SAVE_WORLD,
    SaveWorld,
    save_world
  )

  rospy.spin()
