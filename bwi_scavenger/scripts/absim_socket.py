import absim.hunt
import socket
import time


agent_name = "proximity"
robot = None
datfile = "/home/bwilab/scavenger_hunt/src/bwi_scavenger/exp.dat"
ip = "127.0.0.1" # TODO change to robot IP
port = 5005


def load():
    """Load scavenger hunt from datfile.
    """
    global robot
    world, hunt, start_loc = absim.hunt.parse_world(datfile)
    agent_lambda = absim.hunt.agent_lookup[agent_name]
    robot = agent_lambda(world, hunt, world.node_id(start_loc))
    robot.epoch()
    robot.setup()


if __name__ == "__main__":
  load()
  time.sleep(3)

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

  sock.bind((ip, port))

  while True:
    objects_found, addr = sock.recvfrom(1000) # Get the objects_found data sent over the socket (from absim_ros)
    # print(type(objects_found))
    objects_found = objects_found.decode('utf-8').split(",")

    # Get the next location
    robot.objects_at_loc = objects_found
    robot.run()
    next_location = robot.loc

    sock.sendto(bytes(next_location, 'utf-8'), addr)
