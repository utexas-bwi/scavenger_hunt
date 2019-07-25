# scavenger-hunt

Complete codebase for the Lifelong Learning Machines Scavenger Hunt project.

`website/` contains everything relating to the website. All other directories
are ROS packages for Scavenger Hunt robots. Some important ones:

* `scavenger_hunt/` - The Scavenger Hunt API. Contains classes for connecting to
the website, uploading proofs, organizing information about hunts, etc.
* `bwi_scavenger/` - Drivers for our Scavenger Hunt participant robots.
* `bwi_scavenger_msgs/` - Messages and services for aforementioned drivers.
* `kinect_fusion/` - Utility for estimating relative object pose with a Kinect
sensor.

## BWI Scavenger Startup

0. Bring up offboard nodes

We like to use Kane for offboard computing. With Kane online, navigate to the
workspace containing this repository and source a few things:

```
cd /home/bwilab/scavenger_hunt
source devel/setup.bash
source src/export.bash kane
```

The latter of the sources will export the correct IPs and URIs to facilitate the
remote ROS connection.

With that done, launch the remote nodes:

```
roslaunch bwi_scavenger offboard.launch
```

1. `cd` into the workspace and source a few things. The second script will
launch the necessary ROS drivers, including the segbot drivers. Replace
`<robot-name>` with the robot's name, probably either `pickles` or `bender`.

```
source devel/setup.bash
source src/scav.bash <robot-name>
```

2.
