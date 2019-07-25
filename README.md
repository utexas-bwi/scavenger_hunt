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

### 0. Bring up offboard nodes

We like to use Kane for offboard computing. This is done with OpenVPN, which
should start automatically on all relevant machines.

Navigate to the workspace on Kane containing this repository, source a few
things, and start roscore.

```
cd /home/bwilab/scavenger_hunt
source devel/setup.bash
source src/scav.bash kane
roscore
```

The latter of the sources will export the correct IPs and URIs to facilitate the
remote ROS connection.

With that done, launch the remote nodes in another terminal (remember to
re-soruce):

```
source src/scav.bash kane
roslaunch bwi_scavenger offboard.launch
```

### 1. Scavenger robot startup

Boot up the scavenger robot and `cd` into the workspace containing this
repository. Repeat the source procedure. Replace `<robot-name>` with the robot's
name, probably either `pickles` or `bender`.

```
source devel/setup.bash
source src/scav.bash <robot-name>
```

Confirm the correctness of environment variables in
`bwi_scavenger/config/bwi_scavenger.yaml` (namely, website login credentials and
the fully-qualified workspace path).

Open an SSH tunnel to Kane so the robot can see the website.

```
ssh -L 8080:localhost:80 bwilab@kane.csres.utexas.edu
```

Then, launch the ROS drivers. This will automatically launch the segbot drivers
as well.

```
roslaunch bwi_scavenger drivers.launch
```

Once those have had time to spin up, launch the suite of general scavenger
nodes. These are purely utility and will not initiate any scavenging behavior.
Remember to re-source `scav.bash` for every new terminal running scavenger
nodes.

```
source src/scav.bash <robot-name>
roslaunch bwi_scavenger scavenger.launch
```

### 2. Initiate a scavenger hunt

The node `do_hunt` completes scavenger hunts. After localizing the robot, run
this node by providing a hunt name ("BWI Lab Hunt" is our main test hunt).

```
source src/scav.bash <robot-name>
rosrun bwi_scavenger do_hunt "BWI Lab Hunt"
```
