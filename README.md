# Scavenger Hunt

Complete codebase for BWI's Scavenger Hunt project.

`website/` contains everything relating to the website. All other directories
are ROS packages for Scavenger Hunt robots. Some important ones:

* `scavenger_hunt/` - The Scavenger Hunt API. Contains utilities for connecting
to the website, uploading proofs, and querying proof feedback. Non-ROSified,
though there is an optional ROS wrapper.
* `bwi_scavenger/` - Drivers for our Scavenger Hunt participant robots.

## BWI Scavenger Startup

### Initial Configuration

1. Verify that `#define SIMULATION` is commented out in `robot_motion.h` (if
not running in simulation). This ensures correctness of world coordinates.

2. In `scavenger_hunt/src/scavenger_hunt_client.cpp` ensure
`std::string DOMAIN` is pointing to the correct website domain. Until the web
server is operational, this is either `localhost` for lab machines or
`localhost:8080` for robots.

3. In `scavenger_hunt_node.cpp`, ensure `std::string PROOF_TEMP_PATH` is a
valid path (except for the name of the file at the end of the path--that doesn't
have to exist). This is simply a scratch file used for image transport.

4. Ensure the variables in `bwi_scavenger/config/bwi_scavenger.yaml` are
correct.
  - `bwi_scavenger/scav_hunt_login/*` - valid website credentials
  - `bwi_scavenger/ws_path` - fully-qualified path to the workspace containing
  this repository

Some day we'll allocate all of these things in a single configuration file.

### Robot Startup

1. Boot up the scavenger robot (probably Pickles or Bender). Before any ROS
commands, open an SSH tunnel to Kane so the robot can see the website.

```
ssh -L 8080:localhost:80 bwilab@kane.csres.utexas.edu
```

2. Navigate to the workspace and launch the following drivers in the order they
appear. Verify that `offboard.launch` opens a window displaying object
detections. If it doesn't, you may need to fiddle with the Kinect cable.

```
roslaunch bwi_scavenger drivers.launch
roslaunch bwi_scavenger offboard.launch
roslaunch bwi_scavenger scavenger.launch
```

The segbot drivers are included in `drivers.launch`. `offboard.launch` contains
computationally expensive nodes such as `darknet_ros` that are ideally run
offboard, though this has historically produced latency issues that prevent the
robot from reliably completing tasks. We're still investigating workarounds.

3. Localize the robot in rviz and start a hunt via the `do_hunt` node.

```
rosrun bwi_scavenger do_hunt "BWI Lab Hunt"
```

`BWI Lab Hunt` is the name of our test hunt, though any hunt registered on the
website will work.

## BWI Scavenger With Offboard Computing

The procedure is identical, except that `offboard.launch` is run on Kane. Also,
each terminal window requires sourcing `scav.bash` to export the IPs and master
URI that interact with OpenVPN (which should start automatically on all relevant
machines).

On Kane, enter the following:

```
cd src
source scav.bash kane <robot-name>
```

On the robot, enter the following:

```
source scav.bash <robot-name>
```

Robot name is either `pickles` or `bender`.
