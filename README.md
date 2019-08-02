# Scavenger Hunt

Complete codebase for BWI's Scavenger Hunt project.

`website/` contains everything relating to the website. All other directories
are ROS packages for Scavenger Hunt robots. Some important ones:

* `scavenger_hunt/` - The Scavenger Hunt API. Contains utilities for connecting
to the website, uploading proofs, and querying proof feedback. Non-ROSified,
though there is an optional ROS wrapper.
* `bwi_scavenger/` - Drivers for our Scavenger Hunt participant robots.

## BWI Scavenger Startup

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
