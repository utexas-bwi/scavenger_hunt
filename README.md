# Scavenger Hunt

Complete codebase for BWI's Scavenger Hunt project.

`website/` contains everything relating to the website. All other directories
are ROS packages for Scavenger Hunt robots. Some important ones:

* `scavenger_hunt_api/` - The Scavenger Hunt API. Contains utilities for connecting
to the website, uploading proofs, and querying proof feedback. Non-ROSified,
though there is an optional ROS wrapper.
* `bwi_scavenger/` - Drivers for our Scavenger Hunt participant robots.
* `darksocket/` - Tool for offboard darknet computing.

## BWI Scavenger Startup

### Initial Configuration

* Verify launch parameters in `bwi_scavenger/config/bwi_scavenger.yaml`
  * `scav_hunt_login` - Valid website credentials
  * `ws_path` - Fully-qualified path to the workspace containing these packages
  * `world` - Either `sim` if running in Gazebo or `irl` if running in the lab
  * `topics/perception/xxx` - Camera topics should be prefixed with `/nav_kinect` for IRL and `/camera` for simulation
* Verify launch parameters in `scavenger_hunt_api/config/scavenger_hunt.yaml`
  * `website/domain` - Domain of Scavenger Hunt website (until the web server is
    up this is `localhost` for lab machines and `localhost:8080` for robots)
  * `scratch_path` - Valid path that ROS has permission to create files in
    (we usually use the workspace path)
* Verify darksocket config in `darksocket/scripts/darksocket.conf`
  * Robot and offboard machine
    * `host` - IP of robot (find with `hostname -I`; should be the second IP printed)
    * `port` - we usually use 51820
  * Offboard machine only
    * `darknet_path` - Location of darknet binary (built with `GPU=1`, otherwise you're undermining the purpose of darksocket)
    * `cfg_path` - Fully-qualified path of cfg file
    * `weights_path` - Fully-qualified path of weights file
    * `meta_path` - Fully-qualified path of meta file (which in turn contains the fully-qualified path of the names file)
    * `recv_img_path` - Fully-qualified path to store a received image in (including image file name)
* Verify darksocket launch parameters in `darksocket/launch/darksocket_ros.launch`
  * `topics/camera` - Camera topics should be prefixed with `/nav_kinect` for IRL and `/camera` for simulation
  * `conf_path` - Fully-qualified path to `darksocket.conf`


### Robot Startup

1. Boot up the scavenger robot (probably Pickles or Bender). Before any ROS
commands, open an SSH tunnel to Kane so the robot can see the website.

```
ssh -L 8080:localhost:80 bwilab@kane.csres.utexas.edu
```

2. Navigate to the workspace and run the following launch files.

```
roslaunch bwi_launch segbot_v2.launch
roslaunch bwi_scavenger scavenger.launch
rosrun bwi_scavenger absim_ros.py
python3 src/bwi_scavenger/scripts/absim_socket.py
```

3. Run darksocket on the offboard machine. If everything was set up correctly, it should load the network and then print "connection established."

```
python darksocket.py client
```

4. Localize the robot in rviz and start a hunt via the `do_hunt` node.

```
rosrun bwi_scavenger do_hunt "BWI Lab Hunt"
```

`BWI Lab Hunt` is the name of our test hunt, though any hunt registered on the
website will work.
