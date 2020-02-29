# darksocket

A tool for offboard [Darknet](https://github.com/pjreddie/darknet) computing. Comes as a standalone script and a ROS node.

## Standalone Usage

A config file `darksocket.conf` should exist in the same directory as `darksocket.py`. The config file looks like the following:

```python
darknet_path=xxx   # Path to darknet binary
cfg_path=xxx       # Path to network cfg file
weights_path=xxx   # Path to network weights file
meta_path=xxx      # Path to network metadata file
recv_img_path=xxx  # Path to store a received image in, including file name
host=xxx           # Server hostname or IP
port=xxx           # Server port
```

A robot (server) and offboard compute node (client) communicate through a single TCP socket. An ideal implementation would reverse the client/server roles and use UDP, but this tool represents a hack around the quirks of UT Austin's network.

A class `darksocket.Server` provides server functionality. An example can be seen in the method `server_example`. Run the example with

```
python darksocket.py server_example
```

The client is already written. Run it with

```
python darksocket.py client
```

## ROS Usage

The setup is identical except that we have implemented the server in a ROS node (the client still stands alone). This node broadcasts `Detections` which pair `BoundingBoxes` from [leggedrobotics' darknet_ros wrapper](https://github.com/leggedrobotics/darknet_ros) with the `Image` they correspond to.

Configure the path to `darksocket.conf` and topic names in `config/darksocket_ros.yaml` and launch with

```
roslaunch darksocket_ros darksocket_ros.launch
```

Then, run the client on the offboard machine like usual.

```
python darksocket.py client
```

## Dependencies

* Python 2
* [darknet_ros](https://github.com/leggedrobotics/darknet_ros) (ROS usage only)
