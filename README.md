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
