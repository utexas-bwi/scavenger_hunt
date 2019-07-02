# kinect_fusion

A ROS package for estimating the relative position of objects in 3D space with
CV and a Kinect sensor. This particular implementation uses
[YOLO ROS](https://github.com/leggedrobotics/darknet_ros) for CV and the
[freenect](https://github.com/OpenKinect/libfreenect) Kinect driver.

The algorithm is fast and deterministic. It is an improvement on an undergraduate
research project done at the University of Texas at Austin on computationally
inexpensive object segmentation. The original implementation can be found
[here](https://github.com/stefandebruyn/depth-cv-fusion), along with a paper
detailing its inner workings.

Notable changes include the following:

* Elimination of the extrema clustering step. This was found to be only
marginally detrimental to accuracy, while significantly decreasing runtime.
* IR FOV projection. Bounding boxes are scaled for better correspondence between
depth and color data, as these sensors have inherently different FOVs.
* Use of erode/dilate to filter noisy spots that escape the extrema search.

See `kinect_fusion_test_node.cpp` for example usage. This node can be built and
run with Catkin. Uncommenting the `VISUALIZE` definition in `kinect_fusion.cpp`
can be useful when tuning the estimator's parameters.
