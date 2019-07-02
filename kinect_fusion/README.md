# kinect_fusion

A ROS package for estimating the relative position of objects with CV and a
Kinect sensor. This particular implementation uses
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

See `kinect_fusion_test_node.cpp` for example usage. Uncommenting the `VISUALIZE`
definition in `kinect_fusion.h` can be useful in tuning the estimator's
parameters.
