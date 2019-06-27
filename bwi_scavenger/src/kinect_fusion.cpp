#include <algorithm>
#include <bwi_scavenger/kinect_fusion.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>

#define VISUALIZE

static const float RADIANS_PER_DEGREE = 3.14159 / 180;
// Dimensions of Kinect color FOV in degrees
static const float KINECT_HORIZ_FOV = 62 * RADIANS_PER_DEGREE;
static const float KINECT_VERT_FOV = 48.6 * RADIANS_PER_DEGREE;
// Kinect sensor resolution
static const int IMAGE_WIDTH = 640;
static const int IMAGE_HEIGHT = 480;

void kinect_fusion::adjust_bounding_box(darknet_ros_msgs::BoundingBox &box) {
  const int ALPHA = 34; // # Pixels between top of IR FOV and camera FOV
  const int BETA = 27; // # Pixels between left/right of IR FOV and camera FOV

  int a = box.xmin;
  int b = box.xmax;
  int c = box.ymin;
  int d = box.ymax;

  // Scale vertical
  float sfact = (IMAGE_HEIGHT + ALPHA) * 1.0 / IMAGE_HEIGHT;
  int c_d = IMAGE_HEIGHT - (IMAGE_HEIGHT - c) * sfact;
  int d_d = IMAGE_HEIGHT - (IMAGE_HEIGHT - d) * sfact;

  // Scale horizontal
  sfact = abs(IMAGE_WIDTH / 2 + BETA) * 1.0 / IMAGE_WIDTH;
  int a_d = IMAGE_WIDTH / 2 - (IMAGE_WIDTH / 2 - a) * sfact;
  int b_d = IMAGE_WIDTH / 2 - (IMAGE_WIDTH / 2 - b) * sfact;

  box.xmin = a_d;
  box.xmax = b_d;
  box.ymin = c_d;
  box.ymax = d_d;
}

double kinect_fusion::estimate_distance(darknet_ros_msgs::BoundingBox &box,
    cv::Mat &depth_map) {
  const float KAPPA = 0.25; // Scale factor for centroid sampling radius
  const float KAPPA_LERP = 0.5; // Kappa scale factor when sampling fails to capture >0 points
  const int MAX_ALLOWED_LERPS = 5; // Maximum number of allowed kappa lerps

  // Adjust bounding box to IR FOV
  darknet_ros_msgs::BoundingBox box_adj = box;
  adjust_bounding_box(box);

  int box_width = box_adj.xmax - box_adj.xmin;
  int box_height = box_adj.ymax - box_adj.ymin;

  double estimate = 0;
  double kappa = KAPPA;
  int lerps = 0;

  // Loop until a nonzero estimate is made or we've exceeded max lerps
  while (estimate == 0 && lerps < MAX_ALLOWED_LERPS) {
    int radius = std::min(box_width, box_height) / 2 * KAPPA;
    int depth_points_processed = 0;
    double sigma_depth = 0;

    // Sum depths of points within sampling radius of the centroid
    for (int x = box_adj.xmin; x < box_adj.xmax; x++)
      for (int y = box_adj.ymin; y < box_adj.ymax; y++) {
        double dist = sqrt(pow(x - (box_adj.xmin + box_width / 2), 2) +
            pow(y - (box_adj.ymin + box_height / 2), 2));
        if (dist <= radius) {
          float depth = depth_map.at<float>(y, x, 0);
          if (!std::isnan(depth)) {
            sigma_depth += depth;
          #ifdef VISUALIZE
            depth_map.at<float>(y, x, 0) = 20;
          #endif
            depth_points_processed++;
          }
        }
      }

    estimate = depth_points_processed > 0 ?
        sigma_depth / depth_points_processed : 0;

    // If no depth points were processed, kappa likely wasn't large enough
    if (estimate == 0) {
      kappa += (1 - kappa) * KAPPA_LERP;
      lerps++;
    }
  }

  return estimate;
}

double kinect_fusion::estimate_distance(darknet_ros_msgs::BoundingBox &box,
    sensor_msgs::Image &img) {
  cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1);
  cv::Mat map = cv_ptr->image;
  double estimate = estimate_distance(box, map);

#ifdef VISUALIZE
  cv_bridge::CvImage out_msg;
  out_msg.header = img.header;
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = map;
  sensor_msgs::Image img_vis = *out_msg.toImageMsg();
  img.data = img_vis.data;
#endif

  return estimate;
}

std::pair<double, double> kinect_fusion::get_2d_offset(darknet_ros_msgs::BoundingBox &box,
    sensor_msgs::Image &img) {
  double distance = estimate_distance(box, img);

  if (distance == 0)
    return std::pair<double, double>(0, 0);

  int box_x = (box.xmin + box.xmax) / 2;
  int box_y = (box.ymin + box.ymax) / 2;

  float theta_x = (box_x - IMAGE_WIDTH / 2) * 1.0 / (IMAGE_WIDTH / 2)
                  * (KINECT_HORIZ_FOV / 2);
  float theta_y = (IMAGE_HEIGHT - box_y) * 1.0 / IMAGE_HEIGHT * KINECT_VERT_FOV;
  float x_comp_off = distance * cos(theta_y);
  float x_rel = x_comp_off * sin(theta_x);
  float y_rel = x_comp_off * cos(theta_x);

  return std::pair<double, double>(x_rel, y_rel);
}
