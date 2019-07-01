#include <algorithm>
#include <bwi_scavenger/kinect_fusion.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>

#define VISUALIZE

static const float RADIANS_PER_DEGREE = 3.14159 / 180;
// Dimensions of Kinect color FOV in radians
static const float KINECT_HORIZ_FOV = 62 * RADIANS_PER_DEGREE;
static const float KINECT_VERT_FOV = 48.6 * RADIANS_PER_DEGREE;
// Kinect sensor resolution
static const int IMAGE_WIDTH = 640;
static const int IMAGE_HEIGHT = 480;
static const float VISUALIZE_DEPTH = 20;
// Extrema identification parameters
static const float FDEXT_MINIMUM_EXTREME = 0.25;
static const float FDEXT_INTERQUARTILE = 0.85;
static const int FDEXT_PARTITIONS = 32;
static const int FDEXT_THOROUGHNESS = 6;

int kinect_fusion::find_extreme(std::vector<float> &dat,
                                int partitions = FDEXT_PARTITIONS,
                                int thoroughness = FDEXT_THOROUGHNESS,
                                float minimum_extreme = FDEXT_MINIMUM_EXTREME) {
  int w = dat.size() * 0.5 / partitions, lower = 0, upper = dat.size() * 0.5;
  bool valid_extreme = false;

  // Levels of thoroughness
  for (int i = 0; i < thoroughness && w > 0; i++) {
    float record_high = std::numeric_limits<float>::min();
    int record_high_pos = -1;
    bool found_min_extreme = false;

    // Evaluate slopes of each partition
    for (int pos = lower; pos < upper; pos += w) {
      int pos_end = pos + w > upper ? upper - 1 : pos + w;
      float pos_d = std::isnan(dat[pos]) ? 0 : dat[pos];
      float pos_end_d = std::isnan(dat[pos_end]) ? 0 : dat[pos_end];
      float slope = fabs(pos_end_d - pos_d);
      if ((found_min_extreme || slope > FDEXT_MINIMUM_EXTREME) &&
          slope > record_high) {
        record_high = slope;
        record_high_pos = pos;
        found_min_extreme = true;
      }
    }

    // If a valid extreme wasn't found, we go no further
    if (record_high_pos != -1)
      valid_extreme = true;
    else
      break;

    // Record high partition becomes next search range
    lower = record_high_pos;
    upper = lower + w;
    w = (upper - lower) / partitions;
  }

  return valid_extreme ? (upper + lower) / 2 : 0;
}

void kinect_fusion::safe_depth_override(cv::Mat &depth_map, int r, int c,
    float d) {
  if (r >= 0 && r < depth_map.rows && c >= 0 && c < depth_map.cols)
    depth_map.at<float>(r, c, 0) = d;
}

void kinect_fusion::adjust_bounding_box(darknet_ros_msgs::BoundingBox &box) {
  const int ALPHA = 34; // # pixels between top of IR FOV and camera FOV
  const int BETA = 27; // # pixels between left/right of IR FOV and camera FOV

  int a = box.xmin;
  int b = box.xmax;
  int c = box.ymin;
  int d = box.ymax;

  // Scale vertical
  float sfact = (IMAGE_HEIGHT + ALPHA) * 1.0 / IMAGE_HEIGHT;
  int c_d = IMAGE_HEIGHT - (IMAGE_HEIGHT - c) * sfact;
  int d_d = IMAGE_HEIGHT - (IMAGE_HEIGHT - d) * sfact;

  // Scale horizontal
  sfact = abs(IMAGE_WIDTH / 2 + BETA) * 1.0 / (IMAGE_WIDTH / 2);
  int a_d = IMAGE_WIDTH / 2 - (IMAGE_WIDTH / 2 - a) * sfact;
  int b_d = IMAGE_WIDTH / 2 - (IMAGE_WIDTH / 2 - b) * sfact;

  box.xmin = a_d;
  box.xmax = b_d;
  box.ymin = c_d;
  box.ymax = d_d;
}

double kinect_fusion::estimate_distance(darknet_ros_msgs::BoundingBox &box,
    cv::Mat &depth_map) {
  // Adjust bounding box to IR FOV
  darknet_ros_msgs::BoundingBox focus_box = box;
  adjust_bounding_box(focus_box);

  // Ensure adjustment didn't clip the box outside map boundaries
  if (focus_box.xmin < 0) focus_box.xmin = 0;
  if (focus_box.xmax > depth_map.cols) focus_box.xmax = depth_map.cols;
  if (focus_box.ymin < 0) focus_box.ymin = 0;
  if (focus_box.ymax > depth_map.rows) focus_box.ymax = depth_map.rows;

  int box_width = focus_box.xmax - focus_box.xmin;
  int box_height = focus_box.ymax - focus_box.ymin;

  // To be populated with "extreme" depth points that will be removed
  std::vector<float> x_fdext, y_fdext;

  // Identify extreme vertical ranges
  for (int i = focus_box.xmin; i < focus_box.xmax; i++) {
    // Dump column data into a vector
    std::vector<float> col;
    for (int j = focus_box.ymin; j < focus_box.ymax; j++)
      col.push_back(depth_map.at<float>(j, i, 0));

    // Eliminate lower extreme
    int p = find_extreme(col);
    for (int j = 0; j <= p; j++) {
      x_fdext.push_back(i);
      y_fdext.push_back(focus_box.ymin + j);
    }

    // Eliminate upper extreme
    std::reverse(col.begin(), col.end());

    p = find_extreme(col);
    for (int j = 0; j <= p; j++) {
      x_fdext.push_back(i);
      y_fdext.push_back(focus_box.ymax - j);
    }
  }

  // Identify extreme horizontal ranges
  for (int i = focus_box.ymin; i < focus_box.ymax; i++) {
    // Dump row data into a vector
    std::vector<float> row;
    for (int j = focus_box.xmin; j < focus_box.xmax; j++)
      row.push_back(depth_map.at<float>(i, j, 0));

    // Eliminate lower extreme
    int p = find_extreme(row);
    for (int j = 0; j <= p; j++) {
      x_fdext.push_back(focus_box.xmin + j);
      y_fdext.push_back(i);
    }

    // Eliminate upper extreme
    std::reverse(row.begin(), row.end());

    p = find_extreme(row);
    for (int j = 0; j <= p; j++) {
      x_fdext.push_back(focus_box.xmax - j);
      y_fdext.push_back(i);
    }
  }

  // Remove identified extrema
  std::vector<std::vector<double>> points;

  for (int i = 0; i < x_fdext.size(); i++) {
    int x = x_fdext[i];
    int y = y_fdext[i];
    double d = depth_map.at<float>(y, x, 0);
    if (!std::isnan(d)) {
      std::vector<double> point;
      point.push_back(d);
      points.push_back(point);
      safe_depth_override(depth_map, y, x, NAN);
    }
  }

  // Clustering
  /*ROS_INFO("About to cluster %d points", points.size());
  Clusterer cl(points, 1);
  int num_clusters = cl.get_clusters(0.25, 10);

  if (num_clusters > 2) {
    int largest_cluster = cl.get_largest_cluster();
    for (int r = focus_box.ymin; r <= focus_box.ymax; r++)
      for (int c = focus_box.xmin; c <= focus_box.xmax; c++) {
        float d = depth_map.at<float>(r, c, 0);
        if (!std::isnan(d)) {
          std::vector<double> point;
          point.push_back(d);

          if (cl.in_cluster(point, largest_cluster))
            safe_depth_override(depth_map, r, c, NAN);
        }
      }
  }*/

  // Assemble a set of the remaining points to be sorted
  std::vector<float> point_set;
  for (int r = focus_box.ymin; r <= focus_box.ymax; r++)
    for (int c = focus_box.xmin; c <= focus_box.xmax; c++) {
      float d = depth_map.at<float>(r, c, 0);
      if (!std::isnan(d))
        point_set.push_back(d);
    }

  std::sort(point_set.begin(), point_set.end());

  // Remove the furthest and closest ranges of data
  int fringe_width = point_set.size() * ((1 - FDEXT_INTERQUARTILE) / 2);

  if (fringe_width > 0) {
    float lower_fringe = point_set[fringe_width];
    float upper_fringe = point_set[point_set.size() - fringe_width];

    for (int r = focus_box.ymin; r <= focus_box.ymax; r++)
      for (int c = focus_box.xmin; c <= focus_box.xmax; c++) {
        float d = depth_map.at<float>(r, c, 0);
        if (d <= lower_fringe || d >= upper_fringe)
          safe_depth_override(depth_map, r, c, NAN);
      }
  }

  // Average all remaining points for the final answer
  double total_depth = 0;
  int depth_points_processed = 0;
  for (int r = focus_box.ymin; r < focus_box.ymax; r++)
    for (int c = focus_box.xmin; c < focus_box.xmax; c++) {
      float d = depth_map.at<float>(r, c, 0);
      if (!std::isnan(d)) {
        total_depth += d;
        depth_points_processed++;
      #ifdef VISUALIZE
        safe_depth_override(depth_map, r, c, VISUALIZE_DEPTH);
      #endif
      }
    }
  double estimate = total_depth / depth_points_processed;
  return std::isnan(estimate) ? 0 : estimate;
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
