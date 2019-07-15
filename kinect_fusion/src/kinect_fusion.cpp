#include <algorithm>
#include <iostream>
#include <math.h>

#include "kinect_fusion/kinect_fusion.h"

/**
  Uncomment to get a visualization of the algorithm's depth map processing.
  A marked-up depth map will be published to /kinect_fusion/vis on every call
  to estimate_distance. Visualized as a 32F1C image, every pixel in the
  specified bounding box will be either white or black. Black pixels were
  labeled unlikely to belong to the target object, and vice versa for white
  pixels.
*/
#define VISUALIZE
#define RADIANS_PER_DEGREE 3.14159 / 180;

// Kinect sensor parameters
static const float KINECT_HORIZ_FOV = 62 * RADIANS_PER_DEGREE;
static const float KINECT_VERT_FOV  = 48.6 * RADIANS_PER_DEGREE;
static const int   IMAGE_WIDTH      = 640;
static const int   IMAGE_HEIGHT     = 480;

// Approximate pixel discrepancies between the bounds of the IR and color images
static const int KINECT_FOV_ALPHA = 34;
static const int KINECT_FOV_BETA  = 27;

#ifdef VISUALIZE
  #include <ros/ros.h>

  static const float VISUALIZE_DEPTH = 20;
  static ros::NodeHandle *nh = nullptr;
  static ros::Publisher pub_vis;
#endif

namespace kinect_fusion {

/**
  Default extrema identification parameters. Freely reconfigurable at compile
  or run time.
*/
unsigned int extrema_search_partitions        = 16;   // 16 partitions per iteration
unsigned int extrema_search_thoroughness      = 3;    // 3 iterations per search
unsigned int erode_dilate_strength            = 10;   // 10 layers of erode/dilate
unsigned int simplicity_threshold             = 3000; // 3000px size threshold
float        extrema_search_minimum_magnitude = 0.15; // 0.15 m between partition ends
float        interquartile_trim_width         = 0.85; // Avg inner 85%

// Private utilities
namespace {
  /**
    A binary interpolation-esque search for the point representative of the
    region of greatest change within a vector.

    @param  dat          vector to analyze
    @param  partitions   number of partitions in interpolation search
    @param  thoroughness maximum depth of search
    @param  min_mag      minimum magnitude of an "extreme" value
    @return index of greatest change
  */
  int find_extreme(std::vector<float> &dat,
                   unsigned int partitions = extrema_search_partitions,
                   unsigned int thoroughness = extrema_search_thoroughness,
                   float min_mag = extrema_search_minimum_magnitude)
  {
    int w = dat.size() * 0.5 / partitions, lower = 0, upper = dat.size() * 0.5;
    bool valid_extreme = false;

    // Levels of thoroughness
    for (int i = 0; i < thoroughness && w > 0; i++) {
      float record_high = std::numeric_limits<float>::min();
      int record_high_pos = -1;

      // Evaluate slopes of each partition
      for (int pos = lower; pos < upper; pos += w) {
        int pos_end = pos + w > upper ? upper - 1 : pos + w;
        float pos_d = std::isnan(dat[pos]) ? 0 : dat[pos];
        float pos_end_d = std::isnan(dat[pos_end]) ? 0 : dat[pos_end];
        float slope = fabs(pos_end_d - pos_d);
        if (slope > min_mag && (record_high_pos == -1 || slope > record_high)) {
          record_high = slope;
          record_high_pos = pos;
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

  /**
    @brief matrix editing with bounds checking
  */
  void safe_depth_override(cv::Mat &depth_map,
                           unsigned int r,
                           unsigned int c,
                           float d)
  {
    if (r < depth_map.rows && c < depth_map.cols)
      depth_map.at<float>(r, c, 0) = d;
  }

  /**
    @brief matrix indexing with bounds checking
  */
  float safe_depth_sample(cv::Mat &depth_map,
                          unsigned int r,
                          unsigned int c) {
    if (r < depth_map.rows && c < depth_map.cols)
      return depth_map.at<float>(r, c, 0);
    else
      return 0;
  }

  /**
    @brief projects a bounding box relative to the Kinect's camera FOV onto
           its IR sensor FOV
  */
  void adjust_bounding_box(darknet_ros_msgs::BoundingBox &box)
  {
    int a = box.xmin;
    int b = box.xmax;
    int c = box.ymin;
    int d = box.ymax;

    // Scale vertical
    float sfact = (IMAGE_HEIGHT + KINECT_FOV_ALPHA) * 1.0 / IMAGE_HEIGHT;
    int c_d = IMAGE_HEIGHT - (IMAGE_HEIGHT - c) * sfact;
    int d_d = IMAGE_HEIGHT - (IMAGE_HEIGHT - d) * sfact;

    // Scale horizontal
    sfact = abs(IMAGE_WIDTH / 2 + KINECT_FOV_BETA) * 1.0 / (IMAGE_WIDTH / 2);
    int a_d = IMAGE_WIDTH / 2 - (IMAGE_WIDTH / 2 - a) * sfact;
    int b_d = IMAGE_WIDTH / 2 - (IMAGE_WIDTH / 2 - b) * sfact;

    box.xmin = a_d;
    box.xmax = b_d;
    box.ymin = c_d;
    box.ymax = d_d;
  }

  /**
    @brief a simpler method of estimating proximity used only for very small
           bounding boxes; averages non-NaN points within a circle centered in
           the bounding box with radius argmin(box width, box height)
  */
  float simple_estimate(const darknet_ros_msgs::BoundingBox &box,
                        cv::Mat &depth_map)
  {
    double depth_total = 0;
    unsigned int depth_points_processed = 0;
    unsigned int radius = std::min(box.xmax - box.xmin, box.ymax - box.ymin)
                          / 2;
    unsigned int rc = (box.ymin + box.ymax) / 2;
    unsigned int cc = (box.xmin + box.xmax) / 2;

    for (int r = box.ymin; r < box.ymax; r++)
      for (int c = box.xmin; c < box.xmax; c++) {
        int dist = sqrt(pow(r - rc, 2) + pow(c - cc, 2));
        float depth = safe_depth_sample(depth_map, r, c);
        if (dist <= radius && !std::isnan(depth)) {
          depth_total += depth;
          depth_points_processed++;
        #ifdef VISUALIZE
          safe_depth_override(depth_map, r, c, VISUALIZE_DEPTH);
        #endif
        }
      }

  #ifdef VISUALIZE
    cv_bridge::CvImage bridge;
    bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    bridge.image = depth_map;
    sensor_msgs::Image img_vis = *bridge.toImageMsg();
    pub_vis.publish(img_vis);
  #endif

    return depth_total / depth_points_processed;
  }
}

double estimate_distance(const darknet_ros_msgs::BoundingBox &box,
                         cv::Mat depth_map)
{
#ifdef VISUALIZE
  if (nh == nullptr) {
    nh = new ros::NodeHandle();
    pub_vis = nh->advertise<sensor_msgs::Image>("kinect_fusion/vis", 1);
    ros::Duration(3.0).sleep();
  }
#endif

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
  int box_area = box_width * box_height;

  std::cout << box_area << std::endl;

  if (box_area < simplicity_threshold)
    return simple_estimate(box, depth_map);

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
  for (int i = 0; i < x_fdext.size(); i++)
    safe_depth_override(depth_map, y_fdext[i], x_fdext[i], NAN);

  // Assemble a set of the remaining points to be sorted
  std::vector<float> point_set;

  for (int r = focus_box.ymin; r < focus_box.ymax; r++)
    for (int c = focus_box.xmin; c < focus_box.xmax; c++) {
      float d = depth_map.at<float>(r, c, 0);
      if (!std::isnan(d))
        point_set.push_back(d);
    }

  std::sort(point_set.begin(), point_set.end());

  // Remove the furthest and closest ranges of data
  int fringe_width = point_set.size() * ((1 - interquartile_trim_width) / 2);

  if (fringe_width > 0) {
    float lower_fringe = point_set[fringe_width];
    float upper_fringe = point_set[point_set.size() - fringe_width];

    for (int r = focus_box.ymin; r < focus_box.ymax; r++)
      for (int c = focus_box.xmin; c < focus_box.xmax; c++) {
        float d = depth_map.at<float>(r, c, 0);
        if (d <= lower_fringe || d >= upper_fringe)
          safe_depth_override(depth_map, r, c, NAN);
      }
  }

  // Erode the mask
  std::vector<int> x_erode, y_erode;
  cv::Mat temp = depth_map.colRange(focus_box.xmin, focus_box.xmax)
                          .rowRange(focus_box.ymin, focus_box.ymax);
  depth_map.copyTo(temp);

  for (int i = 0; i < erode_dilate_strength; i++) {
    for (int r = focus_box.ymin; r < focus_box.ymax; r++)
      for (int c = focus_box.xmin; c < focus_box.xmax; c++) {
        float d = depth_map.at<float>(r, c, 0);
        if (!std::isnan(d)) {
          int neighbors = (safe_depth_sample(depth_map, r + 1, c) > 0) +
                          (safe_depth_sample(depth_map, r - 1, c) > 0) +
                          (safe_depth_sample(depth_map, r, c + 1) > 0) +
                          (safe_depth_sample(depth_map, r, c - 1) > 0);
          if (neighbors != 4) {
            x_erode.push_back(c);
            y_erode.push_back(r);
          }
        }
      }

    for (int i = 0; i < x_erode.size(); i++)
      safe_depth_override(depth_map, y_erode[i], x_erode[i], NAN);

    x_erode.clear();
    y_erode.clear();
  }

  // Dilate the mask
  for (int i = 0; i < erode_dilate_strength; i++) {
    for (int r = focus_box.ymin; r < focus_box.ymax; r++) {
      for (int c = focus_box.xmin; c < focus_box.xmax; c++) {
        float d = depth_map.at<float>(r, c, 0);

        if (std::isnan(d)) {
          if (!std::isnan(safe_depth_sample(depth_map, r + 1, c)) ||
              !std::isnan(safe_depth_sample(depth_map, r - 1, c)) ||
              !std::isnan(safe_depth_sample(depth_map, r, c + 1)) ||
              !std::isnan(safe_depth_sample(depth_map, r, c - 1)))
            safe_depth_override(depth_map, r, c, safe_depth_sample(temp, r, c));
        }
      }
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

#ifdef VISUALIZE
  cv_bridge::CvImage bridge;
  bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  bridge.image = depth_map;
  sensor_msgs::Image img_vis = *bridge.toImageMsg();
  pub_vis.publish(img_vis);
#endif

  double estimate = total_depth / depth_points_processed;
  return std::isnan(estimate) ? 0 : estimate;
}

double estimate_distance(const darknet_ros_msgs::BoundingBox &box,
                         const sensor_msgs::Image &img)
{
  cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1);
  cv::Mat map = cv_ptr->image;
  double estimate = estimate_distance(box, map);

  return estimate;
}

geometry_msgs::Point get_position(const darknet_ros_msgs::BoundingBox &box,
                                  const sensor_msgs::Image &img)
{
  double distance = estimate_distance(box, img);

  if (distance == 0)
    return geometry_msgs::Point();

  int box_x = (box.xmin + box.xmax) / 2;
  int box_y = (box.ymin + box.ymax) / 2;

  float theta_xy = (box_x - IMAGE_WIDTH / 2) * 1.0 / (IMAGE_WIDTH / 2)
                  * (KINECT_HORIZ_FOV / 2);
  float theta_z = -(box_y - IMAGE_HEIGHT / 2) * 1.0 / (IMAGE_HEIGHT / 2)
                  * (KINECT_VERT_FOV / 2);
  float x_comp_off = distance * cos(theta_z);
  float y_rel = x_comp_off * sin(theta_xy);
  float x_rel = x_comp_off * cos(theta_xy);
  float z_rel = distance * sin(theta_z);

  geometry_msgs::Point position;
  position.x = x_rel;
  position.y = y_rel;
  position.z = z_rel;

  return position;
}

}; // end namespace kinect_fusion
