#include <ros/ros.h>

#include <bwi_scavenger/globals.h>
#include <bwi_scavenger_msgs/OccurrenceModel.h>
#include <bwi_scavenger_msgs/ObjectProbabilities.h>
#include <bwi_scavenger_msgs/SaveOccurrenceModel.h>

#include <std_msgs/String.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "occurrence_model_node");
  ros::NodeHandle nh;

  ROS_INFO("[occurrence_model_node] Standing by.");

  ros::ServiceClient save_model = nh.serviceClient<bwi_scavenger_msgs::SaveOccurrenceModel>(SRV_SAVE_OCCURRENCE_MODEL);

  bwi_scavenger_msgs::SaveOccurrenceModel model;
  
  // saving object names
  std_msgs::String string;
  string.data = "chair";
  model.request.model.object_names.push_back(string);

  string.data = "bottle";
  model.request.model.object_names.push_back(string);
  model.request.model.num_objects = 2;

  // saving each object specific occurrence model
  std::vector<bwi_scavenger_msgs::ObjectProbabilities> op;


  geometry_msgs::Point point;

  // chair
  bwi_scavenger_msgs::ObjectProbabilities op_1;

  point.x = -30.1798;
  point.y = -4.5981;
  op_1.locations.push_back(point);
  op_1.probabilities.push_back(0.6);

  point.x = -11.365;
  point.y = -12.253;
  op_1.locations.push_back(point);
  op_1.probabilities.push_back(0.4);

  op_1.num_probabilities = 2;

  model.request.model.objects.push_back(op_1);

  //bottle
  bwi_scavenger_msgs::ObjectProbabilities op_2;

  point.x = -30.1798;
  point.y = -4.5981;
  op_2.locations.push_back(point);
  op_2.probabilities.push_back(0.1);

  point.x = -16.658;
  point.y = -12.13;
  op_2.locations.push_back(point);
  op_2.probabilities.push_back(0.4);

  point.x = -47.5810;
  point.y = -4.1472;
  op_2.locations.push_back(point);
  op_2.probabilities.push_back(0.5);

  op_2.num_probabilities = 3;

  model.request.model.objects.push_back(op_2);

  save_model.call(model);

  // ros::spin();
}