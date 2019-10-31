#include <ros/ros.h>

#include <bwi_scavenger/globals.h>
#include <bwi_scavenger_msgs/OccurrenceModel.h>
#include <bwi_scavenger_msgs/GetOccurrenceModel.h>
#include <bwi_scavenger_msgs/SaveOccurrenceModel.h>
#include <bwi_scavenger_msgs/ObjectProbabilities.h>

#include <std_msgs/String.h>

static std::vector<bwi_scavenger_msgs::ObjectProbabilities> obj_p;
static std::vector<std_msgs::String> obj_names;
static bool has_occurrence_model = false;

bool save_occurrence_model(bwi_scavenger_msgs::SaveOccurrenceModel::Request &req,
          bwi_scavenger_msgs::SaveOccurrenceModel::Response &res){

  if(req.model.num_objects == 0){
    ROS_INFO("[occurrence_model_services] Cannot save empty occurence model...");
    return false;
  }

  has_occurrence_model = false;
  
  obj_p.clear();
  obj_names.clear();

  for(bwi_scavenger_msgs::ObjectProbabilities op : req.model.objects)
    obj_p.push_back(op);

  for(std_msgs::String name : req.model.object_names)
    obj_names.push_back(name);
    
  has_occurrence_model = true;

  ROS_INFO("[occurrence_model_services] Saved new occurrence model");

  return true;
}

bool get_occurrence_model(bwi_scavenger_msgs::GetOccurrenceModel::Request &req,
          bwi_scavenger_msgs::GetOccurrenceModel::Response &res){

    if(!has_occurrence_model) {
      ROS_INFO("[occurrence_model_services] I don't have an occurrence model yet, please give me one");
      return false;
    }

    for(bwi_scavenger_msgs::ObjectProbabilities op : obj_p)
      res.model.objects.push_back(op);

    for(std_msgs::String name : obj_names)
      res.model.object_names.push_back(name);

    res.model.num_objects = obj_names.size();

    ROS_INFO("[occurrence_model_services] Sent back occurrence model");
    return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "occurrence_model_services");
  ros::NodeHandle nh;

  ros::ServiceServer srv_get_occurrence_model =
    nh.advertiseService(SRV_GET_OCCURRENCE_MODEL, get_occurrence_model);

  ros::ServiceServer srv_save_occurrence_model =
    nh.advertiseService(SRV_SAVE_OCCURRENCE_MODEL, save_occurrence_model);

  ROS_INFO("[occurrence_model_services] Standing by.");

  ros::spin();
}