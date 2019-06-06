#include <ros/ros.h>
#include <cstdlib>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <math.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <actionlib/client/simple_action_client.h>

#define DISTANCE 0.5
#define STEP_METERS 1
#define AVOID_WALL_METERS 2

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveRobot {
public:
    // nav_msgs::MapMetaData gridInfo;
    // int8_t const *gridData;
    // random_move::array gridData;
    nav_msgs::OccupancyGrid _grid;
    bool storedMap = false;
    bool sentGoal = false;
    bool center = false;
    bool storedinitialPose = false;
    int right = 0;
    int left = 0;
    int up = 0;
    int down = 0;
    int width;
    int height;
    int count = 0;
    move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::Pose initialPose;
    tf::TransformListener tfL;
    MoveBaseClient ac;
    std::string gridFrameId;
    bool moving = false;
    geometry_msgs::Point *searchPoints;
    int searchPointsLength;

    MoveRobot() : ac("move_base", true) {
        while(!ac.waitForServer(ros::Duration(5.0))){   
            ROS_INFO("Wating for the move_base action server to come up");
        }
    }


    // stores map
    void map(const nav_msgs::OccupancyGrid::ConstPtr &grid){
        std::cout << "map" << std::endl;
        // gridInfo = grid -> info;
        // gridData = &grid->data[0];
        _grid = *grid;
        storedMap = true;
        // make robot move forward a little so it can start looking
        
        gridFrameId = _grid.header.frame_id;
        geometry_msgs::Pose gridOrigin = _grid.info.origin;
        tf::StampedTransform initialBaseTransform;

        tfL.waitForTransform(_grid.header.frame_id, "base_link", ros::Time::now(), ros::Duration(4));
        tfL.lookupTransform(_grid.header.frame_id, "base_link", ros::Time(0), initialBaseTransform);

        initialPose.position.x = initialBaseTransform.getOrigin().getX();
        initialPose.position.y = initialBaseTransform.getOrigin().getY();
        initialPose.position.z = initialBaseTransform.getOrigin().getZ();

        initialPose.orientation.x = initialBaseTransform.getRotation().getX();
        initialPose.orientation.y = initialBaseTransform.getRotation().getY();
        initialPose.orientation.z = initialBaseTransform.getRotation().getZ();
        initialPose.orientation.w = initialBaseTransform.getRotation().getW();
    }

    void findObject(const std_msgs::String::ConstPtr &string){
        if(string->data == "find") {
            explore();
        } else if (string->data == "found") {
            std::cout << "go back home" << std::endl;
            returnToInitial();
        }
    }

    // bool stillLooking(){
    //     for(int i = 0; i < _grid.info.width * _grid.info.height; i++){
    //         if(_grid.data[i] == 0) {
    //             return true;
    //         }
    //     }
    //     return false;
    // }

    void sendGoal(geometry_msgs::PoseStamped tag_pose) {
        geometry_msgs::PoseStamped tag_rel_pose;

        tfL.waitForTransform("base_link", tag_pose.header.frame_id, ros::Time::now(), ros::Duration(4));
        tfL.transformPose("base_link", tag_pose, tag_rel_pose);
        
        tag_rel_pose.pose.position.z = 0;
        goal.target_pose = tag_rel_pose;
        std::cout << tag_rel_pose << std::endl;
        ac.sendGoal(goal);
        ac.waitForResult();
        std::cout << "reached goal!" << std::endl;
    }

    void returnToInitial() {
        ac.waitForResult();
        geometry_msgs::PoseStamped homePose;
        homePose.header.seq = 0;
        homePose.header.stamp = ros::Time::now();
        homePose.header.frame_id = gridFrameId;
        homePose.pose = initialPose;

        sendGoal(homePose);
    }

    void generatePoints(){
        std::cout << "start generating points" << std::endl;
        // robot moving back and forth across the width
        int searchWidth = width - (2 * AVOID_WALL_METERS) / _grid.info.resolution;
        int searchHeight = height - (2 * AVOID_WALL_METERS) / _grid.info.resolution;
        int gridStep = STEP_METERS / _grid.info.resolution;

        int searchSteps = searchHeight / gridStep;

        searchPointsLength = searchSteps * 2;
        searchPoints = new geometry_msgs::Point[searchPointsLength];
        // know that searchPoints is even
        for (int i = 0; i < searchSteps; i++) {
            int firstPointIndex = 2 * i;
            int secondPointIndex = firstPointIndex + 1;
            bool isOdd = i & 1;
            
            int yVal = gridStep * i;
            
            searchPoints[firstPointIndex].x = isOdd ? 0 : searchWidth;
            searchPoints[firstPointIndex].y = yVal;
            searchPoints[secondPointIndex].x = isOdd ? searchWidth : 0;
            searchPoints[secondPointIndex].y = yVal;
        }
    }

    void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
        moving = false;
    }

    void explore(){
        if (moving)
            return;

        enum{
            CALC = 0,
            MOVE = 1
        };

        static int state = CALC;
        static int searchPointIndex = 0;
        nav_msgs::MapMetaData gridInfo = _grid.info;
        
        if(state == CALC){
            
            int row = round((initialPose.position.y - gridInfo.origin.position.y) / gridInfo.resolution);
            int col = round((initialPose.position.x - gridInfo.origin.position.x) / gridInfo.resolution);

            // initial point converted onto the occupancy grid
            int initialGridPoint = (row * gridInfo.width) + col;

            while(_grid.data[initialGridPoint + right] == 0)
                right++;
            while(_grid.data[initialGridPoint - left] == 0)
                left++;
            while(_grid.data[initialGridPoint + (gridInfo.width * up)] == 0)
                up++;
            while(_grid.data[initialGridPoint - (gridInfo.width * down)] == 0)
                down++;
            width = left + right;
            height = up + down; 

            generatePoints();
            state = MOVE;
            return;
        }

        if (searchPointIndex == searchPointsLength)
            return;

        // this is the point that the robot is at 
        geometry_msgs::Point initialCoorPoint;
        initialCoorPoint.x = initialPose.position.x;
        initialCoorPoint.y = initialPose.position.y;

        // change this to be to the corner and not the center
        geometry_msgs::PoseStamped goalPose;
        
        goalPose.pose.position.x = initialCoorPoint.x - left * gridInfo.resolution;
        goalPose.pose.position.y = initialCoorPoint.y - down * gridInfo.resolution;
        // it's at the corner of the walls now

        goalPose.pose.position.x += AVOID_WALL_METERS;
        goalPose.pose.position.y += AVOID_WALL_METERS;
        // its now at the corner of the search box

        geometry_msgs::Point searchGridPoint = searchPoints[searchPointIndex];
        goalPose.pose.position.x += searchGridPoint.x * gridInfo.resolution;
        goalPose.pose.position.y += searchGridPoint.y * gridInfo.resolution;


        goalPose.header.seq = 0;
        goalPose.header.stamp = ros::Time::now();
        goalPose.header.frame_id = gridFrameId;
        goalPose.pose.orientation.w = 1;

        geometry_msgs::PoseStamped tag_rel_pose;

        tfL.waitForTransform("base_link", goalPose.header.frame_id, ros::Time::now(), ros::Duration(4));
        tfL.transformPose("base_link", goalPose, tag_rel_pose);
        
        tag_rel_pose.pose.position.z = 0;
        goal.target_pose = tag_rel_pose;

        ac.sendGoal(goal, boost::bind(&MoveRobot::doneCb, this, _1, _2));
        moving = true;
        searchPointIndex++;
    }


    // go to center and rotate stuff
    void explore2(){
        if (moving)
            return;
    
        enum {
            CENTER = 0,
            ROTATE = 1
        };
        
        static int state2 = CENTER;

        if (state2 == CENTER) {

            std::cout << "intialCoorPoint: (" << initialPose.position.x << ", " << initialPose.position.y << ")"<< std::endl;

            nav_msgs::MapMetaData gridInfo = _grid.info;
            
            int row = round((initialPose.position.y - gridInfo.origin.position.y) / gridInfo.resolution);
            int col = round((initialPose.position.x - gridInfo.origin.position.x) / gridInfo.resolution);

            // initial point converted onto the occupancy grid
            int initialGridPoint = (row * gridInfo.width) + col;

            while(_grid.data[initialGridPoint + right] == 0)
                right++;
            while(_grid.data[initialGridPoint - left] == 0)
                left++;
            while(_grid.data[initialGridPoint + (gridInfo.width * up)] == 0)
                up++;
            while(_grid.data[initialGridPoint - (gridInfo.width * down)] == 0)
                down++;
            width = left + right;
            height = up + down; 

            // this is the point that the robot is at 
            geometry_msgs::Point initialCoorPoint;
            initialCoorPoint.x = initialPose.position.x;
            initialCoorPoint.y = initialPose.position.y;

            geometry_msgs::PoseStamped goalPose;
            
            goalPose.header.seq = 0;
            goalPose.header.stamp = ros::Time::now();
            goalPose.header.frame_id = gridFrameId;
            goalPose.pose.orientation.w = 1;
            goalPose.pose.position.x = initialCoorPoint.x - ((left - (width / 2)) * gridInfo.resolution);
            goalPose.pose.position.y = initialCoorPoint.y - ((down - (height / 2)) * gridInfo.resolution);

            sendGoal(goalPose);
            state2 = ROTATE;
            return;
        }
        
        tf::Quaternion rotation(0, 0, M_PI_2);
        rotation.normalize();
        // std::cout << rotation.length() << std::endl;

        geometry_msgs::PoseStamped rotationPose;
        rotationPose.header.seq = 0;
        rotationPose.header.stamp = ros::Time::now();
        rotationPose.header.frame_id = "base_link";
        rotationPose.pose.orientation.x = rotation.getX();
        rotationPose.pose.orientation.y = rotation.getY();
        rotationPose.pose.orientation.z = rotation.getZ();
        rotationPose.pose.orientation.w = rotation.getW();
        rotationPose.pose.position.x = rotationPose.pose.position.y = rotationPose.pose.position.z = 0;
       
        std::cout << "rotate" << std::endl;
        goal.target_pose = rotationPose;
        ac.sendGoal(goal, boost::bind(&MoveRobot::doneCb, this, _1, _2));
        moving = true;
    }
};




int main(int argc, char *argv[]){
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle node;

    MoveRobot mr;
    
    ros::Subscriber mapSub = node.subscribe("/level_mux/map", 1, &MoveRobot::map, &mr);

    ros::Subscriber foundObjectSub = node.subscribe("findObject", 10, &MoveRobot::findObject, &mr);
    // ros::Subscriber locationSub = node.subscribe("/amcl_pose", 1, &MoveRobot::explore, &mr);

    ros::spin();
    return 0;
}

