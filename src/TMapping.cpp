///*
// * TMapping.cpp
// */

#include "traversability_test/TMapping.hpp"

// ROS
#include <geometry_msgs/Pose.h>
#include <ros/package.h>

using namespace std;

namespace traversability_test {

    TMapping::TMapping(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
    {

        readParameters();

        it_ = new image_transport::ImageTransport(nodeHandle_);

//        imageSubscriber_ = nodeHandle_.subscribe(imageTopic_, 1, &TMapping::imageCallback, this);
        imageSubscriber_ = it_->subscribeCamera(imageTopic_, 10,&TMapping::imageCallback, this);

        robotPoseSubscriber_.subscribe(nodeHandle_, robotPoseTopic_, 1);
        robotPoseCache_.connectInput(robotPoseSubscriber_);
        robotPoseCache_.setCacheSize(robotPoseCacheSize_);

        gridMapToInitTraversabilityMapSubscriber_ = nodeHandle_.subscribe(
                gridMapToInitTraversabilityMapTopic_, 1, &TMapping::gridMapToInitTraversabilityMapCallback, this);

    }

    TMapping::~TMapping()
    {
    }


    void TMapping::readParameters()
    {
        // Read parameters for image subscriber.
        nodeHandle_.param("image_topic", imageTopic_, string("/image_elevation"));

        // Read parameters for pose subscriber.
        nodeHandle_.param("robot_pose_with_covariance_topic", robotPoseTopic_, string("/pose"));
        nodeHandle_.param("robot_pose_cache_size", robotPoseCacheSize_, 200);

        // Read parameters for grid map subscriber.
        nodeHandle_.param("grid_map_topic_name", gridMapToInitTraversabilityMapTopic_, string("traversability_estimation/traversability_map"));

    }

    void TMapping::imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr & infomsg) {
//        //image processing
//        cv_bridge::CvImagePtr cv_ptr;
//        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
//        left_image = cv_ptr->image;       
//        ancient_width = left_image.cols;
//        cv::resize(left_image, left_image, cv::Size(), scale, scale);
//        Left_received = true; 
//        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
//        clahe->setClipLimit(5);
//        clahe->apply(left_image,left_image);

//        //camera info
//        P0 = Map<const MatrixXd>(&infomsg->P[0], 3, 4);  
    }

    void TMapping::gridMapToInitTraversabilityMapCallback(const grid_map_msgs::GridMap& message) {
      grid_map::GridMap gridMap;
      grid_map::GridMapRosConverter::fromMessage(message, gridMap);
    }

}  // namespace traversability_test
