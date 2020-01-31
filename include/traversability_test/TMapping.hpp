/*
 * TMapping.hpp
 */

// Grid Map
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/GetGridMapInfo.h>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_ros/grid_map_ros.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS

// ROS
//#include <filters/filter_chain.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// STD
#include <string>
#include <vector>

namespace traversability_test {

class TMapping {

  public:
    TMapping(ros::NodeHandle& nodeHandle);
    ~TMapping();
  private:
    void readParameters();

    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! Image subscriber.
    image_transport::ImageTransport *it_;
    image_transport::CameraSubscriber imageSubscriber_;
//    ros::Subscriber imageSubscriber_;
    std::string imageTopic_;
//    double imageResolution_;
//    double imageMinHeight_;
//    double imageMaxHeight_;

    //! Pose subscriber.
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> robotPoseSubscriber_;
    std::string robotPoseTopic_;
    int robotPoseCacheSize_;

    //! Cache for the robot pose messages.
    message_filters::Cache<geometry_msgs::PoseWithCovarianceStamped> robotPoseCache_;

    //! Grid Map topic to initialize traversability map.
    ros::Subscriber gridMapToInitTraversabilityMapSubscriber_;
    std::string gridMapToInitTraversabilityMapTopic_;

    // Callbacks.
    void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr & infomsg);
    void gridMapToInitTraversabilityMapCallback(const grid_map_msgs::GridMap& message);

};//class

}  // namespace traversability_test
