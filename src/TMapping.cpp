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
    : nodeHandle_(nodeHandle), octree_(128.0f), isCloudIn(false)
    {

        readParameters();

        // Subscribers
//        it_ = new image_transport::ImageTransport(nodeHandle_);
//        depthSubscriber_ = it_->subscribeCamera(depthTopic_, 1, &TMapping::depthCallback, this);
//        imageSubscriber_ = it_->subscribeCamera(imageTopic_, 1, &TMapping::imageCallback, this);

        image_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nodeHandle_, imageTopic_, 10);
        depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (nodeHandle_, depthTopic_, 10);
        info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo> (nodeHandle_, cameraTopic_, 10);

        sync_ = new message_filters::Synchronizer<MySyncPolicy> (MySyncPolicy(100), *image_sub_, *depth_sub_, *info_sub_);
        sync_->registerCallback(boost::bind(&TMapping::imagesCallback, this, _1, _2, _3));

//        message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> sync(image_sub, depth_sub, info_sub, 1);
//        sync.registerCallback(boost::bind(&TMapping::imagesCallback, this, _1, _2, _3));

//        velodyneSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &TMapping::pointCloudCallback, this);

        gridMapToInitTraversabilityMapSubscriber_ = nodeHandle_.subscribe(gridMapToInitTraversabilityMapTopic_, 1, &TMapping::gridMapToInitTraversabilityMapCallback, this);

    }

    TMapping::~TMapping()
    {
    }


    void TMapping::readParameters()
    {
        // Read parameters for image subscriber.
        nodeHandle_.param("image_topic", imageTopic_, string("/camera/color/image_raw"));
        nodeHandle_.param("depth_topic", depthTopic_, string("/camera/aligned_depth_to_color/image_raw"));
        nodeHandle_.param("camera_topic", cameraTopic_, string("/camera/color/camera_info"));
        nodeHandle_.param("camera_base", cameraBase_, string("camera_color_optical_frame"));
        nodeHandle_.param("pointcloud_topic", pointCloudTopic_, string("/velodyne_points"));
        nodeHandle_.param("velodyne_topic", velodyneTopic_, string("velodyne_actual"));

        // Read parameters for pose subscriber.
        nodeHandle_.param("robot_pose_with_covariance_topic", robotPoseTopic_, string("/pose"));
        nodeHandle_.param("robot_pose_cache_size", robotPoseCacheSize_, 200);

        // Read parameters for grid map subscriber.
        nodeHandle_.param("grid_map_topic_name", gridMapToInitTraversabilityMapTopic_, string("/traversability_estimation/traversability_map"));
//        nodeHandle_.param("grid_map_topic_name", gridMapToInitTraversabilityMapTopic_, string("/elevation_mapping/elevation_map"));

        cout<<gridMapToInitTraversabilityMapTopic_<<endl;

    }

    void TMapping::imagesCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr & infomsg){

        if(msg1->header.stamp.toNSec() > gridMap_.getTimestamp()){
            if (!gridMapContainer_.empty())
            {
                gridMap_ = gridMapContainer_.front();//.pop_front();
                gridMapContainer_.erase(gridMapContainer_.begin());
            }
        }
//        cout<<"images test"<<endl;
        //image processing
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::RGB8);
        image_ = cv_ptr->image;   

        sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
        cloud_msg->header = msg->header;
        cloud_msg->height = msg->height;
        cloud_msg->width  = msg->width;
        cloud_msg->is_dense = false;
        cloud_msg->is_bigendian = false;

        sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
        pcd_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::FLOAT32);

        // Update camera model
        image_geometry::PinholeCameraModel camModel;
        camModel.fromCameraInfo(infomsg);
 
        depth_image_proc::convert<uint16_t>(msg, cloud_msg, camModel);


        sensor_msgs::PointCloud submap_cloud;
        sensor_msgs::PointCloud submapTransformed_cloud;
        sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msg, submap_cloud);
        //To gridMap_ frame        
        try {
            transformListener_.transformPointCloud("/map", msg->header.stamp, submap_cloud, cameraBase_, submapTransformed_cloud);
        } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return;
        }
        sensor_msgs::convertPointCloudToPointCloud2(submapTransformed_cloud, velodyne_cloud_);


        sensor_msgs::PointCloud2Iterator<float> iter_x(velodyne_cloud_, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(velodyne_cloud_, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(velodyne_cloud_, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(velodyne_cloud_, "intensity");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
        {
            grid_map::Index index;
            grid_map::Position position(*iter_x, *iter_y);
            if (!gridMap_.getIndex(position, index)) continue;
            *iter_intensity = gridMap_.at("traversability", index);
        }

//        pcl::PointCloud<pcl::PointXYZI> input;
//        pcl::fromROSMsg(velodyne_cloud_, input);
//        cv::Mat proj_img(cv::Size(infomsg->width, infomsg->height), CV_32FC1);
//        for (size_t i = 0; i < infomsg->height; ++i)
//        {
//            for (size_t j = 0; j < infomsg->width; ++j){
//                proj_img.at<float>(i,j) = input.points[i*infomsg->width+j].intensity;
//            }
//        }
//        cv::Mat binary_img; 
//        cv::threshold( proj_img, binary_img, 0.5, 1.0,0);
//        std::string file_name = "./imgs/" + std::to_string(msg1->header.stamp.toNSec()) + ".png" ;
//        imwrite(file_name, binary_img);

        pcl::PointCloud<pcl::PointXYZI> input;
        pcl::fromROSMsg(velodyne_cloud_, input);
        cv::Mat proj_img(cv::Size(infomsg->width, infomsg->height), CV_32FC1);
        for (size_t i = 0; i < infomsg->height; ++i)
        {
            for (size_t j = 0; j < infomsg->width; ++j){
                proj_img.at<float>(i,j) = input.points[i*infomsg->width+j].intensity;
            }
        }
//        cv::Mat binary_img; 
//        cv::threshold( proj_img, binary_img, 0.5, 1.0,0);
        std::string file_name = "./imgs/" + std::to_string(msg1->header.stamp.toNSec()) + ".png" ;
        imwrite(file_name, proj_img);

//        pcl::PointCloud<pcl::PointXYZI> input;
//        pcl::fromROSMsg(velodyne_cloud_, input);
//        cv::Mat proj_img(cv::Size(infomsg->width, infomsg->height), CV_8UC3,cv::Scalar(255,255,255));
//        for (size_t i = 0; i < infomsg->height; ++i)
//        {
//            for (size_t j = 0; j < infomsg->width; ++j){
//            proj_img.at<cv::Vec3b>(i,j) = cv::Vec3b(255.0*input.points[i*infomsg->width+j].intensity,0,255.0*(1.0-input.points[i*infomsg->width+j].intensity));
//            }
//        }
//        cv::Mat added_image; 
//        cv::addWeighted(image_,0.5,proj_img,0.5,0,added_image);
//        imwrite("alpha.png", added_image);
    }


    void TMapping::imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr & infomsg) {
        
        if(msg->header.stamp.toNSec() > gridMap_.getTimestamp()){
            if (!gridMapContainer_.empty())
            {
                gridMap_ = gridMapContainer_.front();//.pop_front();
                gridMapContainer_.erase(gridMapContainer_.begin());
            }
        }
        //image processing
        cout<<msg->header.stamp<<endl;
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        image_ = cv_ptr->image;       

        sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
        cloud_msg->header = msg_->header;
        cloud_msg->height = msg_->height;
        cloud_msg->width  = msg_->width;
        cloud_msg->is_dense = false;
        cloud_msg->is_bigendian = false;

        sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
        pcd_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::FLOAT32);

        // Update camera model
        image_geometry::PinholeCameraModel camModel;
        camModel.fromCameraInfo(infomsg);
 
        depth_image_proc::convert<uint16_t>(msg_, cloud_msg, camModel);


        sensor_msgs::PointCloud submap_cloud;
        sensor_msgs::PointCloud submapTransformed_cloud;
        sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msg, submap_cloud);
        //To gridMap_ frame        
        try {
            transformListener_.transformPointCloud("/map", msg_->header.stamp, submap_cloud, cameraBase_, submapTransformed_cloud);
        } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return;
        }
        sensor_msgs::convertPointCloudToPointCloud2(submapTransformed_cloud, velodyne_cloud_);


        sensor_msgs::PointCloud2Iterator<float> iter_x(velodyne_cloud_, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(velodyne_cloud_, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(velodyne_cloud_, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(velodyne_cloud_, "intensity");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
        {
            grid_map::Index index;
            grid_map::Position position(*iter_x, *iter_y);
            if (!gridMap_.getIndex(position, index)) continue;
            *iter_intensity = gridMap_.at("traversability", index);
        }

        pcl::PointCloud<pcl::PointXYZI> input;
        pcl::fromROSMsg(velodyne_cloud_, input);
        cv::Mat proj_img(cv::Size(infomsg->width, infomsg->height), CV_32FC1);
        for (size_t i = 0; i < infomsg->height; ++i)
        {
            for (size_t j = 0; j < infomsg->width; ++j){
                proj_img.at<float>(i,j) = input.points[i*infomsg->width+j].intensity;
            }
        }
        cv::Mat binary_img; 
        cv::threshold( proj_img, binary_img, 0.5, 1.0,0);
        std::string file_name = "./imgs/" + std::to_string(msg->header.stamp.toNSec()) + ".png" ;
        imwrite(file_name, binary_img);

//        pcl::PointCloud<pcl::PointXYZI> input;
//        pcl::fromROSMsg(velodyne_cloud_, input);
//        cv::Mat proj_img(cv::Size(infomsg->width, infomsg->height), CV_8UC3,cv::Scalar(255,255,255));
//        for (size_t i = 0; i < infomsg->height; ++i)
//        {
//            for (size_t j = 0; j < infomsg->width; ++j){
//            proj_img.at<cv::Vec3b>(i,j) = cv::Vec3b(255.0*input.points[i*infomsg->width+j].intensity,0,255.0*(1.0-input.points[i*infomsg->width+j].intensity));
//            }
//        }
//        cv::Mat added_image; 
//        cv::addWeighted(image_,0.5,proj_img,0.5,0,added_image);
//        imwrite("alpha.png", added_image);
    }


    void TMapping::depthCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr & infomsg){

        msg_ = msg;
//        sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
//        cloud_msg->header = msg->header;
//        cloud_msg->height = msg->height;
//        cloud_msg->width  = msg->width;
//        cloud_msg->is_dense = false;
//        cloud_msg->is_bigendian = false;

//        sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
//        pcd_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::FLOAT32);

//        // Update camera model
//        image_geometry::PinholeCameraModel camModel;
//        camModel.fromCameraInfo(infomsg);
// 
//        depth_image_proc::convert<uint16_t>(msg, cloud_msg, camModel);


//        sensor_msgs::PointCloud submap_cloud;
//        sensor_msgs::PointCloud submapTransformed_cloud;
//        sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msg, submap_cloud);
//        //To gridMap_ frame        
//        try {
//            transformListener_.transformPointCloud("/map", msg->header.stamp, submap_cloud, cameraBase_, submapTransformed_cloud);
//        } catch (tf::TransformException& ex) {
//        ROS_ERROR("%s", ex.what());
//        return;
//        }
//        sensor_msgs::convertPointCloudToPointCloud2(submapTransformed_cloud, velodyne_cloud_);


//        sensor_msgs::PointCloud2Iterator<float> iter_x(velodyne_cloud_, "x");
//        sensor_msgs::PointCloud2Iterator<float> iter_y(velodyne_cloud_, "y");
//        sensor_msgs::PointCloud2Iterator<float> iter_z(velodyne_cloud_, "z");
//        sensor_msgs::PointCloud2Iterator<float> iter_intensity(velodyne_cloud_, "intensity");

//        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
//        {
//            grid_map::Index index;
//            grid_map::Position position(*iter_x, *iter_y);
//            if (!gridMap_.getIndex(position, index)) continue;
//            *iter_intensity = gridMap_.at("traversability", index);
//        }

////        isCloudIn = true;

//        pcl::PointCloud<pcl::PointXYZI> input;
//        pcl::fromROSMsg(velodyne_cloud_, input);
//        cv::Mat proj_img(cv::Size(infomsg->width, infomsg->height), CV_8UC3,cv::Scalar(255,255,255));
//        for (size_t i = 0; i < infomsg->height; ++i)
//        {
//            for (size_t j = 0; j < infomsg->width; ++j){
//            proj_img.at<cv::Vec3b>(i,j) = cv::Vec3b(255.0*input.points[i*infomsg->width+j].intensity,0,255.0*(1.0-input.points[i*infomsg->width+j].intensity));
//            }
//        }
//        cv::Mat added_image; 
//        cv::addWeighted(image_,0.5,proj_img,0.5,0,added_image);
//        imwrite("alpha.png", added_image);

    }


    void TMapping::gridMapToInitTraversabilityMapCallback(const grid_map_msgs::GridMap& message) {


        // To sensor_msgs::PointCloud2
        grid_map::GridMap map;
        grid_map::GridMapRosConverter::fromMessage(message, map);
//        grid_map::GridMapRosConverter::toPointCloud(gridMap_,"elevation", submap_cloud2);
//        submap_cloud2.fields[9].name = "intensity";
        cout<<message.info.header.stamp<<endl;
        gridMapContainer_.push_back(map);
        if (!isCloudIn){
            gridMap_ = gridMapContainer_.front();//.pop_front();
            gridMapContainer_.erase(gridMapContainer_.begin());
            isCloudIn = true;
        }

    }

    void TMapping::pointCloudCallback(const sensor_msgs::PointCloud2& message) {

//        sensor_msgs::PointCloud submap_cloud;
//        sensor_msgs::PointCloud submapTransformed_cloud;
//        sensor_msgs::convertPointCloud2ToPointCloud(message, submap_cloud);
//        //To gridMap_ frame        
//        try {
//            transformListener_.transformPointCloud("/map", message.header.stamp, submap_cloud, velodyneTopic_, submapTransformed_cloud);
//        } catch (tf::TransformException& ex) {
//        ROS_ERROR("%s", ex.what());
//        return;
//        }
//        sensor_msgs::convertPointCloudToPointCloud2(submapTransformed_cloud, velodyne_cloud_);

////        cout<<velodyne_cloud_.fields[0].name<<endl;
////        cout<<velodyne_cloud_.fields[3].name<<endl;
////        cout<<velodyne_cloud_.fields[4].name<<endl;


//        sensor_msgs::PointCloud2Iterator<float> iter_x(velodyne_cloud_, "x");
//        sensor_msgs::PointCloud2Iterator<float> iter_y(velodyne_cloud_, "y");
//        sensor_msgs::PointCloud2Iterator<float> iter_z(velodyne_cloud_, "z");
//        sensor_msgs::PointCloud2Iterator<float> iter_intensity(velodyne_cloud_, "intensity");

//        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity)
//        {
//            grid_map::Index index;
//            grid_map::Position position(*iter_x, *iter_y);
//            if (!gridMap_.getIndex(position, index)) continue;
//            *iter_intensity = gridMap_.at("traversability", index);
//        }

//        isCloudIn = true;
    }

}  // namespace traversability_test
