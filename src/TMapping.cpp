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
        it_ = new image_transport::ImageTransport(nodeHandle_);
        imageSubscriber_ = it_->subscribeCamera(imageTopic_, 1,&TMapping::imageCallback, this);

        velodyneSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &TMapping::pointCloudCallback, this);


        robotPoseSubscriber_.subscribe(nodeHandle_, robotPoseTopic_, 1);
        robotPoseCache_.connectInput(robotPoseSubscriber_);
        robotPoseCache_.setCacheSize(robotPoseCacheSize_);

        gridMapToInitTraversabilityMapSubscriber_ = nodeHandle_.subscribe(
                gridMapToInitTraversabilityMapTopic_, 1, &TMapping::gridMapToInitTraversabilityMapCallback, this);

//        // TF subscribe
//        try {
//            transformListener_.transformPoint(traversabilityMap_.getMapFrameId(), submapPoint_, submapPointTransformed);
//        } catch (tf::TransformException& ex) {
//        ROS_ERROR("%s", ex.what());
//        return false;
//        }
    }

    TMapping::~TMapping()
    {
    }


    void TMapping::readParameters()
    {
        // Read parameters for image subscriber.
        nodeHandle_.param("image_topic", imageTopic_, string("/camera/color/image_raw"));
        nodeHandle_.param("camera_topic", cameraTopic_, string("camera_color_optical_frame"));
        nodeHandle_.param("pointcloud_topic", pointCloudTopic_, string("/velodyne_points"));
        nodeHandle_.param("velodyne_topic", velodyneTopic_, string("velodyne_actual"));

        // Read parameters for pose subscriber.
        nodeHandle_.param("robot_pose_with_covariance_topic", robotPoseTopic_, string("/pose"));
        nodeHandle_.param("robot_pose_cache_size", robotPoseCacheSize_, 200);

        // Read parameters for grid map subscriber.
        nodeHandle_.param("grid_map_topic_name", gridMapToInitTraversabilityMapTopic_, string("/traversability_estimation/traversability_map"));

        cout<<gridMapToInitTraversabilityMapTopic_<<endl;

    }

//    void TMapping::imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr & infomsg) {

//        //image processing
//        cv_bridge::CvImagePtr cv_ptr;
//        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
//        image_ = cv_ptr->image;       

//        //camera info
//        P0_ = Eigen::Map<const Eigen::MatrixXd>(&infomsg->P[0], 3, 4);
//        image_geometry::PinholeCameraModel camModel;
//        camModel.fromCameraInfo(infomsg);


//        /***************************************/
//        /*****transform map to camera frame*****/
//        /***************************************/
//        sensor_msgs::PointCloud2 submapTransformed_cloud2;
//        sensor_msgs::PointCloud submap_cloud;
//        sensor_msgs::PointCloud submapTransformed_cloud;
//        sensor_msgs::convertPointCloud2ToPointCloud(submap_cloud2, submap_cloud);
//        try {
////            transformListener_.transformPointCloud(cameraTopic_, submap_cloud, submapTransformed_cloud);
//            transformListener_.transformPointCloud(cameraTopic_, msg->header.stamp, submap_cloud, "/map", submapTransformed_cloud);
//        } catch (tf::TransformException& ex) {
//        ROS_ERROR("%s", ex.what());
//        return;
//        }
//        sensor_msgs::convertPointCloudToPointCloud2(submapTransformed_cloud, submapTransformed_cloud2);

//        // To PCLPointCloud2
//        pcl::PointCloud<pcl::PointXYZI> input;
//        pcl::fromROSMsg(submapTransformed_cloud2, input);
////        pcl::toPCLPointCloud2(input, submap_);

//        //extract local map & project to cvmat
//        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz (new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::fromROSMsg(submapTransformed_cloud2, *xyz);
//        octree_.setInputCloud (xyz);
//        octree_.addPointsFromInputCloud ();

//        pcl::PointXYZ searchPoint;
//        searchPoint.x = 0.0;
//        searchPoint.y = 0.0;
//        searchPoint.z = 0.0;
//        std::vector<int> pointIdxRadiusSearch;
//        std::vector<float> pointRadiusSquaredDistance;
//        octree_.radiusSearch (searchPoint, 100.0f, pointIdxRadiusSearch, pointRadiusSquaredDistance);

//        cv::Mat proj_img(cv::Size(infomsg->width, infomsg->height), CV_8UC3,cv::Scalar(255,255,255));
//        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
//        {
//            cv::Point3d pt;
//            cv::Point2d uv;
//            if(input.points[ pointIdxRadiusSearch[i] ].z>0){
//                pt.x = input.points[ pointIdxRadiusSearch[i] ].x;
//                pt.y = input.points[ pointIdxRadiusSearch[i] ].y;
//                pt.z = input.points[ pointIdxRadiusSearch[i] ].z;
//                uv = camModel.project3dToPixel(pt);
////                cout<<uv.x<<endl;
//                if(uv.x<infomsg->width && uv.x>0.0 && uv.y<infomsg->height && uv.y>0.0){
//                    proj_img.at<cv::Vec3b>(uv.y,uv.x) = cv::Vec3b(255.0*input.points[ pointIdxRadiusSearch[i] ].intensity,0,255.0*(1.0-input.points[ pointIdxRadiusSearch[i] ].intensity));
//                }
//            }
//        }
//        cv::Mat added_image; 
//        cv::addWeighted(image_,0.5,proj_img,0.5,0,added_image);
//        imwrite("alpha.png", added_image);
////        cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
////        cv::imshow( "Display window", proj_img );                   // Show our image inside it.
////        cv::waitKey(0);

//    }

    void TMapping::imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr & infomsg) {
        
        if(isCloudIn)
        {
        //image processing
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        image_ = cv_ptr->image;       

        //camera info
        P0_ = Eigen::Map<const Eigen::MatrixXd>(&infomsg->P[0], 3, 4);
        image_geometry::PinholeCameraModel camModel;
        camModel.fromCameraInfo(infomsg);


        /***************************************/
        /*****transform map to camera frame*****/
        /***************************************/
        sensor_msgs::PointCloud2 submapTransformed_cloud2;
        sensor_msgs::PointCloud submap_cloud;
        sensor_msgs::PointCloud submapTransformed_cloud;
        sensor_msgs::convertPointCloud2ToPointCloud(velodyne_cloud_, submap_cloud);
        try {
//            transformListener_.transformPointCloud(cameraTopic_, submap_cloud, submapTransformed_cloud);
            transformListener_.transformPointCloud(cameraTopic_, msg->header.stamp, submap_cloud, "/map", submapTransformed_cloud);
        } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return;
        }
        sensor_msgs::convertPointCloudToPointCloud2(submapTransformed_cloud, submapTransformed_cloud2);

        // To PCLPointCloud2
        pcl::PointCloud<pcl::PointXYZI> input;
        pcl::fromROSMsg(submapTransformed_cloud2, input);

        cv::Mat proj_img(cv::Size(infomsg->width, infomsg->height), CV_8UC3,cv::Scalar(255,255,255));
        for (size_t i = 0; i < input.points.size(); ++i)
        {
            cv::Point3d pt;
            cv::Point2d uv;
            if(input.points[i].z>0){
                pt.x = input.points[i].x;
                pt.y = input.points[i].y;
                pt.z = input.points[i].z;
                uv = camModel.project3dToPixel(pt);
//                cout<<uv.x<<endl;
                if(uv.x<infomsg->width && uv.x>0.0 && uv.y<infomsg->height && uv.y>0.0){
                    proj_img.at<cv::Vec3b>(uv.y,uv.x) = cv::Vec3b(255.0*input.points[i].intensity,0,255.0*(1.0-input.points[i].intensity));
                }
            }
        }
        cv::Mat added_image; 
        cv::addWeighted(image_,0.5,proj_img,0.5,0,added_image);
        imwrite("alpha.png", added_image);
//        cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
//        cv::imshow( "Display window", proj_img );                   // Show our image inside it.
//        cv::waitKey(0);

        isCloudIn = false;
        }

    }

    void TMapping::gridMapToInitTraversabilityMapCallback(const grid_map_msgs::GridMap& message) {


        // To sensor_msgs::PointCloud2
//        grid_map::GridMap gridMap;
        grid_map::GridMapRosConverter::fromMessage(message, gridMap_);
        grid_map::GridMapRosConverter::toPointCloud(gridMap_,"elevation", submap_cloud2);
        submap_cloud2.fields[9].name = "intensity";
//        for  (size_t i = 0; i < 11; ++i){
//        cout<<submap_cloud2.fields[i].name<<endl;
//        }
//        std::vector<std::string> layers = gridMap.getLayers();
//        for (const auto& layer : layers) {
//        cout<<layer<<endl;
//        }


    }

    void TMapping::pointCloudCallback(const sensor_msgs::PointCloud2& message) {

//        velodyne_cloud_.clear();
        sensor_msgs::PointCloud submap_cloud;
        sensor_msgs::PointCloud submapTransformed_cloud;
        sensor_msgs::convertPointCloud2ToPointCloud(message, submap_cloud);
        //To gridMap_ frame        
        try {
            transformListener_.transformPointCloud("/map", message.header.stamp, submap_cloud, velodyneTopic_, submapTransformed_cloud);
        } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return;
        }
        sensor_msgs::convertPointCloudToPointCloud2(submapTransformed_cloud, velodyne_cloud_);

//        cout<<velodyne_cloud_.fields[0].name<<endl;
//        cout<<velodyne_cloud_.fields[3].name<<endl;
//        cout<<velodyne_cloud_.fields[4].name<<endl;


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

        isCloudIn = true;
    }

}  // namespace traversability_test
