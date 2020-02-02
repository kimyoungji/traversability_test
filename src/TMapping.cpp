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
        depthSubscriber_ = it_->subscribeCamera(depthTopic_, 1, &TMapping::depthCallback, this);
        imageSubscriber_ = it_->subscribeCamera(imageTopic_, 1, &TMapping::imageCallback, this);

//        message_filters::Subscriber<sensor_msgs::Image> image_sub(nodeHandle_, imageTopic_, 1);
//        message_filters::Subscriber<sensor_msgs::Image> depth_sub(nodeHandle_, depthTopic_, 1);
//        message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nodeHandle_, cameraTopic_, 1);
//        message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> sync(image_sub, depth_sub, info_sub, 10);
//        sync.registerCallback(boost::bind(&TMapping::imagesCallback, this, _1, _2, _3));

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
        nodeHandle_.param("depth_topic", depthTopic_, string("/camera/aligned_depth_to_color/image_raw"));
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

    void TMapping::imagesCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2, const sensor_msgs::CameraInfoConstPtr & infomsg){


        // Depth manipulation//
        sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
        cloud_msg->header = msg2->header;
        cloud_msg->height = msg2->height;
        cloud_msg->width  = msg2->width;
        cloud_msg->is_dense = false;
        cloud_msg->is_bigendian = false;

        sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
        pcd_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::FLOAT32);

        // Update camera model
        image_geometry::PinholeCameraModel camModel;
        camModel.fromCameraInfo(infomsg);

        depth_image_proc::convert<uint16_t>(msg2, cloud_msg, camModel);

        //To gridMap_ frame   
        sensor_msgs::PointCloud submap_cloud;
        sensor_msgs::PointCloud submapTransformed_cloud;
        sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msg, submap_cloud);     
        try {
            transformListener_.transformPointCloud("/map", msg2->header.stamp, submap_cloud, cameraTopic_, submapTransformed_cloud);
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

        // Image processing //
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::RGB8);
        image_ = cv_ptr->image;       


        /***************************************/
        /*****transform map to camera frame*****/
        /***************************************/
        sensor_msgs::PointCloud2 submapTransformed_cloud2;
//        sensor_msgs::PointCloud submap_cloud;
//        sensor_msgs::PointCloud submapTransformed_cloud;
        sensor_msgs::convertPointCloud2ToPointCloud(velodyne_cloud_, submap_cloud);
        try {
//            transformListener_.transformPointCloud(cameraTopic_, submap_cloud, submapTransformed_cloud);
            transformListener_.transformPointCloud(cameraTopic_, msg1->header.stamp, submap_cloud, "/map", submapTransformed_cloud);
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
                    proj_img.at<cv::Vec3b>(uv.y,uv.x) = cv::Vec3b(0,255.0*input.points[i].intensity,255.0*(1.0-input.points[i].intensity));
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
        
        //image processing
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        image_ = cv_ptr->image;       

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
//        sensor_msgs::convertPointCloud2ToPointCloud(velodyne_cloud_, submap_cloud);
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

//        cv::Mat proj_img(cv::Size(infomsg->width, infomsg->height), CV_8UC3,cv::Scalar(255,255,255));
//        for (size_t i = 0; i < input.points.size(); ++i)
//        {
//            cv::Point3d pt;
//            cv::Point2d uv;
//            if(input.points[i].z>0){
//                pt.x = input.points[i].x;
//                pt.y = input.points[i].y;
//                pt.z = input.points[i].z;
//                uv = camModel.project3dToPixel(pt);
////                cout<<uv.x<<endl;
//                if(uv.x<infomsg->width && uv.x>0.0 && uv.y<infomsg->height && uv.y>0.0){
//                    proj_img.at<cv::Vec3b>(uv.y,uv.x) = cv::Vec3b(255.0*input.points[i].intensity,0,255.0*(1.0-input.points[i].intensity));
//                }
//            }
//        }
//        cv::Mat added_image; 
//        cv::addWeighted(image_,0.5,proj_img,0.5,0,added_image);
//        imwrite("alpha.png", added_image);
////        cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
////        cv::imshow( "Display window", proj_img );                   // Show our image inside it.
////        cv::waitKey(0);


    }


    void TMapping::depthCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr & infomsg){

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
            transformListener_.transformPointCloud("/map", msg->header.stamp, submap_cloud, cameraTopic_, submapTransformed_cloud);
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

//        isCloudIn = true;

        pcl::PointCloud<pcl::PointXYZI> input;
        pcl::fromROSMsg(velodyne_cloud_, input);
        cv::Mat proj_img(cv::Size(infomsg->width, infomsg->height), CV_8UC3,cv::Scalar(255,255,255));
        for (size_t i = 0; i < infomsg->height; ++i)
        {
            for (size_t j = 0; j < infomsg->width; ++j){
            proj_img.at<cv::Vec3b>(i,j) = cv::Vec3b(255.0*input.points[i*infomsg->width+j].intensity,0,255.0*(1.0-input.points[i*infomsg->width+j].intensity));
            }
        }
        cv::Mat added_image; 
        cv::addWeighted(image_,0.5,proj_img,0.5,0,added_image);
        imwrite("alpha.png", added_image);


//        /***************************************/
//        /*****transform map to camera frame*****/
//        /***************************************/
//        sensor_msgs::PointCloud2 submapTransformed_cloud2;
////        sensor_msgs::PointCloud submap_cloud;
////        sensor_msgs::PointCloud submapTransformed_cloud;
//        sensor_msgs::convertPointCloud2ToPointCloud(velodyne_cloud_, submap_cloud);
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

//        cv::Mat proj_img(cv::Size(infomsg->width, infomsg->height), CV_8UC3,cv::Scalar(255,255,255));
//        for (size_t i = 0; i < input.points.size(); ++i)
//        {
//            cv::Point3d pt;
//            cv::Point2d uv;
//            if(input.points[i].z>0){
//                pt.x = input.points[i].x;
//                pt.y = input.points[i].y;
//                pt.z = input.points[i].z;
//                uv = camModel.project3dToPixel(pt);
////                cout<<uv.x<<endl;
//                if(uv.x<infomsg->width && uv.x>0.0 && uv.y<infomsg->height && uv.y>0.0){
//                    proj_img.at<cv::Vec3b>(uv.y,uv.x) = cv::Vec3b(255.0*input.points[i].intensity,0,255.0*(1.0-input.points[i].intensity));
//                }
//            }
//        }
//        cv::Mat added_image; 
//        cv::addWeighted(image_,0.5,proj_img,0.5,0,added_image);
//        imwrite("alpha.png", added_image);
////        cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
////        cv::imshow( "Display window", proj_img );                   // Show our image inside it.
////        cv::waitKey(0);

    }


    void TMapping::gridMapToInitTraversabilityMapCallback(const grid_map_msgs::GridMap& message) {


        // To sensor_msgs::PointCloud2
        grid_map::GridMapRosConverter::fromMessage(message, gridMap_);
        grid_map::GridMapRosConverter::toPointCloud(gridMap_,"elevation", submap_cloud2);
        submap_cloud2.fields[9].name = "intensity";

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
