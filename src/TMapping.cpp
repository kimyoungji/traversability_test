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
    : nodeHandle_(nodeHandle), octree_(128.0f)
    {

        readParameters();

        // Subscribers
        it_ = new image_transport::ImageTransport(nodeHandle_);
        imageSubscriber_ = it_->subscribeCamera(imageTopic_, 10,&TMapping::imageCallback, this);

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

        // Read parameters for pose subscriber.
        nodeHandle_.param("robot_pose_with_covariance_topic", robotPoseTopic_, string("/pose"));
        nodeHandle_.param("robot_pose_cache_size", robotPoseCacheSize_, 200);

        // Read parameters for grid map subscriber.
        nodeHandle_.param("grid_map_topic_name", gridMapToInitTraversabilityMapTopic_, string("/traversability_estimation/traversability_map"));

        cout<<gridMapToInitTraversabilityMapTopic_<<endl;

    }

    void TMapping::imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr & infomsg) {

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
        sensor_msgs::convertPointCloud2ToPointCloud(submap_cloud2, submap_cloud);
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
//        pcl::toPCLPointCloud2(input, submap_);

        //extract local map & project to cvmat
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(submapTransformed_cloud2, *xyz);
        octree_.setInputCloud (xyz);
        octree_.addPointsFromInputCloud ();

        pcl::PointXYZ searchPoint;
        searchPoint.x = 0.0;
        searchPoint.y = 0.0;
        searchPoint.z = 0.0;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        octree_.radiusSearch (searchPoint, 100.0f, pointIdxRadiusSearch, pointRadiusSquaredDistance);

        cv::Mat proj_img = cv::Mat::zeros(cv::Size(infomsg->width, infomsg->height), CV_8UC3);
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        {
            cv::Point3d pt;
            cv::Point2d uv;
            if(input.points[ pointIdxRadiusSearch[i] ].z>0){
                pt.x = input.points[ pointIdxRadiusSearch[i] ].x;
                pt.y = input.points[ pointIdxRadiusSearch[i] ].y;
                pt.z = input.points[ pointIdxRadiusSearch[i] ].z;
                uv = camModel.project3dToPixel(pt);
//                cout<<uv.x<<endl;
                if(uv.x<infomsg->width && uv.x>0.0 && uv.y<infomsg->height && uv.y>0.0){
                    proj_img.at<cv::Vec3b>(uv.y,uv.x) = cv::Vec3b(0,0,255.0*(1.0-input.points[ pointIdxRadiusSearch[i] ].intensity));
                }
            }
        }
        cv::Mat added_image; 
        cv::addWeighted(image_,0.5,proj_img,0.5,0,added_image);
        imwrite("alpha.png", added_image);
//        cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
//        cv::imshow( "Display window", proj_img );                   // Show our image inside it.
//        cv::waitKey(0);

        //iterations over the inputs

//        // Add normals to submap_       
//        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz (new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::fromROSMsg(submapTransformed_cloud2, *xyz);

//        pcl::PointCloud<pcl::Normal> normals;
//        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//        ne.setInputCloud (xyz);
//        ne.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
////        ne.setKSearch (10);
//        ne.setRadiusSearch (0.1);
//        ne.compute (normals);

//        pcl::PCLPointCloud2 output_normals;
//        pcl::toPCLPointCloud2 (normals, output_normals);
//        pcl::concatenateFields (submap_, output_normals, submap_);

//        // Project to image  

//        // reference frame for the projection
//        // e.g. take XZ plane around 0,0,0 of length 100 and map to 128*128 image
//        Eigen::Vector3f origin = Eigen::Vector3f(0,0,0);
//        Eigen::Vector3f axis_x = Eigen::Vector3f(1,0,0);
//        Eigen::Vector3f axis_y = Eigen::Vector3f(0,0,1);
//        float length    = 100;
//        int image_size  = 128;

//        auto aux_cloud = ProjectToPlane(submap_, origin, axis_x, axis_y);
//        // aux_cloud now contains the points of original_cloud, with:
//        //      xyz coordinates projected to XZ plane
//        //      color (intensity) of the original_cloud (remains unchanged)
//        //      normals - we lose the normal information, as we use this field to save the projection information. if you wish to keep the normal data, you should define a custom PointType. 
//        // note: for the sake of projection, the origin is only used to define the plane, so any arbitrary point on the plane can be used


//        auto grid = GenerateGrid(origin, axis_x , axis_y, length, image_size);
//        // organized cloud that can be trivially mapped to an image

//        float max_resolution = 2 * length / image_size;
//        int max_nn_to_consider = 16;
//        InterpolateToGrid(aux_cloud, grid, max_resolution, max_nn_to_consider);

//        image_mask is std::pair<cv::Mat, cv::Mat>
//        auto image_mask = ConvertGridToImage(grid, false);
    }

    void TMapping::gridMapToInitTraversabilityMapCallback(const grid_map_msgs::GridMap& message) {


        // To sensor_msgs::PointCloud2
        grid_map::GridMap gridMap;
        grid_map::GridMapRosConverter::fromMessage(message, gridMap);
        grid_map::GridMapRosConverter::toPointCloud(gridMap,"elevation", submap_cloud2);
        submap_cloud2.fields[9].name = "intensity";
//        for  (size_t i = 0; i < 11; ++i){
//        cout<<submap_cloud2.fields[i].name<<endl;
//        }
//        std::vector<std::string> layers = gridMap.getLayers();
//        for (const auto& layer : layers) {
//        cout<<layer<<endl;
//        }


    }

}  // namespace traversability_test
