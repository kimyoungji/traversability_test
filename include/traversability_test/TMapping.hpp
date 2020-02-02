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
#include <pcl/common/io.h>
#include <pcl/search/kdtree.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_impl.h> 

// cv
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h> 
#include <opencv2/calib3d.hpp>

// ROS
//#include <filters/filter_chain.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>


#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
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
    std::string cameraTopic_;
    std::string pointCloudTopic_;
    std::string velodyneTopic_;
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
    ros::Subscriber velodyneSubscriber_;

    // Callbacks.
    void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr & infomsg);
    void gridMapToInitTraversabilityMapCallback(const grid_map_msgs::GridMap& message);
    void pointCloudCallback(const sensor_msgs::PointCloud2& message);

    //! TF listener.
    tf::TransformListener transformListener_;

    //input camera info
    cv::Mat image_;
    Eigen::Matrix<double,3,4> P0_;

    //submap msgs
    sensor_msgs::PointCloud2 submap_cloud2;
    pcl::PCLPointCloud2 submap_;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_;
//    pcl::PCLPointCloud2 submapTransformed_;
    sensor_msgs::PointCloud2 velodyne_cloud_;
//    pcl::PCLPointCloud2 tiltedLidar_;

    //map
    grid_map::GridMap gridMap_;

    //boolean
    bool isCloudIn;

//    pcl::PointCloud<pcl::PointXYZINormal>::Ptr ProjectToPlane(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, Eigen::Vector3f origin, Eigen::Vector3f axis_x, Eigen::Vector3f axis_y)
//    {
//        pcl::PointCloud<pcl::PointXYZINormal>::Ptr aux_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
//        copyPointCloud(*cloud, *aux_cloud);

//        auto normal = axis_x.cross(axis_y);
//        Eigen::Hyperplane<float, 3> plane(normal, origin);

//        for (auto itPoint = aux_cloud->begin(); itPoint != aux_cloud->end(); itPoint++)
//        {
//            // project point to plane
//            auto proj = plane.projection(itPoint->getVector3fMap());
//            itPoint->getVector3fMap() = proj;
//        }
//        return aux_cloud;
//    }

//    pcl::PointCloud<pcl::PointXYZINormal>::Ptr GenerateGrid(Eigen::Vector3f origin, Eigen::Vector3f axis_x , Eigen::Vector3f axis_y, float length, int image_size)
//    {
//        auto step = length / image_size;

//        pcl::PointCloud<pcl::PointXYZINormal>::Ptr image_cloud(new pcl::PointCloud<pcl::PointXYZINormal>(image_size, image_size));
//        for (auto i = 0; i < image_size; i++)
//            for (auto j = 0; j < image_size; j++)
//            {
//                int x = i - int(image_size / 2);
//                int y = j - int(image_size / 2);
//                image_cloud->at(i, j).getVector3fMap() = origin + (x * step * axis_x) + (y * step * axis_y);
//            }

//        return image_cloud;
//    }

//    void InterpolateToGrid(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, pcl::PointCloud<pcl::PointXYZINormal>::Ptr grid, float max_resolution, int max_nn_to_consider)
//    {   
//        pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>);
//        tree->setInputCloud(cloud);

//        for (auto idx = 0; idx < grid->points.size(); idx++)
//        {
//            std::vector<int> indices;
//            std::vector<float> distances;
//            if (tree->radiusSearch(grid->points[idx], max_resolution, indices, distances, max_nn_to_consider) > 0)
//            {
//                // Linear Interpolation of:
//                //      Intensity
//                //      Normals- residual vector to inflate(recondtruct) the surface
//                float intensity(0);
//                Eigen::Vector3f n(0, 0, 0);
//                float weight_factor = 1.0F / accumulate(distances.begin(), distances.end(), 0.0F);
//                for (auto i = 0; i < indices.size(); i++)
//                {
//                    float w = weight_factor * distances[i];
//                    intensity += w * cloud->points[indices[i]].intensity;
//                    auto res = cloud->points[indices[i]].getVector3fMap() - grid->points[idx].getVector3fMap();
//                    n += w * res;
//                }
//                grid->points[idx].intensity = intensity;
//                grid->points[idx].getNormalVector3fMap() = n;
//                grid->points[idx].curvature = 1;
//            }
//            else
//            {
//                grid->points[idx].intensity = 0;
//                grid->points[idx].curvature = 0;
//                grid->points[idx].getNormalVector3fMap() = Eigen::Vector3f(0, 0, 0);
//            }
//        }
//    }

//    // Convert an Organized cloud to cv::Mat (an image and a mask)
//    //      point Intensity is used for the image
//    //          if as_float is true => take the raw intensity (image is CV_32F)
//    //          if as_float is false => assume intensity is in range [0, 255] and round it (image is CV_8U)
//    //      point Curvature is used for the mask (assume 1 or 0)
//    std::pair<cv::Mat, cv::Mat> ConvertGridToImage(pcl::PointCloud<pcl::PointXYZINormal>::Ptr grid, bool as_float)
//    {   
//        int rows = grid->height;
//        int cols = grid->width;

//        if ((rows <= 0) || (cols <= 0)) 
//            return std::pair<cv::Mat, cv::Mat>(cv::Mat(), cv::Mat());

//        // Initialize

//        cv::Mat image = cv::Mat(rows, cols, as_float? CV_32F : CV_8U);
//        cv::Mat mask  = cv::Mat(rows, cols, CV_8U);

//        if (as_float)
//        {
//            for (int y = 0; y < image.rows; y++)
//            {
//                for (int x = 0; x < image.cols; x++)
//                {
//                    image.at<float>(y, x) = grid->at(x, image.rows - y - 1).intensity;
//                    mask.at<uchar>(y, x) = 255 * grid->at(x, image.rows - y - 1).curvature;
//                }
//            }
//        }
//        else
//        {
//            for (int y = 0; y < image.rows; y++)
//            {
//                for (int x = 0; x < image.cols; x++)
//                {
//                    image.at<uchar>(y, x) = (int)round(grid->at(x, image.rows - y - 1).intensity);
//                    mask.at<uchar>(y, x) = 255 * grid->at(x, image.rows - y - 1).curvature;
//                }
//            }
//        }

//        return std::pair<cv::Mat, cv::Mat>(image, mask);
//    }

//    // project image to cloud (using the grid data)
//    // organized - whether the resulting cloud should be an organized cloud
//    pcl::PointCloud<pcl::PointXYZINormal>::Ptr BackProjectImage(cv::Mat image, cv::Mat mask, pcl::PointCloud<pcl::PointXYZINormal>::Ptr grid, bool organized)
//    {
//        if ((image.size().height != grid->height) || (image.size().width != grid->width))
//        {
//            assert(false);
//            throw;
//        }

//        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
//        cloud->reserve(grid->height * grid->width);

//        // order of iteration is critical for organized target cloud
//        for (auto r = image.size().height - 1; r >= 0; r--)
//        {
//            for (auto c = 0; c < image.size().width; c++)
//            {
//                pcl::PointXYZINormal point;
//                auto mask_value = mask.at<uchar>(image.rows - r - 1, c);
//                if (mask_value > 0) // valid pixel
//                {
//                    point.intensity = mask_value;
//                    point.getVector3fMap() = grid->at(c, r).getVector3fMap() + grid->at(c, r).getNormalVector3fMap();
//                }
//                else // invalid pixel
//                {
//                    if (organized)
//                    {
//                        point.intensity = 0;
//                        point.x = std::numeric_limits<float>::quiet_NaN();
//                        point.y = std::numeric_limits<float>::quiet_NaN();
//                        point.z = std::numeric_limits<float>::quiet_NaN();
//                    }
//                    else
//                    {
//                        continue;
//                    }
//                }

//                cloud->push_back(point);
//            }
//        }

//        if (organized)
//        {
//            cloud->width = grid->width;
//            cloud->height = grid->height;
//        }

//        return cloud;
//    }

};//class

}  // namespace traversability_test
