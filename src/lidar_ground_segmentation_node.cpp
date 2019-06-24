#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <sstream>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include "lidar_ground_segmentation/plane_equation.h"

class groundSegmenter {
private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher ground_plane_pub;
    ros::Publisher surface_eqn_pub;

public:
    groundSegmenter() {
        cloud_sub = nh.subscribe("/kitti/velo/pointcloud_withring", 1,
                &groundSegmenter::callback, this);
        ground_plane_pub = nh.advertise<sensor_msgs::PointCloud2>
                ("/kitti/velo/pointcloud_groundplane", 1);
        surface_eqn_pub = nh.advertise<lidar_ground_segmentation::plane_equation>
                ("/kitti/velo/pointcloud_surface_eqn", 1);
    }

    void publishPlane(pcl::PointCloud<pcl::PointXYZ> cloud,
                      pcl::ModelCoefficients::Ptr coefficients,
                      std_msgs::Header header) {
        sensor_msgs::PointCloud2 cloud_ros;
        pcl::toROSMsg(cloud, cloud_ros);
        cloud_ros.header.frame_id = header.frame_id;
        cloud_ros.header.stamp = header.stamp;

        lidar_ground_segmentation::plane_equation pln_eqn;
        pln_eqn.header.frame_id = header.frame_id;
        pln_eqn.header.stamp = header.stamp;
        pln_eqn.a = coefficients->values[0];
        pln_eqn.b = coefficients->values[1];
        pln_eqn.c = coefficients->values[2];
        pln_eqn.d = coefficients->values[3];

        ground_plane_pub.publish(cloud_ros);
        surface_eqn_pub.publish(pln_eqn);
    }

    void segmentCloud(pcl::PointCloud<pcl::PointXYZ> cloud_in,
                      std_msgs::Header header) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        *cloud = cloud_in;

        pcl::ModelCoefficients::Ptr coefficients
                                (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        /// Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        /// Optional
        seg.setOptimizeCoefficients (true);
        /// Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.05);
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

        std::vector<int> inlier_indices(inliers->indices);

        if(inlier_indices.size() == 0) {
            ROS_ERROR("No Plane Detected!");
            ros::shutdown();
        }

//        ROS_INFO_STREAM("Normal Vector: [ "
//                  << coefficients->values[0] << " "
//                  << coefficients->values[1] << " "
//                  << coefficients->values[2] << " ]");

        pcl::PointCloud<pcl::PointXYZ>::Ptr
                        plane_unfiltered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, inlier_indices, *plane_unfiltered);

        pcl::PointCloud<pcl::PointXYZ>::Ptr
                plane_sor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(plane_unfiltered);
        sor.setMeanK(50);
        sor.setStddevMulThresh (1.0);
        sor.filter(*plane_sor_filtered);

        publishPlane(*plane_sor_filtered, coefficients, header);
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud_in_pcl;
        pcl::fromROSMsg(*cloud_msg, cloud_in_pcl);
        segmentCloud(cloud_in_pcl, cloud_msg->header);
    }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_ground_segmentation");
  groundSegmenter gS;
  ros::spin();
  return 0;
}
