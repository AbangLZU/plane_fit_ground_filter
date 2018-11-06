#pragma once

#include <ros/ros.h>

// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

// using eigen lib
#include <Eigen/Dense>

#include <velodyne_pointcloud/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#define CLIP_HEIGHT 7.0 //截取掉高于雷达自身0.2米的点
#define MIN_DISTANCE 2.5
#define SENSOR_HEIGHT 1.78

//Customed Point Struct for holding clustered points
namespace plane_ground_filter
{
/** Euclidean Velodyne coordinate, including intensity and ring number, and label. */
struct PointXYZIRL
{
  PCL_ADD_POINT4D;                // quad-word XYZ
  float intensity;                ///< laser intensity reading
  uint16_t ring;                  ///< laser ring number
  uint16_t label;                 ///< point label
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

}; // namespace plane_ground_filter

#define SLRPointXYZIRL plane_ground_filter::PointXYZIRL
#define VPoint velodyne_pointcloud::PointXYZIR
#define RUN pcl::PointCloud<SLRPointXYZIRL>

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(plane_ground_filter::PointXYZIRL,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(uint16_t, label, label))

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;

/*
    @brief Compare function to sort points. Here use z axis.
    @return z-axis accent
*/
bool point_cmp(VPoint a, VPoint b)
{
  return a.z < b.z;
}

class PlaneGroundFilter
{

private:
  pcl::PointCloud<VPoint>::Ptr g_seeds_pc(new pcl::PointCloud<VPoint>());
  pcl::PointCloud<VPoint>::Ptr g_ground_pc(new pcl::PointCloud<VPoint>());
  pcl::PointCloud<VPoint>::Ptr g_not_ground_pc(new pcl::PointCloud<VPoint>());
  pcl::PointCloud<SLRPointXYZIRL>::Ptr g_all_pc(new pcl::PointCloud<SLRPointXYZIRL>());

  ros::Subscriber sub_point_cloud_;
  ros::Publisher pub_ground_, pub_no_ground_;

  ros::NodeHandle node_handle_;
  ros::Subscriber points_node_sub_;
  ros::Publisher ground_points_pub_;
  ros::Publisher groundless_points_pub_;
  ros::Publisher all_points_pub_;

  std::string point_topic_;

  int sensor_model_;
  double sensor_height_;
  int num_seg_;
  int num_iter_;
  int num_lpr_;
  double th_seeds_;
  double th_dist_;
  // Model parameter for ground plane fitting
  // The ground plane model is: ax+by+cz+d=0
  // Here normal:=[a,b,c], d=d
  // th_dist_d_ = threshold_dist - d
  float d_;
  MatrixXf normal_;
  float th_dist_d_;

  void estimate_plane_(void);
  void extract_initial_seeds_(const pcl::PointCloud<VPoint> &p_sorted);
  void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);
  void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr out);
  void remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

public:
  PlaneGroundFilter(ros::NodeHandle &nh);
  ~PlaneGroundFilter();
  void Spin();
};