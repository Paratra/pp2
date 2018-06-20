/* *****************************************************************
*
* polls3d_ming_node
*
******************************************************************/

/**
* @file   %FILENAME%
* @author %USER% (%$EMAIL%)
* @date   %DATE%
*
* @brief  Filedescription
*/

#ifndef POLLS3D_MING_NODE_H
#define POLLS3D_MING_NODE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "velodyne_msgs/VelodyneScan.h"
#include "velodyne_msgs/VelodynePacket.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include <typeinfo>
#include <ctime>
#include <sstream>
#include <math.h>



#include <sensor_msgs/point_cloud_conversion.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <vector>
#include <Eigen/Dense>
#include <polls3d/point_types.h>


typedef velodyne_pointcloud::PointXYZDR DPoint;
typedef pcl::PointCloud<DPoint> DPointCloud;

class polls3d_ming_node
{
public:
  polls3d_ming_node(ros::NodeHandle &node_handle);

private:
  // node handle
  ros::NodeHandle *node_;

  // ros communication
  ros::Subscriber polls3d_ming_sub_;

  ros::Publisher polls3d_ming_pub_;

  ros::Publisher test_pub_;

  ros::ServiceServer polls3d_ming_srv_;

  ros::ServiceClient polls3d_ming_cli_;

  ros::Timer my_timer_;

  // parameters
  //double my_double_parameter_;
  //std::string my_string_parameter_;
  //int my_int_parameter_;
  //bool my_bool_parameter_;

  // callbacks
  //void subscriberCallback(const sensor_msgs::Joy &msg);
  void polls3d_mingCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void pole_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud);

  // void heightThresholder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cutCloud,
  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr processCloud, double zMin, double zMax);
    //bool serviceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    //void timerCallback(const ros::TimerEvent &evt);
  };

  #endif // polls3d_ming_node_H
