#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/PointCloud2.h>
//#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <polls3d/Pole.h>
#include <polls3d/Poles.h>
#include <std_msgs/Float64.h>
#include <imu_3dm_gx4/GPSData.h>
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

//typedef velodyne_pointcloud::PointXYZIR VPoint;
typedef velodyne_pointcloud::PointXYZDR DPoint;
typedef pcl::PointXYZ Point;
//typedef pcl::PointCloud<VPoint> VPointCloud;
typedef pcl::PointCloud<DPoint> DPointCloud;
typedef pcl::PointCloud<Point> PointCloud;

#ifndef MING_POLLS_H_
#define MING_POLLS_H_

struct PointPTR {
	double phi;
	double theta;
	double r;
};

class Ming_Polls {
	ros::Subscriber pcl_sub_;
	//ros::Subscriber odom_sub_;

	ros::Publisher pcl_pub_rising_;
	ros::Publisher pcl_pub_falling_;
	ros::Publisher pcl_pub_on_obj_;
	ros::Publisher pcl_pub_tmp_on_obj_;
	//ros::Publisher odom_pub_;
	ros::Publisher poles_pub_;
	ros::Publisher obj_filtered_pub_;


	int on_obj_counter_rising_;

	std::vector<int> ring_counters_;
	std::vector<double> last_distances_;
	std::vector<DPoint> last_points_;
	std::vector<bool> rising_edge_;
	std::vector<bool> falling_edge_;
	std::vector<DPointCloud> tmp_object_;
	polls3d::Pole pole_;
	polls3d::Poles poles_;
	DPointCloud::Ptr objects_;



public:
	Ming_Polls(ros::NodeHandle);
	virtual ~Ming_Polls();
	void velodynePCLCallback(sensor_msgs::PointCloud2ConstPtr msg);
	//void GPSdataCallback(imu_3dm_gx4::GPSData odom_msg);
	void risingEdge(DPoint);
	void fallingEdge(DPoint point);



};

#endif /* MING_POLLS_H_ */
