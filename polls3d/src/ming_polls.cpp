#include "polls3d/ming_polls.h"

using namespace std;
using namespace cv;


Ming_Polls::Ming_Polls(ros::NodeHandle nh) {
	pcl_sub_ = nh.subscribe("/velodyne_points", 10, &Ming_Polls::velodynePCLCallback, this);

	pcl_pub_on_obj_ = nh.advertise<sensor_msgs::PointCloud2>("/test_out", 10, this);
	poles_pub_ =  nh.advertise<polls3d::Poles>("/poles", 10, this);
	obj_filtered_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/obj_filtered", 10, this);


	// test_pub_ = nh.advertise<polls3d::poles>("/poles", 10, this);

	on_obj_counter_rising_ = 0;

	polls3d::Pole pole_;
	polls3d::Poles poles_;

	last_distances_ = std::vector<double>(64,0.0);
	last_points_ = std::vector<DPoint>(64);
	rising_edge_ = std::vector<bool>(64, false);
	falling_edge_ = std::vector<bool>(64, false);
	tmp_object_ = std::vector<DPointCloud>(64);
	objects_ = DPointCloud::Ptr(new DPointCloud);

}

Ming_Polls::~Ming_Polls() {
	// TODO Auto-generated destructor stub
}

void Ming_Polls::velodynePCLCallback(sensor_msgs::PointCloud2ConstPtr msg){

	double depth_jump_dist = .2;

	on_obj_counter_rising_ = 0;

	polls3d::Pole pole_;
	polls3d::Poles poles_;
	last_distances_ = std::vector<double>(64,0.0);
	last_points_ = std::vector<DPoint>(64);
	rising_edge_ = std::vector<bool>(64, false);
	tmp_object_ = std::vector<DPointCloud>(64);
	objects_ = DPointCloud::Ptr(new DPointCloud);


	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_kd(new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr obj_filtered(new pcl::PointCloud<pcl::PointXYZ> ());


	pcl::PCLPointCloud2 pcl_pc2;

	pcl_conversions::toPCL(*msg,pcl_pc2);
	DPointCloud::Ptr cloud_ptr(new DPointCloud);
	pcl::fromPCLPointCloud2(pcl_pc2,*cloud_ptr);

	std::vector<int> ring_counters(64, 0);
	std::vector<double> tmp_der_sum(64, 0.0);

	DPointCloud::Ptr edge_points_rising(new DPointCloud);
	DPointCloud::Ptr edge_points_falling(new DPointCloud);


	for(DPointCloud::iterator it = cloud_ptr->begin(); it != cloud_ptr->end(); it++){

		if(it->distance > 2. &&
			it->distance < 30. &&
			it->z > -2.1 &&
			it->z < 0.0){

				int ring = it->ring;

				double distance = it->distance;
				int counter = ring_counters.at(ring);
				//std::cout << ring_counters.at(1) << '\n';


				if(counter != 0){
					double der = it->distance - last_distances_.at(ring);
					//std::cout << "der"<< der << '\n';
					if(der <= -depth_jump_dist){
						risingEdge(*it);
					} else if(der >= depth_jump_dist){

						if(rising_edge_.at(ring)){
							if (tmp_object_.at(ring).size() > 8	&&
							tmp_object_.at(ring).size() < 23

						)
						{
							objects_->insert(objects_->end(), tmp_object_.at(ring).begin(), tmp_object_.at(ring).end());

							if(rising_edge_.at(ring)){
								--on_obj_counter_rising_;
							}
							rising_edge_.at(ring) = false;
							tmp_object_.at(ring).clear();
							tmp_der_sum.at(ring) = 0.0;
						}

					}

					fallingEdge(*it);
				} else {


					if(rising_edge_.at(ring) && on_obj_counter_rising_ > 10){

						tmp_der_sum.at(ring) += fabs(der);

						tmp_object_.at(ring).push_back(*it);

					}
				}
			}
			last_distances_.at(ring) = it->distance;
			ring_counters.at(ring)++;
			last_points_.at(ring) = *it;
		}
	}




	temp_cloud_kd->points.resize(objects_->points.size());


	for (size_t i = 0; i < objects_->points.size(); i++) {

		temp_cloud_kd->points[i].x = objects_->points[i].x;
		temp_cloud_kd->points[i].y = objects_->points[i].y;
		temp_cloud_kd->points[i].z = objects_->points[i].z;


	}


	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

	kdtree.setInputCloud (temp_cloud_kd);
	//
	pcl::PointXYZ searchPoint;

	obj_filtered->points.resize(temp_cloud_kd->points.size());


	for (size_t i = 0;  i < temp_cloud_kd->points.size(); i++) {

		searchPoint.x = temp_cloud_kd->points[i].x;
		searchPoint.y = temp_cloud_kd->points[i].y;
		searchPoint.z = temp_cloud_kd->points[i].z;


		//############################################################

		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		float radius = 1.2;

		if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0&&
		(pointIdxRadiusSearch.size()>249) )
		{
			obj_filtered->push_back(searchPoint);
		}

	}

	obj_filtered->header.stamp = pcl_conversions::toPCL(msg->header).stamp;
	obj_filtered->header.frame_id = "velodyne";
	obj_filtered->height = 1;


	objects_->header.stamp = pcl_conversions::toPCL(msg->header).stamp;
	objects_->header.frame_id = msg->header.frame_id;
	objects_->height = 1;

	obj_filtered_pub_.publish(obj_filtered);
	poles_pub_.publish(poles_);
	pcl_pub_on_obj_.publish(objects_);
}

void Ming_Polls::risingEdge(DPoint point){
	int ring = point.ring;
	double distance = point.distance;

	if(!rising_edge_.at(ring)){
		++on_obj_counter_rising_;
	}
	rising_edge_.at(ring) = true;
	tmp_object_.at(ring).clear();
	if(on_obj_counter_rising_ > 10){
		tmp_object_.at(ring).push_back(point);
	}
}

void Ming_Polls::fallingEdge(DPoint point){
	int ring = point.ring;
	double distance = point.distance;

	if(!falling_edge_.at(ring)){
		++on_obj_counter_rising_;
	}
	falling_edge_.at(ring) = true;
	tmp_object_.at(ring).clear();
	if(on_obj_counter_rising_ > 10){
		tmp_object_.at(ring).push_back(point);
	}
}
