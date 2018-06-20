#include "polls3d/ming_polls.h"

using namespace std;
using namespace cv;


Ming_Polls::Ming_Polls(ros::NodeHandle nh) {
	pcl_sub_ = nh.subscribe("/velodyne_points", 10, &Ming_Polls::velodynePCLCallback, this);
	odom_sub_ = nh.subscribe("/imu/gps", 10, &Ming_Polls::GPSdataCallback, this);

	pcl_pub_on_obj_ = nh.advertise<sensor_msgs::PointCloud2>("/test_out", 10, this);
	//odom_pub_ = nh.advertise<geometry_msgs::PointStamped>("/odom", 10);
	odom_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/odom", 10, this);
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr odom_point_(new pcl::PointCloud<pcl::PointXYZ> ());


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

		if(/*it->distance > 2. &&
			it->distance < 20. &&*/
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

						// double mean_size =
						// (
						// 	tmp_object_.at(ring_m1).size()+
						// 	tmp_object_.at(ring_m2).size()+
						// 	tmp_object_.at(ring_m3).size()+
						// 	tmp_object_.at(ring_m4).size()+
						// 	tmp_object_.at(ring_m5).size()+
						// 	tmp_object_.at(ring_m6).size()+
						// 	tmp_object_.at(ring_m7).size())/7;



							std::vector<double> sum_dist = std::vector<double>(64,0.0);
							std::vector<double> mean_dist = std::vector<double>(64,0.0);
							cv::Mat odom = Mat::zeros(64,3, CV_32F);
							cv::Mat mean_odom = Mat::zeros(64,3, CV_32F);



							// for (size_t i = 0; i < tmp_object_.at(ring).points.size(); i++) {
							//
							// 	sum_dist.at(ring) = tmp_object_.at(ring).points[i].distance + sum_dist.at(ring);
							// 	//std::cout << odom.at<uchar>(1, 0) << '\n';
							// 	odom.at<float>(ring, 0) = tmp_object_.at(ring).points[i].x + odom.at<float>(ring, 0);
							// 	odom.at<float>(ring, 1) = tmp_object_.at(ring).points[i].y + odom.at<float>(ring, 1);
							// 	odom.at<float>(ring, 2) = tmp_object_.at(ring).points[i].z + odom.at<float>(ring, 2);
							//
							// 	//std::cout << odom.at<float>(ring, 0)<< '\n';
							//
							//
							// }

							// mean_dist.at(ring) = sum_dist.at(ring)/(tmp_object_.at(ring).points.size())+1;
							// mean_odom.at<float>(ring, 0) = odom.at<float>(ring, 0)/(tmp_object_.at(ring).points.size())+1;
							// mean_odom.at<float>(ring, 1) = odom.at<float>(ring, 1)/(tmp_object_.at(ring).points.size())+1;
							// mean_odom.at<float>(ring, 2) = odom.at<float>(ring, 2)/(tmp_object_.at(ring).points.size())+1;
							//

							//std::cout << mean_odom.at<float>(ring, 0) << '\n';

							//for (size_t i = 0; i < tmp_object_.at(ring).points.size(); i++) {
							if(rising_edge_.at(ring)){
								if (tmp_object_.at(ring).size() > 5	&&
								tmp_object_.at(ring).size() < 23 //&&
								//fabs(tmp_object_.at(ring).points[i].x - mean_odom.at<float>(ring, 0))>0.5 &&
								//fabs(tmp_object_.at(ring).points[i].x - mean_odom.at<float>(ring, 0))<1.5
								// fabs(tmp_object_.at(ring).points[i].y - mean_odom.at<float>(ring, 1))<1. &&
								//fabs(tmp_object_.at(ring).points[i].distance - mean_dist.at(ring)) <= 1.
								//换成xyz坐标

								// 	tmp_object_.at(ring_m1).size() > 5 &&
								// 	tmp_object_.at(ring_m1).size() <30
							)

							// if (mean_size>=7 &&
							// 	mean_size<=30&&
							// 	tmp_object_.at(ring_m1).size() <= 30 &&
							// 	tmp_object_.at(ring_m3).size() <= 30 &&
							// 	tmp_object_.at(ring_m2).size() <= 30 &&
							// 	tmp_object_.at(ring_m4).size() <= 30 &&
							// 	tmp_object_.at(ring_m6).size() <= 30 &&
							// 	tmp_object_.at(ring_m5).size() <= 30 &&
							// 	tmp_object_.at(ring_m7).size() <= 30 )


							{// && tmp_der_sum.at(ring) <= 1. && tmp_on_object.at(ring).size() < 200){
								// std::cout << "it-1 is 		"<<(it-1)->x << '\n';
								// std::cout << "it is 		"<<(it)->x << '\n';

								objects_->insert(objects_->end(), tmp_object_.at(ring).begin(), tmp_object_.at(ring).end());



								//temp_cloud_kd->points.resize(objects_->points.size());


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



		size_t j = 0;
		for (size_t i = 0;  i < temp_cloud_kd->points.size(); i++) {

			searchPoint.x = temp_cloud_kd->points[i].x;
			searchPoint.y = temp_cloud_kd->points[i].y;
			searchPoint.z = temp_cloud_kd->points[i].z;


			//############################################################

			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;

			float radius = 1.2;

			// std::cout << "Neighbors within radius search at (" << searchPoint.x
			//           << " " << searchPoint.y
			//           << " " << searchPoint.z
			//           << ") with radius=" << radius << '\n';


			if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
			{



				if (pointIdxRadiusSearch.size()>250) {

					// obj_filtered->points[j].x = searchPoint.x;
					// obj_filtered->points[j].y = searchPoint.y;
					// obj_filtered->points[j].z = searchPoint.z;
					obj_filtered->push_back(searchPoint);
					j++;



					/* code */
				}
				// std::cout << "pointIdxRad		"<<j << '\n';
				// std::cout << "KD		" << temp_cloud_kd->points.size() << '\n';
				// std::cout << "OB		" <<obj_filtered->points.size() << '\n';









				// std::cout << "The Size is:	"<<temp_cloud_kd->points[ pointIdxRadiusSearch[i] ].size() << '\n';
				//
				//std::cout << "size is :"	<<pointIdxRadiusSearch.size () << '\n';
				// for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
				//   std::cout << "    "  <<   temp_cloud_kd->points[ pointIdxRadiusSearch[i] ].x
				//             << " " << temp_cloud_kd->points[ pointIdxRadiusSearch[i] ].y
				//             << " " << temp_cloud_kd->points[ pointIdxRadiusSearch[i] ].z
				//             << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
			}
			//############################################################


		}













		// obj_filtered->header.seq = temp_cloud->header.seq;
		// obj_filtered->header.frame_id = "velodyne";

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

	float Ori_x = 0.;
	float Ori_y = 0.;
	float Ori_z = 0.;

	void Ming_Polls:: GPSdataCallback(imu_3dm_gx4::GPSData odom_msg){

		geometry_msgs::PointStamped geo_point;
		//odom_point_ = DPointCloud::Ptr(new DPointCloud);

		pcl::PointCloud<pcl::PointXYZ>::Ptr odom_point_(new pcl::PointCloud<pcl::PointXYZ> ());
		odom_point_->points.resize(1);


		//Mat odom = Mat::zeros(64,odom_msg->points.size(), CV_32F);
		if (Ori_x == 0. && Ori_y == 0. && Ori_z == 0.) {
			Ori_x = (long double)odom_msg.position_x;
			Ori_y = (long double)odom_msg.position_y;
			Ori_z = (long double)odom_msg.position_z;

			//geo_point->point.x = (long double)odom_msg.position_x;
			//std::cout << geo_point.point.x << '\n';

			// geo_point.point.x = (long double)odom_msg.position_x - Ori_x;
			// geo_point.point.y = (long double)odom_msg.position_y - Ori_y;
			// geo_point.point.z = 0;

			odom_point_->points[0].x = (long double)odom_msg.position_x - Ori_x;
			odom_point_->points[0].y = (long double)odom_msg.position_y - Ori_y;
			odom_point_->points[0].z = 0;



			// std::cout << Ori_x << '\n';
			//
			// //geo_point->x = (long double)odom_msg.position_x - Ori_x;
			// //std::cout << geo_point->x  << '\n';
			//
			// std::cout << "x:		"<<fixed << setprecision(2) <<	(long double)odom_msg.position_x - Ori_x << '\n';
			// std::cout << "y:		"<<	(long double)odom_msg.position_y - Ori_y << '\n';
			// std::cout << "z:		"<<	(long double)odom_msg.position_z - Ori_z << '\n';
			// std::cout << "-------------------------------------------" << '\n';

		}else{
			// geo_point.point.x = (long double)odom_msg.position_x - Ori_x;
			// geo_point.point.y = (long double)odom_msg.position_y - Ori_y;
			// geo_point.point.z = 0;

			odom_point_->points[0].x = (long double)odom_msg.position_x - Ori_x;
			odom_point_->points[0].y = (long double)odom_msg.position_y - Ori_y;
			odom_point_->points[0].z = 0;

			// std::cout << Ori_x << '\n';
			//
			// std::cout << "x:		"<<fixed << setprecision(2) <<	(long double)odom_msg.position_x - Ori_x << '\n';
			// std::cout << "y:		"<<	(long double)odom_msg.position_y - Ori_y << '\n';
			// std::cout << "z:		"<<	(long double)odom_msg.position_z - Ori_z << '\n';
			// std::cout << "-------------------------------------------" << '\n';
		}

		//geo_point.header.stamp = pcl_conversions::toPCL(msg->header).stamp;
		// geo_point.header.frame_id = "velodyne";
		//std::cout << "Odom_point:"<< odom_point_->points[0] << '\n';

		odom_point_->header.stamp = odom_msg.time;
		odom_point_->header.frame_id = "velodyne";
		odom_point_->height = 1;

		odom_pub_.publish(odom_point_);

	}
