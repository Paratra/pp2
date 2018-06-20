#include "polls3d/polls3d_ming.h"
#include <Eigen/Dense>

#define counter 1



//########## CONSTRUCTOR ###############################################################################################
polls3d_ming_node::polls3d_ming_node(ros::NodeHandle &node_handle):
node_(&node_handle)
{
  // === PARAMETERS ===
  // node_->param("my_pkg/my_double_param", my_double_parameter_, 0.0);
  // node_->param("my_pkg/my_string_param", my_string_parameter_, std::string(""));
  // node_->param("my_pkg/my_int_param", my_int_parameter_, 0);
  // node_->param("my_pkg/my_bool_param", my_bool_parameter_, false);

  // === SUBSCRIBERS ===
  //my_subscriber_ = node_->subscribe("/joy", 10, &MyNode::subscriberCallback, this);
  //Turtle_subscriber_ = node_->subscribe("/teleop_twist_keyboard", 10, &MyNode::subscriberCallback, this);

  polls3d_ming_sub_ = node_->subscribe("/velodyne_points", 1000, &polls3d_ming_node::polls3d_mingCallback, this);

  // === PUBLISHERS ===
  polls3d_ming_pub_ = node_->advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/test_output", 1, this);
  //test_pub_ = node_->advertise<std_msgs::String>("chatter", 1000);
  //Turtle_publisher_ = node_->advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

  // === SERVICE SERVERS ===
  //my_service_server_ = node_->advertiseService("my_namespace/my_service", &MyNode::serviceCallback, this);

  // === SERVICE CLIENTS ===
  //my_service_client_ = node_->serviceClient<std_srvs::Empty>("some_service");

  // === TIMER ===
  //my_timer_ = node_->createTimer(ros::Duration(10), &MyNode::timerCallback, this);

  ROS_INFO("ROS Node Started");
}

//########## CALLBACK: SUBSCRIBER ######################################################################################
// void MyNode::subscriberCallback(const sensor_msgs::Joy &msg)
// {
//     geometry_msgs::Twist new_msg;
//     ROS_INFO("Benutz Steuerung");
//     new_msg.linear.x=(msg.axes[1])*0.5;
//     new_msg.linear.y=(msg.axes[0])*0.5;
//     new_msg.angular.z=(msg.axes[2])*-1;
// 	ROS_INFO("Message received: %f", msg.axes[1]);
// 	my_publisher_.publish(new_msg);
// 	Turtle_publisher_.publish(new_msg);
// }


//###############################

// void polls3d_ming_node::pole_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud){
//
//   for (size_t i = 0; i < temp_cloud->points.size(); i++) {
//
//     geometry_msgs::Point32 pointset_for_extc;
//     sensor_msgs::PointCloud pointcloud_msg;
//
//     pointset_for_extc.x = temp_cloud->points[i].x;
//     pointset_for_extc.y = temp_cloud->points[i].y;
//     pointset_for_extc.z = temp_cloud->points[i].z;
//
//     pointcloud_msg.points.push_back(pointset_for_extc);
//     //unsigned int a = msg->packets[1].data[i];
//     // std::cout << "x:" << temp_cloud->points[i].x<<'\n';
//     // std::cout << "y:" << temp_cloud->points[i].y<<'\n';
//     // std::cout << "z:" << temp_cloud->points[i].z<<'\n';
//   }
//   //return pointcloud_msg;
// }


//#################################

// void polls3d_ming_node::heightThresholder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_color_cut, pcl::PointCloud<pcl::PointXYZRGB>::Ptr processCloud, double zMin, double zMax){
// 	// Create the filtering object
// 	pcl::PassThrough<pcl::PointXYZRGB> pass;
// 	pass.setInputCloud (*processCloud);
// 	pass.setFilterFieldName ("z");
//   	pass.setFilterLimits (zMin, zMax);
//   	pass.filter(*temp_cloud_color_cut);
// }

//#################################




float getTwoVectorsAngle(const Eigen::Vector3d & vector_1, const Eigen::Vector3d & vector_2)
{

  //std::cout << acos(-1) << '\n';

  float angle =   vector_1.dot(vector_2) /  (sqrt(vector_1[0]*vector_1[0]+vector_1[1]*vector_1[1]+vector_1[2]*vector_1[2]) *
  sqrt(vector_2[0]*vector_2[0]+vector_2[1]*vector_2[1]+vector_2[2]*vector_2[2]));

  float angle_arc = acos(angle);

  return angle_arc;
}


//#################################

void polls3d_ming_node::polls3d_mingCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

  //#### sensor_msgs to PCL
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg,pcl_pc2);

  //pcl::PointXYZRGB temp_cloud_color;

  //pcl::PointCloud<pcl::PointXYZRGB> temp_cloud_color;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_color(new pcl::PointCloud<pcl::PointXYZRGB> ());

  DPointCloud::Ptr temp_cloud(new DPointCloud());
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  //pointcloud_msg = pole_extraction(temp_cloud);

  // sensor_msgs::PointCloud pointcloud_msg;
  // std_msgs::Header point_header;

  temp_cloud_color->points.resize(temp_cloud->points.size());
  for (size_t i = counter*64; i < temp_cloud->points.size(); i++) {

    temp_cloud_color->points[i].x = temp_cloud->points[i].x;
    temp_cloud_color->points[i].y = temp_cloud->points[i].y;
    temp_cloud_color->points[i].z = temp_cloud->points[i].z;





    if( //( i >= 3 ) &&
    (((temp_cloud_color->points[i].z)>=-2.0) &&
    ((temp_cloud_color->points[i].z)<=0)) &&

    (((temp_cloud_color->points[i].x)>=1.0) ||
    ((temp_cloud_color->points[i].x)<-2.0)) &&

    (((temp_cloud_color->points[i].y)>=1.0) ||
    ((temp_cloud_color->points[i].y)<=-1.0)) &&

    ((abs(   sqrt( pow(temp_cloud_color->points[i].x, 2) + pow(temp_cloud_color->points[i].y, 2) + pow(temp_cloud_color->points[i].z, 2) )-
    sqrt( pow(temp_cloud_color->points[i-counter*64].x, 2) + pow(temp_cloud_color->points[i-counter*64].y, 2) + pow(temp_cloud_color->points[i-counter*64].z, 2) ) ) )>3) &&

    ((abs(   sqrt( pow(temp_cloud_color->points[i].x, 2) + pow(temp_cloud_color->points[i].y, 2) + pow(temp_cloud_color->points[i].z, 2) )-
    sqrt( pow(temp_cloud_color->points[i-counter*64].x, 2) + pow(temp_cloud_color->points[i-counter*64].y, 2) + pow(temp_cloud_color->points[i-counter*64].z, 2) ) ) )<20) &&

    (sqrt(pow(temp_cloud_color->points[i].x, 2)+pow(temp_cloud_color->points[i].y, 2)+pow(temp_cloud_color->points[i].z, 2))<15) &&

    (sqrt(pow(temp_cloud_color->points[i].x, 2)+pow(temp_cloud_color->points[i].y, 2)+pow(temp_cloud_color->points[i].z, 2))>1.5) //&&
  /*(abs(temp_cloud_color->points[i].z - temp_cloud_color->points[i-3].z)< 0.8)*/) {

  temp_cloud_color->points[i].r = 255;
  temp_cloud_color->points[i].g = 0;
  temp_cloud_color->points[i].b = 0;

}else{
  temp_cloud_color->points[i].r = 255;
  temp_cloud_color->points[i].g = 255;
  temp_cloud_color->points[i].b = 255;
}

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_color_filter(new pcl::PointCloud<pcl::PointXYZRGB> (*temp_cloud_color));

//#########################################

Eigen::Vector3d v(0, 0, 5);
//Eigen::Vector3d m(0, 0, -5);

//float angle_sec = getTwoVectorsAngle(v, m);
//std::cout << "angle_sec:" <<angle_sec << '\n';


//####################################### try 2

// std::vector<int> red_points;
// for (size_t i = 0; i < temp_cloud_color->points.size(); i++) {
//   if (temp_cloud_color->points[i].r == 255 &&
//     temp_cloud_color->points[i].g == 0 &&
//     temp_cloud_color->points[i].b == 0 ){
//       //std::cout << i << '\n';
//       red_points.push_back(i);
//
//     }
//   }


  //############ KD-TREE   #############
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

  kdtree.setInputCloud (temp_cloud_color);

  pcl::PointXYZRGB searchPoint;

  for (size_t i = 0; i < temp_cloud_color->points.size(); i++) {
    if (temp_cloud_color->points[i].r == 255 &&
      temp_cloud_color->points[i].g == 0 &&
      temp_cloud_color->points[i].b == 0 ){

        searchPoint.x = temp_cloud_color->points[i].x;
        searchPoint.y = temp_cloud_color->points[i].y;
        searchPoint.z = temp_cloud_color->points[i].z;


        //############################################################
        int K = 15;

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

  // std::cout << "K nearest neighbor search at (" << searchPoint.x
  // << " " << searchPoint.y
  // << " " << searchPoint.z
  // << ") with K=" << K << std::endl;


  //################## KDKDKDKDKDK#############################

    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      double sum_pointNKNSquaredDistance = std::accumulate(pointNKNSquaredDistance.begin(), pointNKNSquaredDistance.end(), 0.0);
      double mean_pointNKNSquaredDistance =  sum_pointNKNSquaredDistance / pointNKNSquaredDistance.size();
      //std::cout << mean_pointNKNSquaredDistance << '\n';
      //asrand((unsigned)time(NULL));

      for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){
        //int rand_num = (rand() % (14-0+1))+ 0;
        //std::cout << rand_num << '\n';

        // Eigen::Vector3d m(temp_cloud_color->points[ pointIdxNKNSearch[0] ].x -temp_cloud_color->points[ pointIdxNKNSearch[14] ].x ,
        //   temp_cloud_color->points[ pointIdxNKNSearch[0] ].y-temp_cloud_color->points[ pointIdxNKNSearch[14] ].y,
        //   temp_cloud_color->points[ pointIdxNKNSearch[0] ].z-temp_cloud_color->points[ pointIdxNKNSearch[14] ].z);

        //float angle_sec = getTwoVectorsAngle(v, m);
        //std::cout << angle_sec << '\n';

        if((double)temp_cloud_color->points[ pointIdxNKNSearch[i] ].r == 255 &&
        (double)temp_cloud_color->points[ pointIdxNKNSearch[i] ].g == 0 &&
        (double)temp_cloud_color->points[ pointIdxNKNSearch[i] ].b == 0 //&&
        //mean_pointNKNSquaredDistance <= 2.0 //&&
        // angle_sec < 1.2 &&
        // angle_sec > 2.0
      ){
        temp_cloud_color_filter->points[ pointIdxNKNSearch[i] ].r = 255;
        temp_cloud_color_filter->points[ pointIdxNKNSearch[i] ].g = 0;
        temp_cloud_color_filter->points[ pointIdxNKNSearch[i] ].b = 0;
      }else{
        temp_cloud_color_filter->points[ pointIdxNKNSearch[i] ].r = 255;
        temp_cloud_color_filter->points[ pointIdxNKNSearch[i] ].g = 255;
        temp_cloud_color_filter->points[ pointIdxNKNSearch[i] ].b = 255;



        // std::cout << "    "  <<   temp_cloud_color->points[ pointIdxNKNSearch[i] ].x
        // << " " << temp_cloud_color->points[ pointIdxNKNSearch[i] ].y
        // << " " << temp_cloud_color->points[ pointIdxNKNSearch[i] ].z
        // << "    "  <<   (double)temp_cloud_color->points[ pointIdxNKNSearch[i] ].r
        // << " " << (double)temp_cloud_color->points[ pointIdxNKNSearch[i] ].g
        // << " " << (double)temp_cloud_color->points[ pointIdxNKNSearch[i] ].b
        // << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
      }
    }
  }

  //############################################################


    }
    else{
      //ROS_INFO("There are no poles!");
      //std::cout << mean_pointNKNSquaredDistance << '\n';
    }

  }


  //################## KDKDKDKDKDK#############################

  //############################################
  temp_cloud_color->header.seq = temp_cloud->header.seq;
  temp_cloud_color->header.frame_id = "velodyne";
  temp_cloud_color_filter->header.seq = temp_cloud->header.seq;
  temp_cloud_color_filter->header.frame_id = "velodyne";
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_color_cut;
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr processCloud;
  //heightThresholder(temp_cloud_color_cut, processCloud, -2.0, -0.5);



  polls3d_ming_pub_.publish(temp_cloud_color_filter);

}


//########## CALLBACK: SERVICE SERVER ##################################################################################
// bool MyNode::serviceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
// {
//     ROS_INFO("Service call received.");
//     return true;
// }

//########## CALLBACK: TIMER ###########################################################################################
// void MyNode::timerCallback(const ros::TimerEvent &evt)
// {
//     // === PUBLISH A MESSAGE ===
//     //std_srvs::Empty srv;
//     //my_service_client_.call(srv);
//
//     //my_publisher_.publish(msg);
//
//     // === CALL A SERVICE ===
//     std_srvs::Empty srv;
//     my_service_client_.call(srv);
// }

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
  ros::init(argc, argv, "polls3d_ming_node_name");

  ros::NodeHandle node_handle;
  polls3d_ming_node polls3d_ming_node(node_handle);



  //ROS_INFO("DingDingDingDing....");
  ros::spin();

  return 0;
}
