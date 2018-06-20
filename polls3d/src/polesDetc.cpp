#include "polls3d/polls3d_ming.h"
using namespace std;
using namespace cv;

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

//#################################


//#################################

void polls3d_ming_node::polls3d_mingCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{

  //#### sensor_msgs to PCL

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg,pcl_pc2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_color(new pcl::PointCloud<pcl::PointXYZRGB> ());

  DPointCloud::Ptr temp_cloud(new DPointCloud());
  //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);


  temp_cloud_color->points.resize(temp_cloud->points.size());

  Mat dist_of_scans = Mat::zeros(64,temp_cloud->points.size(), CV_32F);
  std::vector<double> Distances = std::vector<double>(64,0.0);
  std::vector<double> last_Points = std::vector<double>(64,0.0);
  std::vector<double> counter_ver = std::vector<double>(64,0);

  //int poles = 0;
  int haha = 0;
  for (size_t i = 0 ; i < temp_cloud->points.size(); i++) {


    // dist_of_scans.at<uchar>(temp_cloud->points[i].ring, i )=temp_cloud->points[i].distance;
    // std::cout << "distance is:"<< (double)temp_cloud->points[i].distance << '\n'<<"ring is:"<<
    //  temp_cloud->points[i].ring<<'\n';






    Distances.at(temp_cloud->points[i].ring) = temp_cloud->points[i].distance;

    double GAP = Distances.at(temp_cloud->points[i].ring) - last_Points.at(temp_cloud->points[i].ring);

    // std::cout << "result"<< " "<< GAP << '\n';
    // std::cout << "ring"<< " "<< temp_cloud->points[i].ring << '\n';

    if (  ((temp_cloud->points[i].distance)>=3.0) &&
    ((temp_cloud->points[i].distance)<=16.0) && ((temp_cloud->points[i].z)<=-1.9) )
      /*(((temp_cloud->points[i].z)<=-1.9) &&
      ((temp_cloud->points[i].z)<=0))) */{




      temp_cloud_color->points[i].x = temp_cloud->points[i].x;
      temp_cloud_color->points[i].y = temp_cloud->points[i].y;
      temp_cloud_color->points[i].z = temp_cloud->points[i].z;
      temp_cloud_color->points[i].r = 0;
      temp_cloud_color->points[i].g = 255;
      temp_cloud_color->points[i].b = 0;


        if (GAP > 0.2 ) {

          temp_cloud_color->points[i].x = temp_cloud->points[i].x;
          temp_cloud_color->points[i].y = temp_cloud->points[i].y;
          temp_cloud_color->points[i].z = temp_cloud->points[i].z;
          temp_cloud_color->points[i].r = 255;
          temp_cloud_color->points[i].g = 0;
          temp_cloud_color->points[i].b = 0;


        }else if ( GAP < -0.2 ){
          temp_cloud_color->points[i].x = temp_cloud->points[i].x;
          temp_cloud_color->points[i].y = temp_cloud->points[i].y;
          temp_cloud_color->points[i].z = temp_cloud->points[i].z;
          temp_cloud_color->points[i].r = 0;
          temp_cloud_color->points[i].g = 255;
          temp_cloud_color->points[i].b = 0;


        }
      }










      last_Points = Distances;
      //std::cout << temp_cloud->points[i].ring << '\n';
      // if (temp_cloud->points[i].ring > 61 || temp_cloud->points[i].ring <1  ) {
      //   std::cout << temp_cloud->points[i].ring << '\n';
      //   /* code */
      // }

      //counter_ver.at(temp_cloud->points[i].ring)++;



      //std::cout << "    " <<temp_cloud->points[i].ring  << '\n';


      // Distances.at(temp_cloud->points[i].ring) = temp_cloud->points[i].ring;
      // std::cout << Distances.at(63) << '\n';


      //std::cout << (double)dist_of_scans.at<uchar>(temp_cloud->points[i].ring, i ) << '\n';

      //imshow( "h", dist_of_scans);                // Show our image inside it.
      //cvWaitKey(1);

      //     if( //( i >= 3 ) &&
      //     (((temp_cloud_color->points[i].z)>=-2.0) &&
      //     ((temp_cloud_color->points[i].z)<=0)) &&
      //
      //     (((temp_cloud_color->points[i].x)>=0.6) ||
      //     ((temp_cloud_color->points[i].x)<-0.6)) &&
      //
      //     (((temp_cloud_color->points[i].y)>=0.7) ||
      //     ((temp_cloud_color->points[i].y)<=-0.7)) &&
      //
      //     ((abs(   sqrt( pow(temp_cloud_color->points[i].x, 2) + pow(temp_cloud_color->points[i].y, 2) + pow(temp_cloud_color->points[i].z, 2) )-
      //     sqrt( pow(temp_cloud_color->points[i-counter*64].x, 2) + pow(temp_cloud_color->points[i-counter*64].y, 2) + pow(temp_cloud_color->points[i-counter*64].z, 2) ) ) )>2.5) &&
      //
      //     ((abs(   sqrt( pow(temp_cloud_color->points[i].x, 2) + pow(temp_cloud_color->points[i].y, 2) + pow(temp_cloud_color->points[i].z, 2) )-
      //     sqrt( pow(temp_cloud_color->points[i-counter*64].x, 2) + pow(temp_cloud_color->points[i-counter*64].y, 2) + pow(temp_cloud_color->points[i-counter*64].z, 2) ) ) )<15) &&
      //
      //     (sqrt(pow(temp_cloud_color->points[i].x, 2)+pow(temp_cloud_color->points[i].y, 2)+pow(temp_cloud_color->points[i].z, 2))<15) &&
      //
      //     (sqrt(pow(temp_cloud_color->points[i].x, 2)+pow(temp_cloud_color->points[i].y, 2)+pow(temp_cloud_color->points[i].z, 2))>1.5) //&&
      //
      // {
      //
      //   temp_cloud_color->points[i].r = 255;
      //   temp_cloud_color->points[i].g = 0;
      //   temp_cloud_color->points[i].b = 0;
      //
      // }else{
      //   temp_cloud_color->points[i].r = 2;
      //   temp_cloud_color->points[i].g = 255;
      //   temp_cloud_color->points[i].b = 2;
      // }



      //
      // pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_color_filter(new pcl::PointCloud<pcl::PointXYZRGB> (*temp_cloud_color));
      //
      // //     //############################################
      // temp_cloud_color->header.seq = temp_cloud->header.seq;
      // temp_cloud_color->header.frame_id = "velodyne";
      // // temp_cloud_color_filter->header.seq = temp_cloud->header.seq;
      // // temp_cloud_color_filter->header.frame_id = "velodyne";
      //
      // polls3d_ming_pub_.publish(temp_cloud_color);

    }
    // std::cout <<"the first" <<"   "<<counter_ver.at(0) << '\n';
    // std::cout <<"the 30th"  <<"   "<<counter_ver.at(30) << '\n';
    // std::cout << "the last"  <<"   "<<counter_ver.at(62) << '\n';
    temp_cloud_color->header.seq = temp_cloud->header.seq;
    temp_cloud_color->header.frame_id = "velodyne";
    polls3d_ming_pub_.publish(temp_cloud_color);
  }

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
