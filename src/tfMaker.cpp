#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <niv_comm/FcInfo.h>
#include <niv_comm/GimbalInfo.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <niv_comm/VuInfo.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;
sensor_msgs::PointCloud2::Ptr publish_cloud;

std::vector<float> lidarX;
std::vector<float> lidarY;

ros::Publisher pub;

float N;
float E;
float D;

float roll;
float pitch;
float yaw;

float gimbal_pitch;

float d2r;


void poseCallback(const niv_comm::FcInfoConstPtr& msg){
    N = msg->pos_n;
    E = msg->pos_e;
    D = msg->pos_d;

    roll = msg->euler_roll;
    pitch = msg->euler_pitch;
    yaw = msg->euler_yaw;
}

void gimbalCallback(const niv_comm::GimbalInfoConstPtr& msg){
    d2r = M_PI / 180;
    gimbal_pitch = msg->pitch_cur * d2r;
}


void tfPub(float posN, float posE, float posD, float fcRoll, float fcPitch, float fcYaw, float gimPitch){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(posN, posE, posD) );
  tf::Quaternion q;
  q.setRPY(fcRoll, fcPitch, fcYaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "/fc/info"));

  static tf::TransformBroadcaster br2;
  tf::Transform transform2;
  transform2.setOrigin( tf::Vector3(0, 0, -1));
  tf::Quaternion q2;
  std::cout << "gimbalpitch" << std::endl;
  std::cout << gimPitch << std::endl;
  q2.setRPY(0, gimPitch, 0);
  transform2.setRotation(q2);
  br2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "/fc/info", "/gimbal/info"));
}

void topc2cb (const niv_comm::VuInfoConstPtr& input){
    lidarX.clear();
    lidarY.clear();

    for (int i=0; i<input->vu_chunk_array.size(); i++){
        if ((input->vu_chunk_array[i].xrel != 0) & (input->vu_chunk_array[i].yrel != 0)){
            lidarX.push_back(input->vu_chunk_array[i].xrel);
            lidarY.push_back(input->vu_chunk_array[i].yrel);
        }
    }

    temp_cloud->points.resize(lidarX.size());
    for (int i=0; i<lidarX.size(); i++){

        temp_cloud->points[i].x = lidarX[i];
        temp_cloud->points[i].y = lidarY[i];
        temp_cloud->points[i].z = 0;

    }


    pcl::toROSMsg(*temp_cloud, *publish_cloud);
    publish_cloud->header.frame_id = "/gimbal/info";

    pub.publish(*publish_cloud);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;
  temp_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  publish_cloud = sensor_msgs::PointCloud2::Ptr(new sensor_msgs::PointCloud2);

  ros::Subscriber sub_fcInfo = node.subscribe("/fc/info", 10, poseCallback);
  ros::Subscriber sub_gimbal = node.subscribe("/gimbal/info", 1, gimbalCallback);
  ros::Subscriber sub = node.subscribe ("/payload_lidar/info", 1, topc2cb);
  pub = node.advertise<sensor_msgs::PointCloud2>("/publish_cloud", 10);

  while(ros::ok()){
    tfPub(N, E, D, roll, pitch, yaw, gimbal_pitch);
    ros::spinOnce();
  }
  return 0;
};

