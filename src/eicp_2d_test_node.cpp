#include <fstream>
#include <sstream>

// ROS
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "nav_msgs/Odometry.h"

// OpenCV
#include "opencv2/opencv.hpp"

// tsm_core
#include "tsm_core/defs.h"
#include "tsm_core/cloud2d.h"
#include "tsm_core/projector2d.h"
#include "tsm_core/correspondence_finder2d.h"
#include "tsm_core/cloud_processor.h"
#include "tsm_core/solver2d.h"
#include "tsm_core/tracker.h"

#include "message_handler.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "tsm_test_node");

  std::string laser_topic;
  std::string camera_topic;
  std::string published_odom_topic;
  std::string camera_info_topic;
  int frame_skip;
  double bpr;

  MessageHandler message_handler;
  tsm::Solver2D solver;
  tsm::Projector2D* projector = NULL;
  tsm::Tracker tracker;
  tracker.setSolver(&solver);

  ros::NodeHandle n("~");
  
  // ROS parameters setting
  n.param("laser_topic", laser_topic, std::string("/scan"));
  n.param("camera_topic", camera_topic, std::string("/camera/depth/image_raw"));
  n.param("published_odom_topic", published_odom_topic, std::string("/odom_calib"));
  n.param("frame_skip", frame_skip, 1);
  n.param("bpr", bpr, 0.15);

  if (frame_skip <= 0)
    frame_skip = 1;

  printf("Launched with params:\n");
  printf("_laser_topic:= %s\n", laser_topic.c_str());
  printf("_camera_topic:= %s\n", camera_topic.c_str());
  printf("_published_odom_topic:= %s\n", published_odom_topic.c_str());
  printf("_frame_skip:= %d\n", frame_skip);
  printf("_bpr:= %f\n", bpr);
  fflush(stdout);

  // Setting input parameters
  unsigned found = camera_topic.find_last_of("/");
  camera_info_topic = camera_topic.substr(0, found) + "/camera_info";
  message_handler.setFrameSkip(frame_skip);
  tracker.setBpr(bpr);

  // ROS topic subscriptions
  ros::Subscriber laser_sub = n.subscribe(laser_topic, 100, &MessageHandler::laser_callback, &message_handler);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(published_odom_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> camera_sub(n, camera_topic, 100);
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub(n, camera_info_topic, 100);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), camera_sub, camera_info_sub);
  sync.registerCallback(boost::bind(&MessageHandler::camera_callback, &message_handler, _1, _2));

  tsm::Cloud2D *cloud = NULL;
  Eigen::Isometry2f global_t;

  while (ros::ok()) {
    std::list<tsm::Cloud2D*>* clouds = message_handler.clouds();

    if (clouds->size() > 0) {
      if (projector == NULL) {
	projector = message_handler.projector();
	tracker.setProjector(projector);
      }

      cloud = clouds->front();
      clouds->pop_front();

      tracker.update(cloud);
      global_t = tracker.globalT();

      nav_msgs::Odometry odom_msg;
      Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
      rot.block<2,2>(0, 0) = global_t.linear();
      Eigen::Quaternionf q(rot);
 
      odom_msg.header.stamp = ros::Time::now();
      odom_msg.pose.pose.position.x = global_t.translation().x();
      odom_msg.pose.pose.position.y = global_t.translation().y();
      odom_msg.pose.pose.position.z = 0;
      odom_msg.pose.pose.orientation.w = q.w();
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
 
      odom_pub.publish(odom_msg);
    }

    ros::spinOnce();
  }

  return 0;
}
