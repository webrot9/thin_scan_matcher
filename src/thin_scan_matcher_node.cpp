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
  std::string published_odom_topic;
  std::string published_tf_origin_frame_id;
  std::string published_tf_destination_frame_id;
  std::string current_map_topic;
  std::string reference_map_topic;
  std::string base_link_frame_id;
  double inlier_distance;
  int frame_skip;
  double bpr;
  bool publish_odom;
  bool publish_tf;

  MessageHandler message_handler;
  tsm::Solver2D solver;
  tsm::Projector2D* projector = 0;
  tsm::Tracker tracker;
  tracker.setSolver(&solver);

  ros::NodeHandle n("~");
  
  // ROS parameters setting
  n.param("laser_topic", laser_topic, std::string("/scan"));
  n.param("published_odom_topic", published_odom_topic, std::string("/odom_calib"));
  n.param("base_link_frame_id", base_link_frame_id, std::string("/base_link"));
  n.param("published_tf_origin_frame_id", published_tf_origin_frame_id, std::string("/tsm_frame"));
  n.param("published_tf_destination_frame_id", published_tf_destination_frame_id, std::string("/base_link"));
  n.param("current_map_topic", current_map_topic, std::string("current_map"));
  n.param("reference_map_topic", reference_map_topic, std::string("reference_map"));
  n.param("publish_odom", publish_odom, true);
  n.param("publish_tf", publish_tf, false);
  n.param("inlier_distance", inlier_distance,0.1);
  n.param("frame_skip", frame_skip, 1);
  n.param("bpr", bpr, 0.15);
  
  if (frame_skip <= 0)
    frame_skip = 1;

  printf("Launched with params:\n");
  printf("_laser_topic:= %s\n", laser_topic.c_str());
  printf("_published_odom_topic:= %s\n", published_odom_topic.c_str());
  printf("_frame_skip:= %d\n", frame_skip);
  printf("_bpr:= %f\n", bpr);
  printf("_base_link_frame_id:=%s\n", base_link_frame_id.c_str());
  printf("_published_tf_origin_frame_id:=%s\n", published_tf_origin_frame_id.c_str());
  printf("_published_tf_destination_frame_id:=%s\n", published_tf_destination_frame_id.c_str());
  printf("_current_map_topic:=%s\n", current_map_topic.c_str());
  printf("_reference_map_topic:=%s\n", reference_map_topic.c_str());
  printf("_publish_odom:=%d\n", publish_odom);
  printf("_publish_tf:=%d\n", publish_tf);
  printf ("_inlier_distance:=%f\n", inlier_distance);
  fflush(stdout);

  // Setting input parameters
  message_handler.setFrameSkip(frame_skip);
  tracker.setBpr(bpr);
  tracker.setInlierDistance(inlier_distance);
  
  std::cerr << "Tracker allocated" << std::endl;
  std::cerr<< "inlier distance from tracker: "<<tracker.inlierDistance()<<std::endl;
  // ROS topic subscriptions
  ros::Subscriber laser_sub = n.subscribe(laser_topic, 100, &MessageHandler::laser_callback, &message_handler);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(published_odom_topic, 1);

  CloudWithTime *cloud = 0;
  Eigen::Isometry2f global_t;

  std::cerr << "Entering ROS loop" << std::endl;

  while (ros::ok()) {
    //std::cerr << "loopig" << std::endl;

    std::list<CloudWithTime*>& clouds = message_handler.clouds();

    if (clouds.size() > 0) {
      if (projector == 0) {
	projector = message_handler.projector();
	tracker.setProjector(projector);
      }
      
      CloudWithTime* cloud = clouds.front();
      ros::Time timestamp = cloud->timestamp;
      clouds.pop_front();

      
      tracker.update(cloud);
      global_t = tracker.globalT();

      nav_msgs::Odometry odom_msg;
      Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
      rot.block<2,2>(0, 0) = global_t.linear();
      Eigen::Quaternionf q(rot);
 
      odom_msg.header.stamp = timestamp;
      odom_msg.header.frame_id = "/odom";
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
    //usleep(10000);
  }

  return 0;
}
