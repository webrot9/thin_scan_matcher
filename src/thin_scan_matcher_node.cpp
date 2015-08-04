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
#include "tf/transform_broadcaster.h"

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
  double min_correspondences_ratio;
  int iterations;

  double local_map_clipping_range;
  double local_map_clipping_translation_threshold;

  double max_matching_range;
  double min_matching_range;
  int num_matching_beams;
  double matching_fov;

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
  n.param("current_map_topic", current_map_topic, std::string("/current_map"));
  n.param("reference_map_topic", reference_map_topic, std::string("/reference_map"));
  n.param("publish_odom", publish_odom, true);
  n.param("publish_tf", publish_tf, false);
  n.param("inlier_distance", inlier_distance, 0.1);
  n.param("iterations", iterations, 10);
  n.param("frame_skip", frame_skip, 1);
  n.param("bpr", bpr, 0.2);
  n.param("min_correspondences_ratio", min_correspondences_ratio, 0.3);
  n.param("local_map_clipping_range", local_map_clipping_range, 10.0);
  n.param("local_map_clipping_translation_threshold", local_map_clipping_translation_threshold, 5.0);
  n.param("max_matching_range", max_matching_range, 0.0);
  n.param("min_matching_range", min_matching_range, 0.0);
  n.param("num_matching_beams", num_matching_beams, 0);
  n.param("matching_fov", matching_fov, 0.0);
  
  if (frame_skip <= 0)
    frame_skip = 1;

  std::cout << "Launched with params:" << std::endl;
  std::cout << "_laser_topic:=" << laser_topic.c_str() << std::endl;
  std::cout << "_published_odom_topic:=" << published_odom_topic.c_str() << std::endl;
  std::cout << "_frame_skip:=" << frame_skip << std::endl;
  std::cout << "_bpr:=" << bpr << std::endl;
  std::cout << "_base_link_frame_id:=" << base_link_frame_id.c_str() << std::endl;
  std::cout << "_published_tf_origin_frame_id:=" << published_tf_origin_frame_id.c_str() << std::endl;
  std::cout << "_published_tf_destination_frame_id:=" << published_tf_destination_frame_id.c_str() << std::endl;
  std::cout << "_current_map_topic:=" << current_map_topic.c_str() << std::endl;
  std::cout << "_reference_map_topic:=" << reference_map_topic.c_str() << std::endl;
  std::cout << "_publish_odom:=" << publish_odom << std::endl;
  std::cout << "_publish_tf:=" << publish_tf << std::endl;
  std::cout << "_inlier_distance:=" << inlier_distance << std::endl;
  std::cout << "_min_correspondences_ratio:=" << min_correspondences_ratio << std::endl;
  std::cout << "_local_map_clipping_range:=" << local_map_clipping_range << std::endl;
  std::cout << "_local_map_clipping_translation_threshold:=" << local_map_clipping_translation_threshold << std::endl;
  std::cout << "_max_matching_range:=" << max_matching_range << std::endl;
  std::cout << "_min_matching_range:=" << min_matching_range << std::endl;
  std::cout << "_num_matching_beams:=" << num_matching_beams << std::endl;
  std::cout << "_matching_fov:=" << matching_fov << std::endl;
  std::cout << "_iterations:=" <<  iterations << std::endl;

  fflush(stdout);

  // Setting input parameters
  message_handler.setFrameSkip(frame_skip);
  tracker.setBpr(bpr);
  tracker.setIterations(iterations);
  tracker.setInlierDistance(inlier_distance);
  tracker.setMinCorrespondencesRatio(min_correspondences_ratio);
  tracker.setLocalMapClippingRange(local_map_clipping_range);
  tracker.setClipTranslationThreshold(local_map_clipping_translation_threshold);
  std::cerr << "Tracker allocated" << std::endl;
  std::cerr << "inlier distance from tracker: "<< tracker.inlierDistance() << std::endl;
  // ROS topic subscriptions
  ros::Subscriber laser_sub = n.subscribe(laser_topic, 100, &MessageHandler::laser_callback, &message_handler);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(published_odom_topic, 1);

  CloudWithTime *cloud = 0;
  Eigen::Isometry2f global_t;

  std::cerr << "Entering ROS loop" << std::endl;

  while (ros::ok()) {
    std::list<CloudWithTime*>& clouds = message_handler.clouds();

    if (clouds.size() > 0) {
      if (projector == 0) {
	projector = new tsm::Projector2D;
	const tsm::Projector2D* mh_projector = message_handler.projector();

	if (max_matching_range != 0)
	  projector->setMaxRange(max_matching_range);
	else
	  projector->setMaxRange(mh_projector->maxRange());

	if (min_matching_range != 0)
	  projector->setMinRange(min_matching_range);
	else
	  projector->setMinRange(mh_projector->minRange());

	if (num_matching_beams != 0)
	  projector->setNumRanges(num_matching_beams);
	else
	  projector->setNumRanges(mh_projector->numRanges());

	if (matching_fov != 0)
	  projector->setFov(matching_fov);
	else
	  projector->setFov(mh_projector->fov());

	tracker.setProjector(projector);
      }
      
      CloudWithTime* cloud = clouds.front();
      ros::Time timestamp = cloud->timestamp;
      clouds.pop_front();

      
      tracker.update(cloud);
      global_t = tracker.globalT();

      Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
      rot.block<2,2>(0, 0) = global_t.linear();
      Eigen::Quaternionf q(rot);
      q.normalize();

      if (publish_odom) {
	nav_msgs::Odometry odom_msg;
	
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

      if (publish_tf) {
	static tf::TransformBroadcaster tf_broadcaster;
	const Eigen::Vector2f& t = global_t.inverse().translation();

	tf::Vector3 tf_translation(t.x(), t.y(), 0);
	Eigen::Quaternionf qi(rot.transpose());

	tf::Quaternion tf_quaternion(qi.x(), qi.y(), qi.z(), qi.w());

	tf::Transform tf_content;
	tf_content.setOrigin(tf_translation);
	tf_content.setRotation(tf_quaternion);

	tf::StampedTransform tf_msg(tf_content, timestamp, published_tf_destination_frame_id, published_tf_origin_frame_id);
	tf_broadcaster.sendTransform(tf_msg);
      }
    }

    ros::spinOnce();
    //usleep(10000);
  }

  return 0;
}
