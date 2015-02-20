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
#include "tsm_core/solver2d.h"

#include "message_handler.h"

inline char float2char(float d) {
  float alpha = 255/10;
  float d2 = 255-alpha*d;
  if (d2>255)
    d2 = 255;
  return d2;
}

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
  tsm::CorrespondenceFinder2D correspondence_finder;

  correspondence_finder.setSolver(&solver);

  ros::NodeHandle n("~");
  
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

  unsigned found = camera_topic.find_last_of("/");
  camera_info_topic = camera_topic.substr(0, found) + "/camera_info";
  message_handler.setFrameSkip(frame_skip);

  ros::Subscriber laser_sub = n.subscribe(laser_topic, 100, &MessageHandler::laser_callback, &message_handler);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(published_odom_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> camera_sub(n, camera_topic, 100);
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub(n, camera_info_topic, 100);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), camera_sub, camera_info_sub);
  sync.registerCallback(boost::bind(&MessageHandler::camera_callback, &message_handler, _1, _2));

  tsm::Cloud2D *reference = NULL;
  tsm::Cloud2D *current = NULL;  

  Eigen::Isometry2f global_t;

  float scale = 1;
  float offset_x = 0;
  float offset_y = 0;

  while (ros::ok()) {
    std::list<tsm::Cloud2D*>* clouds = message_handler.clouds();

    if (clouds->size() > 0) {
      if (projector == NULL) {
	projector = message_handler.projector();
	correspondence_finder.setProjector(projector);
      }


      if (reference == NULL) {
	reference = clouds->front();
	global_t.setIdentity();
      }

      current = clouds->front();
      clouds->pop_front();

      //current->transformInPlace(global_t);
      solver.reference = reference;
      solver.T.setIdentity();
      solver.current = current;
      solver.computeOmegas();
      correspondence_finder.compute();

      tsm::UnsignedCharImage correspondences_img;
      correspondence_finder.drawCorrespondences(correspondences_img);
      //cv::imshow("Correspondences", correspondences_img);

    
      for (int i = 0; i < 30; ++i) {
	correspondence_finder.compute();
	solver.oneRound(
			correspondence_finder.correspondences(),
			correspondence_finder.indicesCurrent(),
			correspondence_finder.indicesReference()
			);
      }

      tsm::FloatVector currentRanges, currentInRefRanges, referenceRanges;
      tsm::IntVector currentIndices, currentInRefIndices, referenceIndices;
      Eigen::Isometry2f global_t_inverse = global_t.inverse();
      
      current->transformInPlace(solver.T);
      projector->project(
			  referenceRanges,
			  referenceIndices,
			  Eigen::Isometry2f::Identity(),
			  *reference
			  );

      projector->project(
			  currentRanges,
			  currentIndices,
			  Eigen::Isometry2f::Identity(),
			  *current
			  );

      float num_points = 0;
      float outliers = 0;
      float inliers = 0;
      float mean_dist = 0;
      int size = std::min(referenceIndices.size(), currentIndices.size());

      tsm::UnsignedCharImage current_im, reference_im, refcurr_im;
      //current_im = cv::Mat::zeros(30, currentIndices.size(), CV_8UC1);

      // for (size_t c = 0; c < currentIndices.size(); c++) {
      // 	float d = 0;
      // 	if (currentIndices[c] >= 0) {
      // 	  d = currentRanges[c];
      // 	}
      // 	for (size_t r = 0; r < 10; r++) {
      // 	  current_im.at<uchar>(r, c) = float2char(d);
      // 	}
      // }

      // for(size_t c = 0; c < referenceIndices.size(); c++) {
      // 	float d = 0;
      // 	if (referenceIndices[c] >= 0) {
      // 	  d = referenceRanges[c];
      // 	}
      // 	for (size_t r = 20; r < 30; r++) {
      // 	  current_im.at<uchar>(r, c) = float2char(d);
      // 	}
      // }

      for (int c = 0; c < size; c++) {
	int& refIdx = referenceIndices[c];
	int& currIdx = currentIndices[c];
	float diff = 0;

	if (refIdx >= 0 && currIdx >= 0){
	  ++num_points;
	  diff = std::fabs(currentRanges[c] - referenceRanges[c]);

	  mean_dist += diff;

	  if (diff < 0.10)
	    ++inliers;
	  else
	    ++outliers;
	}

	//for(size_t r = 10; r < 20; r++)
	//current_im.at<uchar>(r, c) = float2char(diff);
 
      }

      //cv::imshow("Scans", current_im);
 
      if (num_points/size < 0.05) {
	if(current != reference)
	  delete current;
	continue;
      }

      mean_dist /= num_points;
      float error = outliers/num_points;

      tsm::UnsignedCharImage current_image;
      current->draw(current_image);
      //cv::imshow("Current Image", current_image);

      if(error <= bpr) {
	tsm::merge(referenceRanges, referenceIndices, *reference,
      		     currentRanges, currentIndices, *current,
		     0.8f, 0.25f);
	global_t = global_t * solver.T;
	reference->transformInPlace(solver.T.inverse());

	if(reference != current)
	  delete current;
      } else {
	if(reference != current)
	  delete reference;

	reference = current;
	std::cout << std::endl << "Track broken" << std::endl;
	std::cout << "Mean dist: " << mean_dist << std::endl;
	std::cout << "Outliers: " << outliers << std::endl;
	std::cout << "Inliers: " << inliers << std::endl;
	std::cout << "Outliers percentage: " << error << std::endl;
	std::cout << "Inliers percentage: " << 1 - error << std::endl;
      }

      tsm::UnsignedCharImage show;
      
      Eigen::Isometry2f im_tf;
      im_tf.linear() = global_t.linear()*scale;
      im_tf.translation() = (global_t.translation() + Eigen::Vector2f(offset_x, offset_y))*scale;
      reference->draw(show, false, global_t);

      if(show.size().area() > 0) {
	cv::imshow("LocalMap", show);
      }

      int tasto = cv::waitKey(1);
      
      nav_msgs::Odometry odom_msg;
      Eigen::Matrix3f rot = Eigen::Matrix3f::Identity();
      rot.block<2,2>(0, 0) = global_t.linear();
      Eigen::Quaternionf q(rot);
 
      odom_msg.header.stamp = ros::Time::now();
      odom_msg.pose.pose.position.x = global_t.translation().x();
      odom_msg.pose.pose.position.y = global_t.translation().y();
      odom_msg.pose.pose.position.z = global_t.translation().z();
      odom_msg.pose.pose.orientation.w = q.w();
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
 
      odom_pub.publish(odom_msg);

      switch(tasto) {
      case '+': scale *=1.1; break;
      case '-': scale /=1.1; break;
      case 1113938: offset_x += 1; break;
      case 1113940: offset_x -= 1; break;
      }
      
      std::cout << ".";
      std::cout.flush();
    }

    ros::spinOnce();
  }

  return 0;
}
