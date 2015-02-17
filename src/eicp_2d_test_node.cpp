#include <fstream>
#include <sstream>

// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"

// OpenCV
#include "opencv2/opencv.hpp"

// tsm_core
#include "tsm_core/defs.h"
#include "tsm_core/cloud2d.h"
#include "tsm_core/projector2d.h"
#include "tsm_core/correspondence_finder2d.h"
#include "tsm_core/solver2d.h"

#include "laser_handler.h"

inline char float2char(float d) {
  float alpha = 255/10;
  float d2 = 255-alpha*d;
  if (d2>255)
    d2 = 255;
  return d2;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tsm_test_node");

  std::string scan_topic;
  int frame_skip;
  double bpr;
  LaserHandler laser_handler;
  PSolver::Solver2D solver;
  PSolver::Projector2D* projector = NULL;
  PSolver::CorrespondenceFinder2D correspondence_finder;

  correspondence_finder.setSolver(&solver);

  ros::NodeHandle n("~");
  n.param("scan_topic", scan_topic, std::string("/scan"));
  n.param("frame_skip", frame_skip, 1);
  n.param("bpr", bpr, 0.15);

  if (frame_skip <= 0)
    frame_skip = 1;

  printf("Launched with params:\n");
  printf("_scan_topic:= %s\n", scan_topic.c_str());
  printf("_frame_skip:= %d\n", frame_skip);
  printf("_bpr:= %f\n", bpr);
  fflush(stdout);

  laser_handler.setFrameSkip(frame_skip);

  ros::Subscriber sub = n.subscribe(scan_topic, 100, &LaserHandler::laser_callback, &laser_handler);
  PSolver::Cloud2D *reference = NULL;
  PSolver::Cloud2D *current = NULL;  

  Eigen::Isometry2f global_t;

  float scale = 1;
  float offset_x = 0;
  float offset_y = 0;

  while (ros::ok()) {
    std::list<PSolver::Cloud2D*>* clouds = laser_handler.clouds();

    if (clouds->size() > 0) {
      if (projector == NULL) {
	projector = laser_handler.projector();
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

      PSolver::UnsignedCharImage correspondences_img;
      correspondence_finder.drawCorrespondences(correspondences_img);
      cv::imshow("Correspondences", correspondences_img);

    
      for (int i = 0; i < 30; ++i) {
	correspondence_finder.compute();
	solver.oneRound(
			correspondence_finder.correspondences(),
			correspondence_finder.indicesCurrent(),
			correspondence_finder.indicesReference()
			);
      }

      PSolver::FloatVector currentRanges, currentInRefRanges, referenceRanges;
      PSolver::IntVector currentIndices, currentInRefIndices, referenceIndices;
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

      PSolver::UnsignedCharImage current_im, reference_im, refcurr_im;
      current_im = cv::Mat::zeros(30, currentIndices.size(), CV_8UC1);

      for (size_t c = 0; c < currentIndices.size(); c++) {
	float d = 0;
	if (currentIndices[c] >= 0) {
	  d = currentRanges[c];
	}
	for (size_t r = 0; r < 10; r++) {
	  current_im.at<uchar>(r, c) = float2char(d);
	}
      }

      for(size_t c = 0; c < referenceIndices.size(); c++) {
	float d = 0;
	if (referenceIndices[c] >= 0) {
	  d = referenceRanges[c];
	}
	for (size_t r = 20; r < 30; r++) {
	  current_im.at<uchar>(r, c) = float2char(d);
	}
      }

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

	for(size_t r = 10; r < 20; r++)
	  current_im.at<uchar>(r, c) = float2char(diff);
 
      }

      cv::imshow("Scans", current_im);
 
      if (num_points/size < 0.05) {
	if(current != reference)
	  delete current;
	continue;
      }

      mean_dist /= num_points;
      float error = outliers/num_points;

      PSolver::Cloud2D* to_delete = NULL;
      if(error <= bpr) {
	PSolver::merge(referenceRanges, referenceIndices, *reference,
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

      PSolver::UnsignedCharImage show;
      
      Eigen::Isometry2f im_tf;
      im_tf.linear() = global_t.linear()*scale;
      im_tf.translation() = (global_t.translation() + Eigen::Vector2f(offset_x, offset_y))*scale;
      reference->draw(show, false, global_t);

      cv::imshow("LocalMap", show);
      int tasto = cv::waitKey(1);
      
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
