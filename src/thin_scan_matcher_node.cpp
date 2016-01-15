#include <fstream>
#include <sstream>

// ROS
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include "message_filters/sync_policies/approximate_time.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/String.h"
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

#include "qapplication.h"
#include "tsm_core/tracker_viewer.h"

using namespace std;


struct CloudWithTime: public tsm::Cloud2D {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CloudWithTime(const ros::Time& t, const Eigen::Isometry2f& guess_=Eigen::Isometry2f::Identity()){
    timestamp=t;
   guess=guess_;
  }
  ros::Time timestamp;
  Eigen::Isometry2f guess;
};

class MessageHandler {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  MessageHandler();
  ~MessageHandler();
  void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);  
  void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);  

  // setter & getter
  inline void setFrameSkip(int frame_skip) { _frame_skip = frame_skip; }
  inline int frameSkip() const { return _frame_skip; }
  inline int laserFrameCount() const { return _frame_count; }
  inline std::list<CloudWithTime*>& clouds()  { return _clouds; }
  inline const tsm::Projector2D* projector() const { return _projector; }
 private:
  tsm::Projector2D* _projector;
  std::list<CloudWithTime*> _clouds;
  sensor_msgs::Imu _last_imu_msg;
  int _frame_count;
  int _frame_skip;
};

MessageHandler::MessageHandler() {
  _projector = NULL;
  _frame_count = 0;
  _frame_skip = 1;
  _last_imu_msg.header.stamp=ros::Time(0.0d);
}

MessageHandler::~MessageHandler() {
  if (_projector)
    delete _projector;
}

void MessageHandler::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ++_frame_count;

  if (_frame_count % _frame_skip != 0)
    return;

  if (_projector == NULL) {
    float fov = 0.f;
    fov = msg->angle_increment * msg->ranges.size();
    _projector = new tsm::Projector2D();
    _projector->setMaxRange(msg->range_max);
    _projector->setMinRange(msg->range_min);
    _projector->setFov(fov);
    _projector->setNumRanges(msg->ranges.size());
  }

  Eigen::Isometry2f guess;
  guess.setIdentity();
  if (_last_imu_msg.header.stamp.toSec()>0) {
     guess.translation().setZero();
     float angle=Eigen::AngleAxisf(Eigen::Quaternionf(_last_imu_msg.orientation.x,
					 _last_imu_msg.orientation.y,
					 _last_imu_msg.orientation.z,
					 _last_imu_msg.orientation.w).toRotationMatrix()).angle();
     guess.linear()=Eigen::Rotation2Df(angle).toRotationMatrix();
  }
CloudWithTime* current = new CloudWithTime(msg->header.stamp, guess);
  _projector->unproject(*current, msg->ranges);
  _clouds.push_back(current);
}

void MessageHandler::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
  _last_imu_msg= *msg;
}


int main(int argc, char **argv) {

  std::string laser_topic;
  std::string imu_topic;
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
  bool use_gui;

  ros::init(argc, argv, "tsm_test_node");
  ros::NodeHandle n("~");
  
  // ROS parameters setting
  n.param("laser_topic", laser_topic, std::string("/scan"));
  n.param("imu_topic", imu_topic, std::string(""));
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
  n.param("use_gui", use_gui, false);
  
  if (frame_skip <= 0)
    frame_skip = 1;

  std::cout << "Launched with params:" << std::endl;
  std::cout << "_laser_topic:=" << laser_topic << std::endl;
  std::cout << "_imu_topic:=" << imu_topic << std::endl;
  std::cout << "_published_odom_topic:=" << published_odom_topic << std::endl;
  std::cout << "_frame_skip:=" << frame_skip << std::endl;
  std::cout << "_bpr:=" << bpr << std::endl;
  std::cout << "_base_link_frame_id:=" << base_link_frame_id << std::endl;
  std::cout << "_published_tf_origin_frame_id:=" << published_tf_origin_frame_id << std::endl;
  std::cout << "_published_tf_destination_frame_id:=" << published_tf_destination_frame_id << std::endl;
  std::cout << "_current_map_topic:=" << current_map_topic << std::endl;
  std::cout << "_reference_map_topic:=" << reference_map_topic << std::endl;
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
  std::cout << "_use_gui:=" <<  (use_gui?"true":"false") << std::endl;


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

  ros::Subscriber imu_sub;
  if (imu_topic!="")
    imu_sub = n.subscribe(imu_topic, 100, &MessageHandler::imu_callback, &message_handler);

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>(published_odom_topic, 1);

  CloudWithTime *cloud = 0;
  Eigen::Isometry2f global_t;

  std::cerr << "Entering ROS loop" << std::endl;
  
  QApplication* app=0;
  tsm::TrackerViewer* viewer=0;
  if (use_gui) {
    app=new QApplication(argc, argv);
    viewer=new tsm::TrackerViewer(&tracker);
    viewer->init();
    viewer->show();
  }
  Eigen::Isometry2f previous_guess;
  bool has_guess=false;

  
  ros::Rate loop_rate(50);

  while (ros::ok()) {
    std::list<CloudWithTime*>& clouds = message_handler.clouds();

    float q_size=clouds.size();

    while (clouds.size() > 0) {
      if (projector == 0) {
	const tsm::Projector2D* mh_projector = message_handler.projector();
	projector = new tsm::Projector2D;

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

      if(has_guess) {
	tracker.update(cloud, previous_guess.inverse()*cloud->guess);
      } else {
	tracker.update(cloud);
	has_guess=true;
      }
      previous_guess=cloud->guess;

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

      cerr << "Max freq: " << 1.0/tracker.cycleTime() \
	   << " q_size:" << q_size
	   << " ref_pts: ";
      if (tracker.reference()){
	cerr << tracker.reference()->size();
      } else {
	cerr << "0";
      }
      cerr << endl;
    }

    loop_rate.sleep();
    ros::spinOnce();

    if (use_gui) {
      viewer->updateGL();
      app->processEvents();
    }
  }

  return 0;
}
