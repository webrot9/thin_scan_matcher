#pragma once

// ROS
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/CameraInfo.h"

#include "tsm_core/cloud2d.h"
#include "tsm_core/projector2d.h"


struct CloudWithTime: public tsm::Cloud2D {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CloudWithTime(const ros::Time& t){
    timestamp=t;
  }
  ros::Time timestamp;
};

class MessageHandler {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  MessageHandler();
  ~MessageHandler();
  void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);  

  // setter & getter
  inline void setFrameSkip(int frame_skip) { _frame_skip = frame_skip; }
  inline int frameSkip() const { return _frame_skip; }
  inline int laserFrameCount() const { return _frame_count; }
  inline std::list<CloudWithTime*>& clouds()  { return _clouds; }
  inline const tsm::Projector2D* projector() const { return _projector; }
 private:
  tsm::Projector2D* _projector;
  std::list<CloudWithTime*> _clouds;
  int _frame_count;
  int _frame_skip;
};
  
