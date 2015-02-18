#pragma once

#include "sensor_msgs/LaserScan.h"

#include "tsm_core/cloud2d.h"
#include "tsm_core/projector2d.h"

class LaserHandler {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  LaserHandler() {
    _laser_projector = NULL;
    _frame_count = 0;
    _frame_skip = 1;
  }
  ~LaserHandler() {
    if (_laser_projector != NULL)
      delete _laser_projector;
  }
  void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ++_frame_count;

    if (_frame_count % _frame_skip != 0)
      return;

    if (_laser_projector == NULL) {
      float fov = 0.f;
      fov = msg->angle_increment * msg->ranges.size();
      _laser_projector = new tsm::Projector2D();
      _laser_projector->setMaxRange(10.f);
      _laser_projector->setMinRange(msg->range_min);
      _laser_projector->setFov(fov);
      _laser_projector->setNumRanges(msg->ranges.size());
    }

    tsm::Cloud2D *current = NULL;
    current = new tsm::Cloud2D();

    _laser_projector->unproject(*current, msg->ranges);

    if(current != NULL)
      _clouds.push_back(current);
  }
  
  // setter & getter
  inline void setFrameSkip(int frame_skip) { _frame_skip = frame_skip; }
  inline int frameSkip() const { return _frame_skip; }
  inline int frameCount() const { return _frame_count; }
  inline std::list<tsm::Cloud2D*>* clouds()  { return &_clouds; }
  inline tsm::Projector2D* projector()  { return _laser_projector; }
 private:
  tsm::Projector2D* _laser_projector;
  Eigen::Isometry2f _transform;
  std::list<tsm::Cloud2D*> _clouds;
  int _frame_count;
  int _frame_skip;
};
  
