#pragma once

// ROS
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#include "tsm_core/cloud2d.h"
#include "tsm_core/projector2d.h"

struct CloudWithTime {
  ros::Time timestamp;
  tsm::Cloud2D* data;
};

class MessageHandler {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  MessageHandler() {
    _laser_projector = NULL;
    _camera_projector = NULL;
    _laser_frame_count = 0;
    _frame_skip = 1;
  }
  ~MessageHandler() {
    if (_laser_projector != NULL)
      delete _laser_projector;
    if (_camera_projector != NULL)
      delete _camera_projector;
  }
  void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ++_laser_frame_count;

    if (_laser_frame_count % _frame_skip != 0)
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

    CloudWithTime *current = NULL;
    tsm::Cloud2D *current_data = NULL;
    current = new CloudWithTime();
    current_data = new tsm::Cloud2D();

    _laser_projector->unproject(*current_data, msg->ranges);

    current->timestamp = msg->header.stamp;
    current->data = current_data;

    if(current_data != NULL && current != NULL)
      _clouds.push_back(current);
  }
  
void camera_callback(const sensor_msgs::Image::ConstPtr& image,
		       const sensor_msgs::CameraInfo::ConstPtr& camera_info) {
  ++_camera_frame_count;
  Eigen::Matrix3f camera_matrix;
  cv_bridge::CvImagePtr cvImagePtr = cv_bridge::toCvCopy(image);  
  tsm::UnsignedShortImage depth_image = cvImagePtr->image;

  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      camera_matrix(r, c) = camera_info->K[c + r * 3];

  if (_camera_frame_count % _frame_skip != 0)
    return;

  if (_camera_projector == NULL) {
    _camera_projector = new tsm::Projector2D();
    _camera_projector->setMaxRange(5.0);
    _camera_projector->setMinRange(0.20);
    _camera_projector->setFov(2*atan2(camera_matrix(0, 2), camera_matrix(0, 0)));
    _camera_projector->setNumRanges(depth_image.cols);
  }

  CloudWithTime *current = NULL;
  tsm::Cloud2D *current_data = NULL;
  current = new CloudWithTime();
  current_data = new tsm::Cloud2D();

  _camera_projector->unproject(*current_data, depth_image, camera_matrix, Eigen::Isometry3f::Identity());

  current->timestamp = image->header.stamp;
  current->data = current_data;

  if(current_data != NULL && current != NULL)
    _clouds.push_back(current);
}
  // setter & getter
  inline void setFrameSkip(int frame_skip) { _frame_skip = frame_skip; }
  inline int frameSkip() const { return _frame_skip; }
  inline int laserFrameCount() const { return _laser_frame_count; }
  inline int cameraFrameCount() const { return _camera_frame_count; }
  inline std::list<CloudWithTime*>* clouds()  { return &_clouds; }
  inline tsm::Projector2D* projector() {
    if (_camera_projector != NULL) return _camera_projector;

    return _laser_projector;
  }
 private:
  tsm::Projector2D* _laser_projector;
  tsm::Projector2D* _camera_projector;
  std::list<CloudWithTime*> _clouds;
  int _laser_frame_count;
  int _camera_frame_count;
  int _frame_skip;
};
  
