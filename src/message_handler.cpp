#include "message_handler.h"

MessageHandler::MessageHandler() {
  _projector = NULL;
  _frame_count = 0;
  _frame_skip = 1;
}

MessageHandler::~MessageHandler() {}

void MessageHandler::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  ++_frame_count;

  if (_frame_count % _frame_skip != 0)
    return;

  if (_projector == NULL) {
    float fov = 0.f;
    fov = msg->angle_increment * msg->ranges.size();
    _projector = new tsm::Projector2D();
    _projector->setMaxRange(10.f);
    _projector->setMinRange(msg->range_min);
    _projector->setFov(fov);
    _projector->setNumRanges(msg->ranges.size());
  }

  CloudWithTime* current = new CloudWithTime(msg->header.stamp);
  _projector->unproject(*current, msg->ranges);
  _clouds.push_back(current);
}
  
  
