#include "correspondence_finder2d.h"
#include <stdexcept>

namespace tsm {
  void CorrespondenceFinder2D::init() {
    _correspondences.clear();
    _indices_current.clear();

    if(_projector == NULL || _solver == NULL) {
      throw std::runtime_error("Cannot init correspondences without a projector and a solver");
    }

    _projector->project(
			_projected_reference_ranges,
			_indices_reference,
			Eigen::Isometry2f::Identity(),
			*_solver->reference()
			);
  }

  void CorrespondenceFinder2D::compute() {
    _correspondences.clear();
    _indices_current.clear();

    if(_projector == NULL || _solver == NULL) {
      throw std::runtime_error("Cannot compute correspondences without a projector and a solver");
    }

    _projector->project(
			_projected_current_ranges,
			_indices_current,
			_solver->T(),
			*_solver->current()
			);

    size_t size = std::min(_indices_current.size(), _indices_reference.size());

    for (size_t i = 0; i < size; ++i) {
      int idx1 = _indices_current[i];
      int idx2 = _indices_reference[i];
       
      if (idx1 < 0 || idx2 < 0)
	continue;

      RichPoint2D pt_curr = (*_solver->current())[idx1];
      RichPoint2D pt_ref = (*_solver->reference())[idx2];

      if((pt_curr.point() - pt_ref.point()).squaredNorm() < 0.09 &&
	 pt_curr.normal().dot(pt_ref.normal()) > 0.7) {
	_correspondences.push_back(i);
      }
    }
  }

  void CorrespondenceFinder2D::drawCorrespondences(UnsignedCharImage& img) const {
    UnsignedCharImage tmp;

    _solver->current()->draw(tmp, true);
    _solver->reference()->draw(tmp, true);

    for (size_t i = 0; i < _correspondences.size(); ++i) {
       int idx1 = _indices_current[_correspondences[i]];
       int idx2 = _indices_reference[_correspondences[i]];

       Eigen::Vector2f pt_curr = (*_solver->current())[idx1].point() * 10;
       Eigen::Vector2f pt_ref = (*_solver->reference())[idx2].point() * 10;

       float center_x = tmp.cols * 0.5;
       float center_y = tmp.rows * 0.5;

       cv::line(
		tmp,
		cv::Point(pt_curr.x() + center_x, pt_curr.y() + center_y),
		cv::Point(pt_ref.x() + center_x, pt_ref.y() + center_y),
		255
		);
    }

    img = tmp;
  }
}

