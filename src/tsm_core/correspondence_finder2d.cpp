#include "correspondence_finder2d.h"

namespace tsm {
  void CorrespondenceFinder2D::compute() {
    _correspondences.clear();
    _indicesCurrent.clear();
    if(_projector != NULL && _solver != NULL) {
      _projector->project(
			  _projectedReferenceRanges,
			  _indicesReference,
			  Eigen::Isometry2f::Identity(),
			  *_solver->reference
			  );

      _projector->project(
			  _projectedCurrentRanges,
			  _indicesCurrent,
			  _solver->T,
			  *_solver->current
			  );

      size_t size = std::min(_indicesCurrent.size(), _indicesReference.size());

      for (size_t i = 0; i < size; i++) {
	int idx1 = _indicesCurrent[i];
	int idx2 = _indicesReference[i];
       
	if (idx1 < 0 || idx2 < 0)
	  continue;

	RichPoint2D pt_curr = (*_solver->current)[idx1];
	RichPoint2D pt_ref = (*_solver->reference)[idx2];

	if((pt_curr.point() - pt_ref.point()).squaredNorm() < 0.09 &&
	  pt_curr.normal().dot(pt_ref.normal()) > 0.7) {
	  _correspondences.push_back(i);
	}
      }
    } else {
      std::cerr << "Cannot compute correspondences without a projector and a solver" << std::endl;
    }
  }

  void CorrespondenceFinder2D::drawCorrespondences(UnsignedCharImage& img){
    UnsignedCharImage tmp;

    _solver->current->draw(tmp, true);
    _solver->reference->draw(tmp, true);

    for (size_t i = 0; i < _correspondences.size(); ++i) {
       int idx1 = _indicesCurrent[_correspondences[i]];
       int idx2 = _indicesReference[_correspondences[i]];

       Eigen::Vector2f pt_curr = (*_solver->current)[idx1].point() * 10;
       Eigen::Vector2f pt_ref = (*_solver->reference)[idx2].point() * 10;

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

