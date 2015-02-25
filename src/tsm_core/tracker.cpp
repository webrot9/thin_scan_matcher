#include "tracker.h"

namespace tsm {
  inline double tv2sec(struct timeval& tv) {
    return (double) tv.tv_sec + 1e-6 * (double) tv.tv_usec;
  }

  //! returns the system time in seconds
  double getTime() {
    struct timeval tv;
    gettimeofday(&tv, 0);
    return tv2sec(tv);
  }

  Tracker::Tracker() {
    _iterations = 30;
    _bpr = 0.05;
    _global_t.setIdentity();
    _reference = NULL;
    _current = NULL;
    _solver = NULL;
    _projector = NULL;
    _correspondence_finder = NULL;
  }

  Tracker::~Tracker() {
    if (_reference != NULL)
      delete _reference;

    if (_current != NULL)
      delete _current;

    if (_solver != NULL)
      delete _solver;

    if (_projector != NULL)
      delete _projector;
  }

  void Tracker::update() {
    double init = getTime();
    float num_points = 0;
    float outliers = 0;
    float inliers = 0;
    float mean_dist = 0;
    FloatVector current_ranges, current_in_ref_ranges, reference_ranges;
    IntVector current_indices, current_in_ref_indices, reference_indices;

    if(_reference == NULL && _current == NULL) {
      throw std::runtime_error("Cannot track without data");
    } else if (_reference == NULL && _current != NULL) {
      _reference = _current;
      _global_t.setIdentity();
      return;
    } else if (_reference != NULL && _current == NULL) {
      return;
    }

    if(_projector == NULL || _solver == NULL) {
      throw std::runtime_error("Cannot track without a projector and a solver");
    }

    Eigen::Isometry2f global_t_inverse = _global_t.inverse();
    _solver->setReference(_reference);
    _solver->setCurrent(_current);
    _solver->setT(Eigen::Isometry2f::Identity());
    _correspondence_finder.init();
    _solver->setReferencePointsHint(_correspondence_finder.indicesReference());

    for (int i = 0; i < _iterations; ++i) {
      _correspondence_finder.compute();
      _solver->optimize(
		      _correspondence_finder.correspondences(),
		      _correspondence_finder.indicesCurrent(),
		      _correspondence_finder.indicesReference()
		      );
    }
   
    _current->transformInPlace(_solver->T());
    _projector->project(reference_ranges, reference_indices, Eigen::Isometry2f::Identity(), *_reference);
    _projector->project(current_ranges, current_indices, Eigen::Isometry2f::Identity(), *_current);

    int size = std::min(reference_indices.size(), current_indices.size());

    for (int c = 0; c < size; ++c) {
      int& ref_idx = reference_indices[c];
      int& curr_idx = current_indices[c];
      float diff = 0;

      if (ref_idx >= 0 && curr_idx >= 0){
	++num_points;
	diff = std::fabs(current_ranges[c] - reference_ranges[c]);

	mean_dist += diff;

	if (diff < 0.10)
	  ++inliers;
	else
	  ++outliers;
      }
    }

    if (num_points/size < 0.05) {
      if(_current != _reference)
	delete _current;
      return;
    }

    mean_dist /= num_points;
    float error = outliers/num_points;

    if (error <= _bpr) {
      _cloud_processor.merge(reference_ranges, reference_indices, *_reference,
			    current_ranges, current_indices, *_current,
			    1.f, 0.5f);

      _global_t = _global_t * _solver->T();
      _reference->clip(10.f, Eigen::Isometry2f::Identity());
      _reference->voxelize(*_reference, 1);
      _reference->transformInPlace(_solver->T().inverse());

      if (_reference != _current)
	delete _current;
    } else {
      if (_reference != _current)
	delete _reference;

      _reference = _current;
      std::cout << std::endl << "Track broken" << std::endl;
      std::cout << "Mean dist: " << mean_dist << std::endl;
      std::cout << "Outliers: " << outliers << std::endl;
      std::cout << "Inliers: " << inliers << std::endl;
      std::cout << "Outliers percentage: " << error << std::endl;
      std::cout << "Inliers percentage: " << 1 - error << std::endl;
    }

    double finish = getTime() - init;
    std::cout << "Hz: " << 1.f / finish << " points: " << _reference->size() << std::endl;
    std::cout.flush();
  }

  void Tracker::reset() {
    _global_t.setIdentity();

    if (_reference != NULL)
      delete _reference;

    if (_current != NULL)
      delete _current;
  }
}
