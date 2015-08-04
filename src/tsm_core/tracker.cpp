#include <cstdio>
#include <fstream>

#include "tracker.h"

namespace tsm {
  using namespace std;

  inline double tv2sec(struct timeval& tv) {
    return (double) tv.tv_sec + 1e-6 * (double) tv.tv_usec;
  }

  Tracker::~Tracker() {
    reset();
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
    _reference = 0;
    _current = 0;
    _solver = 0;
    _projector = 0;
    _correspondence_finder = 0;
    _inlier_distance = 0.1;
    _min_correspondences_ratio = 0.5;
    _local_map_clipping_range = 5;
    _clip_translation_threshold = 5;
  }

  void Tracker::setSolver(Solver2D* solver) {
    _solver = solver;
    _correspondence_finder.setSolver(_solver);
  }
  
  void Tracker::setProjector(Projector2D* projector) { 
    _projector = projector;
    _correspondence_finder.setProjector(_projector);
  }


  void Tracker::update(Cloud2D* cloud_, const Eigen::Isometry2f& initial_guess) {
    if (cloud_)
      setCurrent(cloud_);
    double init = getTime();
    float num_correspondences = 0;
    float outliers = 0;
    float inliers = 0;
    float mean_dist = 0;
    FloatVector current_ranges, current_in_ref_ranges, reference_ranges;
    IntVector current_indices, current_in_ref_indices, reference_indices;

    if (! _reference) {
      _reference = _current;
      _global_t.setIdentity();
      _last_clipped_pose = _global_t;
      std::cerr <<  "1st cloud" << std::endl; 
      return;
    }

    if(_projector == 0 || _solver == 0) {
      throw std::runtime_error("Cannot track without a projector and a solver");
    }

    _solver->setReference(_reference);
    _solver->setCurrent(_current);
    _solver->setT(initial_guess);
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
	++num_correspondences;
	diff = std::fabs(current_ranges[c] - reference_ranges[c]);

	mean_dist += diff;

	if (diff < _inlier_distance)
	  ++inliers;
	else
	  ++outliers;
      }
    }

    float correspondences_ratio  = (float)num_correspondences/(float)_current->size();
    if (correspondences_ratio < _min_correspondences_ratio) {
      if(_current && _current != _reference) {
	std::cerr << "Too few correspondences:" << std::endl
		  << "current: " << _current 
		  << " size: " << _current->size()
		  << " num_correspondences: " << num_correspondences 
		  << " ratio: " << correspondences_ratio << std::endl;

	delete _current;
	_current = 0;
      }

      return;
    }

    mean_dist /= num_correspondences;
    float current_bad_points_ratio = outliers / num_correspondences;

    if (current_bad_points_ratio <= _bpr) {
      _cloud_processor.merge(reference_ranges, reference_indices, *_reference,
			    current_ranges, current_indices, *_current,
			    1.f, 0.5f);

      _global_t = _global_t * _solver->T();
      //_reference->voxelize(*_reference, 2);
      _reference->transformInPlace(_solver->T().inverse());

      if (_current && _reference != _current) {
	delete _current;
	_current = 0;
      }
    } else {
      _last_clipped_pose = _global_t;

      std::cerr << "Track bad" << std::endl;
      std::cerr << "Track broken" << std::endl;

      if (_reference  && _reference != _current) {
	_reference = 0;
      }

      _reference = _current;
      std::cerr << std::endl << "Track broken" << std::endl;
      std::cerr << "Mean dist: " << mean_dist << std::endl;
      std::cerr << "Outliers: " << outliers << std::endl;
      std::cerr << "Inliers: " << inliers << std::endl;
      std::cerr << "Outliers percentage: " << current_bad_points_ratio << std::endl;
      std::cerr << "Inliers percentage: " << 1 - current_bad_points_ratio << std::endl;
    }
    Eigen::Isometry2f delta_clip = _last_clipped_pose.inverse() * _global_t;
    if (delta_clip.translation().norm() > _clip_translation_threshold) {
      _reference->clip(_local_map_clipping_range);
      _last_clipped_pose=_global_t;		
      std::cerr << "Clipping" << std::endl;
    }
      

    double finish = getTime() - init;
    std::cerr << "Hz: " << 1.f / finish << " points: " << _reference->size() << std::endl;
    std::cerr.flush();
  }

  void Tracker::reset() {
    _global_t.setIdentity();

    if (_reference ){
      delete _reference;
      _reference=0;
    }

    if (_current) {
      delete _current;
      _current=0;
    }
  }
}
