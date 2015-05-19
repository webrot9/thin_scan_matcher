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
  }

  void Tracker::setSolver(Solver2D* solver) {
    _solver = solver;
    _correspondence_finder.setSolver(_solver);
  }
  
  void Tracker::setProjector(Projector2D* projector) { 
    _projector = projector;
    _correspondence_finder.setProjector(_projector);
  }


  void Tracker::update(Cloud2D* cloud_) {
    if (cloud_)
      setCurrent(cloud_);
    double init = getTime();
    float num_points = 0;
    float outliers = 0;
    float inliers = 0;
    float mean_dist = 0;
    FloatVector current_ranges, current_in_ref_ranges, reference_ranges;
    IntVector current_indices, current_in_ref_indices, reference_indices;

    //std::cerr <<  "tracker update " <<  _reference << " " << _current << std::endl; 

    if (! _reference) {
      _reference = _current;
      _global_t.setIdentity();
      std::cerr <<  "1st cloud" << std::endl; 
      return;
    }

    if(_projector == 0 || _solver == 0) {
      throw std::runtime_error("Cannot track without a projector and a solver");
    }
    //cerr << "setting up solver" << endl;

    Eigen::Isometry2f global_t_inverse = _global_t.inverse();
    _solver->setReference(_reference);
    _solver->setCurrent(_current);
    _solver->setT(Eigen::Isometry2f::Identity());
    _correspondence_finder.init();
    _solver->setReferencePointsHint(_correspondence_finder.indicesReference());
    // cerr << "starting iterations " 
    // 	 << " reference_size:" << _reference->size()
    // 	 << " current_size:" << _current->size() << endl;

    for (int i = 0; i < _iterations; ++i) {
      _correspondence_finder.compute();
      _solver->optimize(
		      _correspondence_finder.correspondences(),
		      _correspondence_finder.indicesCurrent(),
		      _correspondence_finder.indicesReference()
		      );
    }
   
    //cerr << "done with iterations" << endl;

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
      if(_current && _current != _reference) {
	cerr << "bad num points, current: " << _current 
	     << " size: " << _current->size()
	     << " num_points: " << num_points << endl;
	//cerr << "deleting current (" << _current << ")" << endl;
	delete _current;
	//cerr << "done" << endl;
	_current=0;
      }
      return;
    }

    mean_dist /= num_points;
    float error = outliers/num_points;

    if (error <= _bpr) {
      //cerr << "merging... ";
      _cloud_processor.merge(reference_ranges, reference_indices, *_reference,
			    current_ranges, current_indices, *_current,
			    1.f, 0.5f);

      _global_t = _global_t * _solver->T();
      _reference->clip(10.f, Eigen::Isometry2f::Identity());
      _reference->voxelize(*_reference, 2);
      _reference->transformInPlace(_solver->T().inverse());

      //cerr << "done" << endl;

      if (_current && _reference != _current) {
	//cerr << "track good!" << endl;
	////cerr << "deleting current (" << _current << ")" << endl;
	delete _current;
	_current = 0;
	//cerr << "done" << endl;
      }
    } else {
      cerr << "track bad" << endl;
      cerr << "track broken" << endl;
      if (_reference  && _reference != _current) {
	//cerr << "deleting reference (" << _reference << ")" << endl;
	_reference = 0;
	//cerr << "done" << endl;
      }
      _reference = _current;
      std::cerr << std::endl << "Track broken" << std::endl;
      std::cerr << "Mean dist: " << mean_dist << std::endl;
      std::cerr << "Outliers: " << outliers << std::endl;
      std::cerr << "Inliers: " << inliers << std::endl;
      std::cerr << "Outliers percentage: " << error << std::endl;
      std::cerr << "Inliers percentage: " << 1 - error << std::endl;
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
