#pragma once

#include <stdexcept>
#include <iostream>
#include <sys/time.h>

#include "cloud2d.h"
#include "solver2d.h"
#include "correspondence_finder2d.h"
#include "cloud_processor.h"

namespace tsm {
  class Tracker {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  public:
    Tracker();
    virtual ~Tracker();
    void update();
    inline void update(Cloud2D* cloud) {
      setCurrent(cloud);
      update();
    }
    inline void reset();

    // setter & getter
    inline void setIterations(const int &iterations) { _iterations = iterations; }
    inline void setBpr(const float &bpr) { _bpr = bpr; }
    inline Eigen::Isometry2f globalT() { return _global_t; }
    inline const Cloud2D* reference() const { return _reference; }
    inline void setReference(Cloud2D* reference) { _reference = reference; }
    inline const Cloud2D* current() const { return _current; }
    inline void setCurrent(Cloud2D* current) { _current = current; }
    inline const Solver2D* solver() const { return _solver; }
    inline void setSolver(Solver2D* solver) {
      _solver = solver;
      _correspondence_finder.setSolver(_solver);
    }
    inline const Projector2D* projector() const { return _projector; }
    inline void setProjector(Projector2D* projector) { 
      _projector = projector;
      _correspondence_finder.setProjector(_projector);
    }
    inline const CorrespondenceFinder2D* correspondenceFinder() const { return &_correspondence_finder; }
  private:
    int _iterations;
    float _bpr;
    Eigen::Isometry2f _global_t;
    Cloud2D *_reference, *_current;
    Solver2D *_solver;
    Projector2D *_projector;
    CorrespondenceFinder2D _correspondence_finder;
    CloudProcessor _cloud_processor;
  };
}
