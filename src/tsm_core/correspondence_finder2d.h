#pragma once

#include "projector2d.h"
#include "solver2d.h"

namespace tsm {
  class CorrespondenceFinder2D {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    CorrespondenceFinder2D(Projector2D* projector = NULL, Solver2D* solver = NULL) {
      _correspondences.clear();
      _projector = projector;
      _solver = solver;
    }
    ~CorrespondenceFinder2D() {};
    void compute();
    void init();
    inline const std::vector<int>& correspondences() const { return _correspondences; }
    inline const IntVector& indicesCurrent() const { return _indices_current; }
    inline const IntVector& indicesReference() const { return _indices_reference; }
    void drawCorrespondences(UnsignedCharImage& img) const;

    // setter & getter
    inline void setProjector(const Projector2D* projector) { 
      _projector = projector;
    }
    inline void setSolver(const Solver2D* solver) {
      _solver = solver;
    }
  private:
    const Projector2D* _projector;
    const Solver2D* _solver;
    IntVector _indices_current, _indices_reference;
    FloatVector _projected_current_ranges, _projected_reference_ranges;

    std::vector<int> _correspondences;
  };
}
