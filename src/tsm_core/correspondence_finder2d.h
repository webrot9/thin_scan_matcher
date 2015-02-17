#pragma once

#include "projector2d.h"
#include "solver2d.h"

namespace PSolver {
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
    inline const std::vector<int>& correspondences() const { return _correspondences; }
    inline const IntVector& indicesCurrent() const { return _indicesCurrent; }
    inline const IntVector& indicesReference() const { return _indicesReference; }
    void drawCorrespondences(UnsignedCharImage& img);

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
    IntVector _indicesCurrent, _indicesReference;
    FloatVector _projectedCurrentRanges, _projectedReferenceRanges;

    std::vector<int> _correspondences;
  };
}
