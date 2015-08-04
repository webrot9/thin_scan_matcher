#pragma once

#include "projector2d.h"
#include "solver2d.h"

namespace tsm {
  class CorrespondenceFinder2D {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    CorrespondenceFinder2D(Projector2D* projector = NULL, Solver2D* solver = NULL);
    ~CorrespondenceFinder2D() {};
    void compute();
    void init();
    inline const std::vector<int>& correspondences() const { return _correspondences; }
    inline const IntVector& indicesCurrent() const { return _indices_current; }
    inline const IntVector& indicesReference() const { return _indices_reference; }
    void drawCorrespondences(RGBImage& img, Eigen::Isometry2f T, float scale=20) const;

    // setter & getter
    inline void setProjector(const Projector2D* projector) { 
      _projector = projector;
    }
    inline void setSolver(const Solver2D* solver) {
      _solver = solver;
    }
    inline float minNormalCos() const {return _min_normal_cos;}
    inline void setMinNormalCos(float mnc) {_min_normal_cos=mnc;}
    inline float maxSquaredDistance() const {return _max_squared_distance;}
    inline void setMaxSquaredDistance(float msd) {_max_squared_distance=msd;}

  private:
    const Projector2D* _projector;
    const Solver2D* _solver;
    IntVector _indices_current, _indices_reference;
    FloatVector _projected_current_ranges, _projected_reference_ranges;

    float _max_squared_distance;
    float _min_normal_cos;
    std::vector<int> _correspondences;
  };
}
