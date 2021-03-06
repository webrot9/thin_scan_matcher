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
    void update(Cloud2D* cloud=0, const Eigen::Isometry2f& initial_guess=Eigen::Isometry2f::Identity());
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
    void setSolver(Solver2D* solver);
    inline const Projector2D* projector() const { return _projector; }
    void setProjector(Projector2D* projector);
    inline const CorrespondenceFinder2D* correspondenceFinder() const { return &_correspondence_finder; }
    inline void setInlierDistance(float inlier_distance){_inlier_distance=inlier_distance;}
    inline float inlierDistance()const {return _inlier_distance;}

    inline void setMinCorrespondencesRatio(float ratio){ _min_correspondences_ratio = ratio; }
    inline float minCorrespondencesRatio() const { return _min_correspondences_ratio; }

    inline float localMapClippingRange() const { return _local_map_clipping_range; }
    inline float clipTranslationThreshold() const { return _clip_translation_threshold; }

    inline void setLocalMapClippingRange(float range) { _local_map_clipping_range = range; }
    inline void setClipTranslationThreshold(float t) { _clip_translation_threshold = t; }
    
    inline void setVoxelizeResolution(float res) { _voxelize_resolution = res; }
    inline float voxelizeResolution() const { return _voxelize_resolution;}
    inline float cycleTime() const {return _cycle_time;}

  private:
    int _iterations;
    float _bpr;
    float _inlier_distance;
    float _min_correspondences_ratio;
    float _cycle_time;    

    Eigen::Isometry2f _global_t;
    Eigen::Isometry2f _last_clipped_pose;
    float _clip_translation_threshold;
    float _local_map_clipping_range;

    Cloud2D *_reference, *_current;
    Solver2D *_solver;
    Projector2D *_projector;
    CorrespondenceFinder2D _correspondence_finder;
    float _voxelize_resolution;
  };
}
