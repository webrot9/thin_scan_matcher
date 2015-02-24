#pragma once
#include "cloud2d.h"

namespace tsm {
  struct SinCosTable;

  class Projector2D {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Projector2D();
    ~Projector2D();

    void project(FloatVector& zbuffer, IntVector& indices,
		 const Eigen::Isometry2f& T, const Cloud2D& model) const;
    void unproject(Cloud2D& model, const FloatVector& ranges);
    void unproject(Cloud2D& model, const UnsignedShortImage& depth_image,
		   const Eigen::Matrix3f camera_matrix, const Eigen::Isometry3f T);

    // setter e getter
    inline void setMaxRange(float max_range) { _max_range = max_range; }
    inline float maxRange() const { return _max_range; }
    inline void setMinRange(float min_range) { _min_range = min_range; }
    inline float minRange() const { return _min_range; }
    inline float angleIncrement() const { return _angle_increment; }
    inline void setFov(float fov_) {
      _fov = fov_;
      updateParameters();
    }
    inline float fov() const { return _fov; }
    inline void setNumRanges(int num_ranges) {
      _num_ranges = num_ranges;
      updateParameters();
    }
    inline int numRanges() const { return _num_ranges; }

  protected:
    struct SinCosTable{
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      SinCosTable(float angle_min, float angle_increment, size_t s){
	init(angle_min, angle_increment, s);
      }

      void init(float angle_min, float angle_increment, size_t s) {
	if (_angle_min == angle_min && _angle_increment == angle_increment && _table.size() == s)
	  return;

	_angle_min = angle_min;
	_angle_increment = angle_increment;
	_table.resize(s);

	for (size_t i = 0; i < s; ++i){
	  float alpha = _angle_min + i * _angle_increment;
	  _table[i] = Eigen::Vector2f(cos(alpha), sin(alpha));
	}
      }

      inline const Eigen::Vector2f& sincos(int i) const {
	return _table[i];
      }

      float _angle_min;
      float _angle_increment;
      Vector2fVector _table;
    };

    void updateParameters();
    float _max_range, _min_range;
    int _num_ranges;
    float _fov;
    float _angle_increment;
    SinCosTable* _sct;
  };

}
