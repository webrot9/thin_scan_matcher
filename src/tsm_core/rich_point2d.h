#pragma once

#include "defs.h"

namespace tsm {
  struct RichPoint2D {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    inline RichPoint2D(const Eigen::Vector2f& p = Eigen::Vector2f::Zero(),
		       const Eigen::Vector2f& n=Eigen::Vector2f::Zero(),
		       float a=0.0f) {
      _point = p;
      _normal = n; 
      _accumulator = a;
      _is_normalized = true;
      _color = Eigen::Vector3f(0,0,0);
    }

    inline RichPoint2D& operator+=(const RichPoint2D& rp) {
      denormalize();
      RichPoint2D rp2(rp);
      rp2.denormalize();
      _point += rp2._point;
      _normal += rp2._normal;
      _accumulator += rp2._accumulator;
      return *this;
    }
    
    inline bool isNormalized() const { return _is_normalized; }
    inline const Eigen::Vector2f& point() const { return _point; }
    inline const Eigen::Vector2f& normal() const { return _normal; }
    inline float accumulator() const { return _accumulator; }
    inline void transformInPlace(Eigen::Isometry2f iso) {
      _point = iso * _point;
      _normal = iso.linear() * _normal;
    }
    inline RichPoint2D transform(const Eigen::Isometry2f& iso) const {
      return RichPoint2D(iso * _point, iso.linear() * _normal, _accumulator);
    }

    inline const Eigen::Vector3f& color() const {return _color;}

    inline void setColor(const Eigen::Vector3f& color_) {_color=color_;}

    inline void denormalize() {
      if (!_is_normalized) {
	return;
      }

      _point *= _accumulator;
      _normal *= _accumulator;
      _is_normalized = false;
    }

    inline void normalize() {
      if (_is_normalized) {
	return;
      }

      if (_accumulator > 0) {
	float iv = 1. / _accumulator;
	_point *= iv;
	_normal *= iv;
	_normal.normalize();
      } else {
	_point.setZero();
	_normal.setZero();
      }

      _is_normalized = true;
    }

    Eigen::Vector2f _point;
    Eigen::Vector2f _normal;
    float _accumulator;
    bool _is_normalized;
    Eigen::Vector3f _color;
  };
}
