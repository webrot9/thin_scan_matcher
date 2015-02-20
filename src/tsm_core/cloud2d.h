#pragma once
#include "defs.h"
#include <iostream>

namespace tsm{

  struct RichPoint2D {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    inline RichPoint2D(const Eigen::Vector2f& p = Eigen::Vector2f::Zero(), const Eigen::Vector2f& n=Eigen::Vector2f::Zero(), float a=0.0f){
      _point = p;
      _normal = n; 
      _accumulator = a;
      _is_normalized = true;
    }

    inline RichPoint2D& operator+=(const RichPoint2D& rp){
      denormalize();
      RichPoint2D rp2(rp);
      rp2.denormalize();
      _point += rp2._point;
      _normal += rp2._normal;
      _accumulator += rp2._accumulator;
      return *this;
    }
    
    inline bool isNormalized() const {return _is_normalized;}
    inline const Eigen::Vector2f& point() const {return _point;}
    inline const Eigen::Vector2f& normal() const {return _normal;}
    inline float accumulator() const {return _accumulator;}
    inline void transformInPlace(Eigen::Isometry2f iso) {
      _point = iso*_point;
      _normal = iso.linear()*_normal;
    }
    inline RichPoint2D transform(const Eigen::Isometry2f& iso) const {
      return RichPoint2D(iso*_point, iso.linear()*_normal, _accumulator);
    }

    inline void denormalize() {
      if (!_is_normalized) {
	return;
      }
      _point*=_accumulator;
      _normal*=_accumulator;
      _is_normalized = false;
    }

    inline void normalize() {
      if (_is_normalized) {
	return;
      }
      if (_accumulator>0) {
	float iv = 1./_accumulator;
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
  };

  typedef std::vector<RichPoint2D, Eigen::aligned_allocator<RichPoint2D> > RichPoint2DVector;



  /**)
     This class represents a 3D model, as a collection of rich points
   */

  struct Cloud2D: public RichPoint2DVector {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    //! applies the transformation to each entity in the model, doing side effect.
    //! @param dest: output
    //! @param T: the transform

    void transformInPlace(const Eigen::Isometry2f& T);
    //! applies the transformation to each entity in the model, but writes the output in dest.
    //! @param dest: output
    //! @param T: the transform
    void transform(Cloud2D& dest, const Eigen::Isometry2f& T) const;

    //! adds other to this point cloud, doing side effect
    void add(const Cloud2D& other);


    //! clips to a maxRange around a pose
    void clip(float range, const Eigen::Isometry2f& pose=Eigen::Isometry2f::Identity());
    void draw(UnsignedCharImage& img, bool draw_normals = false, Eigen::Isometry2f T = Eigen::Isometry2f::Identity(), bool draw_pose_origin = false);
  };


  //! does the merge of src in dest
  //! it requires the index image of dest and of src, seen from the same point
  //! and also the depth buffers
  //! the clouds should be aligned
  //! points that are closer than distanceThreshold are merged based on the scaling values
  //! if the normals are compatible
  void merge(FloatVector&, IntVector& destIndices, Cloud2D& dest,
	     FloatVector& srcBuffer, IntVector& srcIndices, Cloud2D& src, 
	     float normalThreshold = 1,
	     float distanceThreshold = 0.2);

  //! prunes the points in model, computing a scaled average
  //! one point will survive for each voxel of side res
  void voxelize(Cloud2D& model, float res);
}
