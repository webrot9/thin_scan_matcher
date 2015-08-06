#pragma once

#include "defs.h"
#include "rich_point2d.h"

namespace tsm {
  typedef std::vector<RichPoint2D, Eigen::aligned_allocator<RichPoint2D> > RichPoint2DVector;

  /*
     This class represents a 3D model, as a collection of rich points
  */
  struct Cloud2D: public RichPoint2DVector {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    struct IndexPair {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      IndexPair() {
	x = y = 0;
	index = -1;
      }
      
      // Resolution is in cm
      IndexPair(const Eigen::Vector2f& v, int idx, float ires) {
	x = static_cast<int>(ires * v.x() * 100);
	y = static_cast<int>(ires * v.y() * 100);
	index = idx;
      }

      inline bool operator<(const IndexPair& o) const {
	if (x < o.x)
	  return true;
	if (x > o.x)
	  return false;
	if (y < o.y)
	  return true;
	if (y > o.y)
	  return false;
	if (index < o.index)
	  return true;

	return false;
      }

      inline bool sameCell(const IndexPair& o) const {
	return x == o.x && y == o.y;
      }

      int x, y, index;
    };

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
    void clip(float range, const Eigen::Isometry2f& pose = Eigen::Isometry2f::Identity());

    //! prunes the points in model, computing a scaled average
    //! one point will survive for each voxel of side res. Res is in cm
    void voxelize(Cloud2D& model, float res);  

    // //! draws a cloud, by applying the provided transformation, with normals and origin pose
    // void draw(RGBImage& img, cv::Vec3b color, bool draw_normals = false,
    // 	      Eigen::Isometry2f T = Eigen::Isometry2f::Identity(),
    // 	      bool draw_pose_origin = false, float scale = 20) const;

    void draw(bool draw_normals = false,
	      Eigen::Isometry2f T = Eigen::Isometry2f::Identity(),
	      bool draw_pose_origin = false,
	      bool use_fans=false) const;

    virtual ~Cloud2D();
  };
}
