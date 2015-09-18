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

    //! draws a cloud, by applying the provided transformation, with normals and origin pose
    void draw(RGBImage& img, cv::Vec3b color, bool draw_normals = false,
    	      Eigen::Isometry2f T = Eigen::Isometry2f::Identity(),
    	      bool draw_pose_origin = false, float scale = 20) const;

    void draw(bool draw_normals = false,
	      Eigen::Isometry2f T = Eigen::Isometry2f::Identity(),
	      bool draw_pose_origin = false,
	      bool use_fans=false) const;

    virtual ~Cloud2D();
  };
}
