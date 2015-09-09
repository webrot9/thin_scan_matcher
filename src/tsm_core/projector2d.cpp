#include "projector2d.h"

#include <Eigen/Eigenvalues> 
using namespace std;

namespace tsm {
  Projector2D::Projector2D() {
    _sct = 0;
    _max_range = 30;
    _min_range = 0.1;
    _num_ranges = 1024;
    setFov(1.5 * M_PI);
    updateParameters();
  }
  
  void Projector2D::updateParameters() {
    _angle_increment = _fov / _num_ranges;

    if (_sct == 0)
      _sct = new SinCosTable(-_fov * 0.5, _angle_increment, _num_ranges);
    else
      *_sct = SinCosTable(-_fov * 0.5, _angle_increment, _num_ranges);
  }

  Projector2D::~Projector2D() {
    if (_sct) 
      delete _sct;
  }

  void Projector2D::project(FloatVector& ranges, IntVector& indices,
			    const Eigen::Isometry2f& T,const Cloud2D& model) const {
    float middle = _num_ranges * 0.5;
    float inverse_angle_increment = 1. / _angle_increment;
    const Eigen::Matrix2f& R = T.linear();
    ranges.resize(_num_ranges);
    indices.resize(_num_ranges);
    std::fill(ranges.begin(), ranges.end(), 1e3);
    std::fill(indices.begin(), indices.end(), -1);

    for (size_t i = 0; i < model.size(); ++i){   
      const RichPoint2D& rp = model[i];
      Eigen::Vector2f p = T * rp.point();
      Eigen::Vector2f n = R * rp.normal();

      float angle = atan2(p.y(), p.x());
      int bin = (int)(angle * inverse_angle_increment + middle);

      if (bin < 0 || bin >= _num_ranges)
	continue;

      float range = p.norm();
      float& brange = ranges[bin];

      if (brange > range)
	brange = range;
      else 
	continue;

      indices[bin] = (p.dot(n) <= 0) ? i : -1;
    }
  }

  void Projector2D::unproject(Cloud2D& model, const UnsignedShortImage& depth_image,
			      const Eigen::Matrix3f camera_matrix, const Eigen::Isometry3f T) {
    FloatVector ranges, final_ranges;
    IntVector accumulators;
    std::vector<FloatVector> ranges_to_merge;
    Eigen::Matrix3f camera_matrix_inv = camera_matrix.inverse();
    ranges.resize(depth_image.cols);
    final_ranges.resize(depth_image.cols);
    accumulators.resize(depth_image.cols);
    ranges_to_merge.resize(depth_image.rows);
    std::fill(ranges.begin(), ranges.end(), 0);
    std::fill(final_ranges.begin(), final_ranges.end(), 0);
    std::fill(accumulators.begin(), accumulators.end(), 0);

    Eigen::Vector3f real_pt;
    Eigen::Vector3f im_pt;
    int index = 0;

    for (size_t r = 0; r < depth_image.rows; ++r) {
      for (size_t c = 0; c < depth_image.cols; ++c) {
	float z = depth_image.at<unsigned short>(r, c) * 0.001;

	im_pt = Eigen::Vector3f(c, r, z);
	real_pt = T.linear() * camera_matrix_inv * im_pt;

	float range = sqrt(real_pt.x() * real_pt.x() + real_pt.z() * real_pt.z());

	if (real_pt.z() != real_pt.z() || real_pt.z() <= _min_range || real_pt.z() >= _max_range)
	  continue;

	ranges[c] = range;
      }

      ranges_to_merge[r] = ranges;
    }

    for (size_t i = 0; i < depth_image.rows; ++i) {
      for(size_t j = 0; j < depth_image.cols; ++j) {
	float& final = final_ranges[j];
	float& actual = ranges_to_merge[i][j];
	float distance = (final - actual);

	// if a new point appears behind an old replace the old
	if (distance < -0.4) {
	  final = actual;
	 
	  if(accumulators[j] == 0)
	    ++accumulators[j];

	  continue;
	}

	final += actual;
	++accumulators[j];
      }
    }

    for (size_t i = 0; i < depth_image.cols; ++i) {
      if (accumulators[i] != 0)
	final_ranges[i] /= accumulators[i];
    }
    
    unproject(model, final_ranges);
  }

  void Projector2D::unproject(Cloud2D& model, const FloatVector& ranges) {
    float dist = 0.02;
    int minPoints = 3;
    Eigen::Vector2f mean;
    Eigen::Matrix2f cov;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigenSolver;

    model.resize(ranges.size());

    int k = 0;
    for (size_t i = 0; i < ranges.size(); ++i){
      float r = ranges[i];

      if (r != r || r <= _min_range || r >= _max_range)
	continue;

      model[k]._point = _sct->sincos(i) * r;
      model[k]._accumulator = 1.f / r;
      k++;
    }

    model.resize(k);
    
    // extract the normals, naive way
    for (size_t i = 0; i < model.size(); ++i) {
      // for each point go left and right, until the distance
      //from the current point exceeds a threshold, and accumulate the points;
      Eigen::Vector2f p0 = model[i].point();
      Eigen::Vector2f p1 = model[i].point();
      int imin = i, imax = i;

      while (imin > 0){
	if ((p1 - p0).squaredNorm() >= dist) {
	  ++imin;
	  break;
	}

	--imin;
	p1 = model[imin].point();
      }

      p1 = model[i].point();
      while (imax < static_cast<int>(model.size())){
	if((p1 - p0).squaredNorm() >= dist){
	  break;
	}

	++imax;
	p1 = model[imax].point();
      }

      model[i]._normal.setZero();

      if (imax - imin < minPoints) {
	model[i]._normal.setZero();
	continue;
      }
      mean.setZero();
      cov.setZero();

      float scale = 1. / (imax - imin);

      for (int j = imin; j < imax; ++j) {
	mean += model[j].point();
	cov += model[j].point() * model[j].point().transpose();
      }

      mean *= scale;
      cov *= scale;
      cov -= mean * mean.transpose();

      eigenSolver.computeDirect(cov, Eigen::ComputeEigenvectors);
 
      if (eigenSolver.eigenvalues()(0) / eigenSolver.eigenvalues()(1) > 1e-1) {
	continue;
      }
      Eigen::Vector2f n = eigenSolver.eigenvectors().col(0);

      if (p0.dot(n) > 0)
	n = -n;

      model[i]._normal = n;
    }
  }
}
