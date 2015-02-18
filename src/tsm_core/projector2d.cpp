#include "projector2d.h"

#include <Eigen/Eigenvalues> 

namespace tsm {

  struct SinCosTable{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    SinCosTable(float angle_min, float angle_increment, size_t s){
      init(angle_min, angle_increment, s);
    }

    void init(float angle_min, float angle_increment, size_t s) {
      if (_angle_min == angle_min && 
	  _angle_increment == angle_increment &&
	  _table.size() == s)
	return;
      _angle_min = angle_min;
      _angle_increment = angle_increment;
      _table.resize(s);
      for (size_t i = 0; i<s; i++){
	float alpha = _angle_min+i*_angle_increment;
	_table[i] = Eigen::Vector2f(cos(alpha),sin(alpha));
      }
    }
    inline const Eigen::Vector2f& sincos(int i) const {
      return _table[i];
    }
    float _angle_min;
    float _angle_increment;
    Vector2fVector _table;
  };

  Projector2D::Projector2D() {
    _sct = NULL;
    _max_range = 30;
    _min_range=0.1;
    _num_ranges=1024;
    setFov(1.5*M_PI);
    updateParameters();
  }
  
  void Projector2D::updateParameters(){
    _angle_increment = _fov/_num_ranges;

    if (_sct == NULL)
      _sct = new SinCosTable(-_fov/2, _angle_increment, _num_ranges);
    else
      *_sct = SinCosTable(-_fov/2, _angle_increment, _num_ranges);
  }

  Projector2D::~Projector2D(){
    if (_sct)
      delete _sct;
  }

  void Projector2D::project(FloatVector& zbuffer, IntVector& indices,
			    const Eigen::Isometry2f& T,
			    const Cloud2D& model) const {
    const Eigen::Matrix2f& R = T.linear();
    zbuffer.resize(_num_ranges);
    indices.resize(_num_ranges);
    std::fill(indices.begin(), indices.end(), -1);
    std::fill(zbuffer.begin(), zbuffer.end(), 1e3);
    float middle = _num_ranges/2;
    float inverse_angle_increment = 1./_angle_increment;
    for (size_t i = 0; i<model.size(); i++){   
      const RichPoint2D& rp = model[i];
      Eigen::Vector2f p = T*rp.point();
      Eigen::Vector2f n = R*rp.normal();
      float angle = atan2(p.y(), p.x());
      int bin = angle * inverse_angle_increment + middle;
      if (bin<0 || bin>=_num_ranges)
	continue;

      float range = p.norm();

      float& brange = zbuffer[bin];

      if (brange > range) {
	brange = range;
      }
      else 
	continue;

      indices[bin] = (p.dot(n) <0) ? i : -1;
    }
  }

  void Projector2D::unproject(Cloud2D& model, const UnsignedShortImage& depth_image,
		 const Eigen::Matrix3f camera_matrix, const Eigen::Isometry3f T) {
    FloatVector ranges;
    IntVector accumulators;
    ranges.resize(depth_image.cols);
    accumulators.resize(depth_image.cols);
    std::fill(ranges.begin(), ranges.end(), 0);
    std::fill(accumulators.begin(), accumulators.end(), 0);
    float r = 0.f;
    Eigen::Vector3f real_point;
    Eigen::Vector3f im_point;

    Eigen::Matrix3f camera_matrix_inv = camera_matrix.inverse();

    for (int r = 0; r < depth_image.rows; ++r) {
      for (int c = 0; c < depth_image.cols; ++c) {
	im_point = Eigen::Vector3f(c, r, depth_image.at<unsigned short>(r, c));
	real_point = T.linear() * camera_matrix_inv * im_point;
	ranges[c] += real_point.x();
	++accumulators[c];
      }
    }

    for (size_t i = 0; i < depth_image.cols; ++i) {
      ranges[i] /= accumulators[i];
    }

    unproject(model, ranges);
  }


  void Projector2D::unproject(Cloud2D& model, const FloatVector& ranges) {
    model.resize(ranges.size());
    int k = 0;

    for (size_t i = 0; i<ranges.size(); i++){
      float r = ranges[i];
      if (r != r || r<_min_range || r>_max_range)
	continue;
      model[k]._point=_sct->sincos(i)*r;
      model[k]._accumulator = 1./r;
      k++;
    }
    model.resize(k);

    float dist = 0.01;
    int minPoints = 3;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigenSolver;

    // extract the normals, naive way
    for (size_t i =0; i<model.size(); i++) {
      // for each point go left and right, until the distance from the current point exceeds a threshold, and accumulate the points;
      Eigen::Vector2f p0 = model[i].point();
      int imin = i, imax = i;
      Eigen::Vector2f p1 = model[i].point();

      while (imin>0){
	if ((p1-p0).squaredNorm()>dist) {
	  imin++;
	  break;
	}
	imin--;
	p1=model[imin].point();
      }

      p1 = model[i].point();
      while (imax<(int)model.size()){
	if((p1-p0).squaredNorm()>dist){
	  break;
	}
	imax++;
	p1=model[imax].point();
      }

      model[i]._normal.setZero();

      if (imax-imin<minPoints)
	continue;

      Eigen::Vector2f mean;
      Eigen::Matrix2f cov;
      mean.setZero();
      cov.setZero();
      float scale = 1./(imax-imin);
      for (int j=imin; j<imax; j++) {
	mean += model[j].point();
	cov += model[j].point()*model[j].point().transpose();
      }
      mean *= scale;
      cov *= scale;
      cov -= mean * mean.transpose();

      eigenSolver.computeDirect(cov, Eigen::ComputeEigenvectors);
      if (eigenSolver.eigenvalues()(0) / eigenSolver.eigenvalues()(1) > 1e-1)
	continue;
      Eigen::Vector2f n = eigenSolver.eigenvectors().col(0);

      if (p0.dot(n) > 0)
	n = -n;

      Eigen::Vector2f t(n.y(), -n.x());
      model[i]._normal = n;
    }
  }
}
