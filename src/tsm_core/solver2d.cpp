#include "solver2d.h"

namespace tsm {
  Solver2D::Solver2D(){
    _T.setIdentity();
    _flat_omega << 
      100, 0, 
      0, 1e-3;

    _long_linear_omega << 
      1, 0,
      0, 1;

    _chi = 0.f;
    _max_error = 10.f;
    _damping = 0.f;
    _inliers = 0;
    _inliers_error = 0.f;
  }

  void Solver2D::errorAndJacobian(const Eigen::Vector2f& reference_pt, const Eigen::Vector2f& reference_normal,
				  const Eigen::Vector2f& current_pt, const Eigen::Vector2f& current_normal,
				  Eigen::Vector2f& pt_error, Eigen::Vector2f& normal_error, Matrix4_3f& J) {
    J.setZero();

    // apply the transform to the point
    Eigen::Vector2f tp = _T * current_pt;
    Eigen::Vector2f tn = _T.linear() * current_normal;

    pt_error = tp - reference_pt;
    normal_error = tn - reference_pt;

    // jacobian of the transform part = [     I(2x2)       0     0
    //                                    -tp.y() tp.x() -tn.y() tn.x()].transpose()
    J.block<2,2>(0,0).setIdentity();  
    J.block<4,1>(0,2) << -tp.y(), tp.x(), -tn.y(), tn.x();
  }

  void Solver2D::computeOmegas(const std::vector<int> update_list) {
    bool compact_compute = false;
    size_t num_omegas = 0;
    size_t reference_size = _reference->size();
    _omega_points.resize(reference_size);
    _omega_normals.resize(reference_size);

    if (update_list.size()) {
      num_omegas = update_list.size();
      compact_compute = true;
    } else {
      num_omegas = _reference->size();
    }

    for (size_t i = 0; i < num_omegas; ++i) {
      int idx = compact_compute ? update_list[i] : i;

      if (idx < 0)
	continue;

      const Eigen::Vector2f& reference_pt = (*_reference)[idx].point();
      const Eigen::Vector2f& reference_normal = (*_reference)[idx].normal();
      
      Eigen::Matrix2f& omega_pt = _omega_points[idx];
      Eigen::Matrix2f& omega_normal = _omega_normals[idx];
     
      omega_pt.setZero();
      omega_normal.setZero();

      // if the point has a normal
      if (reference_normal.squaredNorm() > 0) {
	Eigen::Matrix2f R;

	// Rotate the omega accordingly to the following:
	// cos(angle_normal), -sin(angle_normal)
	// sin(angle_normal), cos(angle_normal)
	R << 
	  reference_normal.x(), -reference_normal.y(), 
	  reference_normal.y(), reference_normal.x();

	omega_pt = R * _flat_omega * R.transpose();
	omega_normal = R * _long_linear_omega * R.transpose();
      }
    }
  }

  void Solver2D::setReferencePointsHint(const std::vector<int>& reference_points_hint) {
    computeOmegas(reference_points_hint);
  }

  void Solver2D::linearize(const std::vector<int>& correspondences, const IntVector& indices_current, 
			   const IntVector& indices_reference, Eigen::Matrix3f& H, Eigen::Vector3f& b,
			   float& chi, int& inliers, float inliers_error,
			   size_t imin, size_t imax) {
    Eigen::Vector2f error_pt;
    Eigen::Vector2f error_normal;
    Eigen::Vector4f error;
    Eigen::Matrix4f omega;
    Matrix4_3f J;

    H.setZero();
    b.setZero();
    omega.setZero();

    chi = 0;
    inliers = 0;
    inliers_error = 0;

    imax = imax > correspondences.size() ? correspondences.size() : imax;

    for (size_t i = imin; i < imax; ++i) {
      int current_idx = indices_current[correspondences[i]];
      int reference_idx = indices_reference[correspondences[i]];

      const Eigen::Vector2f& current_pt = (*_current)[current_idx].point();
      const Eigen::Vector2f& current_normal = (*_current)[current_idx].normal();

      const Eigen::Vector2f& reference_pt = (*_reference)[reference_idx].point();
      const Eigen::Vector2f& reference_normal = (*_reference)[reference_idx].normal();

      const Eigen::Matrix2f& omega_pt = _omega_points[reference_idx];
      const Eigen::Matrix2f& omega_normal = _omega_normals[reference_idx];

      errorAndJacobian(reference_pt, reference_normal, current_pt, current_normal,
		       error_pt, error_normal, J);

      error.block<2,1>(0,0) = error_pt;
      error.block<2,1>(2,0) = error_normal;
    
      omega.block<2,2>(0,0) = omega_pt;
      omega.block<2,2>(2,2) = omega_normal;

      float scale = 1;
      chi = error.transpose() * omega * error;

      if (chi > _max_error) {
	scale = _max_error / chi;
	_inliers_vector[i] = false;
      } else {
	++inliers;
	_inliers_vector[i] = true;
	inliers_error += error_pt.squaredNorm();
      }

      omega *= scale;

      Eigen::Matrix3f Hi = J.transpose() * omega * J;
      Eigen::Vector3f bi = J.transpose() * omega * error;

      if (isNan(Hi) || isNan(bi)) {
	throw std::runtime_error("NAN detected");
      }

      H.noalias() += J.transpose() * omega * J;
      b.noalias() += J.transpose() * omega * error;
    }
  }
    
  void Solver2D::optimize(const std::vector<int>& correspondences, 
			  const IntVector& indices_current, const IntVector& indices_reference) {
    size_t num_points = correspondences.size();
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    Eigen::Vector3f b = Eigen::Vector3f::Zero();

    _inliers_vector.resize(_reference->size(), false);
    _chi = 0;
    _inliers = 0;
    _inliers_error = 0;

#ifdef _GO_PARALLEL_
    int num_threads = omp_get_max_threads();
    int iterations_per_thread = num_points / num_threads;

#pragma omp parallel num_threads(num_threads)
    {
      int thread_id = omp_get_thread_num();
      int imin = iterations_per_thread * thread_id;
      int imax = imin + iterations_per_thread;

      Eigen::Matrix3f tH; 
      Eigen::Vector3f tb; 
      float tchi = 0;
      float tinliers_error = 0;
      int tinliers = 0;
      linearize(correspondences, indices_current, indices_reference,
		tH, tb, tchi, tinliers, tinliers_error,	imin, imax);

#pragma omp critical 
      {
	H += tH;
	b += tb;
	_inliers += tinliers;
	_inliers_error += tinliers_error;
	_chi += tchi;
      }
    }
#else
    linearize(correspondences, indices_current, indices_reference,
	      H, b, _chi, _inliers, _inliers_error, 0, num_points);
#endif

    H += _damping * Eigen::Matrix3f::Identity();

    Eigen::Vector3f dt = H.ldlt().solve(-b);
    _T = v2t(dt) * _T;

    // Normalization of the rotation matrix
    Eigen::Matrix2f R = _T.linear();
    Eigen::Matrix2f E = R.transpose() * R;
    E.diagonal().array() -= 1;
    _T.linear() -= 0.5 * R * E;
  }
}
