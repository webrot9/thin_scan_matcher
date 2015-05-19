#pragma once

#include <iostream>
#include <stdexcept>

#include "cloud2d.h"

namespace tsm {
  class Solver2D{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Solver2D();
    virtual ~Solver2D() {}
    void errorAndJacobian(const Eigen::Vector2f& reference_pt, const Eigen::Vector2f& reference_normal,
			  const Eigen::Vector2f& current_pt, const Eigen::Vector2f& current_normal,
			  Eigen::Vector2f& pt_error, Eigen::Vector2f& normal_error, Matrix4_3f& J);

    // updates and computes the omegas
    // if no argument is given it updates all omegas, otherwise 
    // it updates the omegas of the passed vector of indices
    void computeOmegas(const std::vector<int> update_list = std::vector<int>());

    //! gives a hint to the system on which omegas need to be recomputed
    //! needs to be called after setting the reference model
    void setReferencePointsHint(const std::vector<int>& reference_points_hint);

    // computes the Hessian and the coefficient vector of the reference points
    // between imin and imax; it also computes the error and the number of inliers
    virtual void linearize(const std::vector<int>& correspondences, const IntVector& indices_current, 
			   const IntVector& indices_reference, Eigen::Matrix3f& H, Eigen::Vector3f& b,
			   float& chi, int& inliers, float inliers_error,
			   size_t imin = 0, size_t imax = std::numeric_limits<size_t>::max());    

    // does one round of optimization: updates the transform T,
    // the error and the number of inliers
    void optimize(const std::vector<int>& correspondences, 
		  const IntVector& indices_current, const IntVector& indices_reference);

    // setter & getter
    inline const Eigen::Isometry2f T() const { return _T; }
    inline void setT(Eigen::Isometry2f T) { _T = T; }
    inline const Eigen::Matrix2f flatOmega() const { return _flat_omega; }
    inline void setFlatOmega(Eigen::Matrix2f flat_omega) { _flat_omega = flat_omega; }
    inline const Eigen::Matrix2f longLinearOmega() const { return _long_linear_omega; }
    inline void setLongLinearOmega(Eigen::Matrix2f long_linear_omega) { _long_linear_omega = long_linear_omega; }
    inline const Cloud2D* reference() const { return _reference; }
    inline void setReference(Cloud2D* reference) { _reference = reference; }
    inline const Cloud2D* current() const { return _current; }
    inline void setCurrent(Cloud2D* current) { _current = current; }
    inline float chi() const { return _chi; }
    inline int inliers() const { return _inliers; }
    inline float inliersError() const { return _inliers_error; }

  private:
    std::vector<bool> _inliers_vector;
    Eigen::Isometry2f _T; // position of the world w.r.t the sensor
    Eigen::Matrix2f _flat_omega, _long_linear_omega;
    Cloud2D *_reference, *_current;
    Matrix2fVector _omega_points, _omega_normals;
    float _chi;
    float _max_error;
    float _damping;
    int _inliers;
    float _inliers_error;
  };
}


