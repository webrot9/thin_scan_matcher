#pragma once

#include <omp.h>
#include <iostream>
#include <cassert>
#include <stdexcept>

#include "cloud2d.h"

namespace tsm {
  struct Solver2D{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // initializes the solver
    Solver2D();

    void errorAndJacobian(Eigen::Vector2f&  pointError, 
			  Eigen::Vector2f&  normalError,
			  Matrix4_3f&  J, 
			  const Eigen::Vector2f& referencePoint, 
			  const Eigen::Vector2f& referenceNormal, 
			  const Eigen::Vector2f& currentPoint,
			  const Eigen::Vector2f& currentNormal);

    // computes the hessian and the coefficient vector of the reference points between imin and imax
    // it also computes the error and the number of inliers
    virtual void linearize(Eigen::Matrix3f& H,Eigen::Vector3f& b, 
			   float& error, float& inlierError, int& inliers, 
			   const std::vector<int>& correspondences,
			   const IntVector& indicesCurrent, const IntVector& indicesReference,
			   size_t imin=0, size_t imax=std::numeric_limits<size_t>::max());    

    // does one round of optimization
    // updates the transform T, the error and the numver of inliers
    void oneRound(const std::vector<int>& correspondences, const IntVector& indicesCurrent, const IntVector& indicesReference);

    // computesTheOmegas
    void computeOmegas();
    
    virtual ~Solver2D();
    Cloud2D* reference, *current;
    Matrix2fVector omegaPoints, omegaNormals;
    std::vector<bool> inliersVector;
    Eigen::Isometry2f T; // position of the world w.r.t the camera
    float error;
    float maxError;
    Eigen::Matrix2f flatOmega;
    Eigen::Matrix2f longLinearOmega;
    float damping;
    int inliers;
    float inliersError;
  };
}


