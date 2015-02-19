#include "solver2d.h"

//#define _GO_PARALLEL_
//#define _FAST_MULT_

namespace tsm {
  Solver2D::Solver2D(){
    T.setIdentity();
    maxError = 10;
    flatOmega << 
      100, 0, 
      0, 1e-3;

    longLinearOmega << 
      1, 0,
      0, 1;

    error = 0;
    inliers = 0;
    damping = 0;
  }

  void Solver2D::errorAndJacobian(Eigen::Vector2f&  pointError, 
				       Eigen::Vector2f&  normalError,
				       Matrix4_3f&  J, 
				       const Eigen::Vector2f& referencePoint, 
				       const Eigen::Vector2f& referenceNormal, 
				       const Eigen::Vector2f& currentPoint,
				       const Eigen::Vector2f& currentNormal){
    J.setZero();
    // apply the transform to the point
    Eigen::Vector2f tp = T*currentPoint;
    Eigen::Vector2f tn = T.linear()*currentNormal;

    pointError = tp - referencePoint;
    normalError = tn - referenceNormal;

    // jacobian of the transform part = [     I(2x2)       0     0
    //                                    -tp.y() tp.x() -tn.y() tn.x()].transpose()
    J.block<2,2>(0,0).setIdentity();  
    J.block<4,1>(0,2) << -tp.y(), tp.x(), -tn.y(), tn.x();
  }

  void Solver2D::computeOmegas() {
    size_t referenceSize = reference->size();
    size_t currentSize = current->size();
    omegaPoints.resize(referenceSize);
    omegaNormals.resize(referenceSize);

#ifdef _GO_PARALLEL_
#pragma omp parallel for
#endif
    for (size_t i = 0; i < referenceSize; ++i) {
      const Eigen::Vector2f& referencePoint = (*current)[i].point();
      const Eigen::Vector2f& referenceNormal = (*reference)[i].normal();
      
      Eigen::Matrix2f& omegap = omegaPoints[i];
      Eigen::Matrix2f& omegan = omegaNormals[i];
      
      omegap.setIdentity();

      // if the point has a normal
      if (referenceNormal.squaredNorm()>0) {
	Eigen::Matrix2f Rn;
	Rn << 
	  referenceNormal.x(), -referenceNormal.y(), 
	  referenceNormal.y(), referenceNormal.x();
	omegap += Rn * flatOmega * Rn.transpose();
	omegan = Rn * longLinearOmega * Rn.transpose();
      } else {
	omegan.setZero();
      }
    }
  }

  void Solver2D::linearize(Eigen::Matrix3f& H, Eigen::Vector3f& b, 
				float& myerror, float& inliersMyerror, int& inliers,
				const std::vector<int>& correspondences,
				const IntVector& indicesCurrent, const IntVector& indicesReference,
			      	size_t imin, size_t imax){
    imax = imax > correspondences.size() ? correspondences.size() : imax;
    H.setZero();
    b.setZero();
    inliersError = 0;
    inliers = 0;
    Eigen::Vector2f ep;
    Eigen::Vector2f en;
 
    Eigen::Vector4f e;
    Matrix4_3f J;
    Eigen::Matrix4f Omega;
    Omega.setZero();

    for (size_t i = imin; i < imax; ++i) {
      int current_idx = indicesCurrent[correspondences[i]];
      int reference_idx = indicesReference[correspondences[i]];

      const Eigen::Vector2f& currentPoint = (*current)[current_idx].point();
      const Eigen::Vector2f& currentNormal = (*current)[current_idx].normal();

      const Eigen::Vector2f& referencePoint = (*reference)[reference_idx].point();
      const Eigen::Vector2f& referenceNormal = (*reference)[reference_idx].normal();

      const Eigen::Matrix2f& omegap = omegaPoints[reference_idx];
      const Eigen::Matrix2f& omegan = omegaNormals[reference_idx];

      errorAndJacobian(ep, en, J, 
		       referencePoint, referenceNormal, 
		       currentPoint, currentNormal);

      e.block<2,1>(0,0) = ep;
      e.block<2,1>(2,0) = en;
    
      Omega.block<2,2>(0,0) = omegap;
      Omega.block<2,2>(2,2) = omegan;

      float scale = 1;
      float chi = e.transpose() * Omega * e;
      error = e.transpose() * e;
      if (chi > maxError) {
	scale = maxError / chi;
	inliersVector[i] = false;
      } else {
	inliers++;
	inliersVector[i] = true;
	inliersError += ep.squaredNorm();
      }

      Omega *= scale;
#ifdef NAN_CHECK
      Matrix3f Hi=J.transpose()*Omega*J;
      Vector3f bi=J.transpose()*Omega*e;
      if (isNan(Hi)) {
	cerr << endl;
	cerr << Hi;
	cerr << "J" << endl << J << endl;
	cerr << "Omega" << endl << Omega << endl;
	cerr << "e" << endl << e << endl;
	throw std::runtime_error("NAN detected");
      }
#endif

      H.noalias() += J.transpose()*Omega*J;
      b.noalias() += J.transpose()*Omega*e;
    }
  }
    
  void Solver2D::oneRound(const std::vector<int>& correspondences, const IntVector& indicesCurrent, const IntVector& indicesReference){
    inliersVector.resize(reference->size(), false);
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    Eigen::Vector3f b = Eigen::Vector3f::Zero();
    error = 0;
    inliers = 0;
    inliersError = 0;
    size_t numPoints = correspondences.size();
#ifdef _GO_PARALLEL_
    int numThreads = omp_get_max_threads();
    int iterationsPerThread = numPoints / numThreads;

#pragma omp parallel num_threads(numThreads)
    {
      int threadId = omp_get_thread_num();
      int imin = iterationsPerThread * threadId;
      int imax = imin + iterationsPerThread;

      Eigen::Matrix3f tH; 
      Eigen::Vector3f tb; 
      float terror = 0;
      float tinliersError = 0;
      int tinliers = 0;
      linearize(
		tH,
		tb,
		terror,
		tinliersError,
		tinliers,
		correspondences,
		indicesCurrent,
		indicesReference,
		imin,
		imax
		);

#pragma omp critical 
      {
	H += tH;
	b += tb;
	inliers += tinliers;
	inliersError += tinliersError;
	error += terror;
      }
    }
#else
    linearize(
	      H,
	      b,
	      error,
	      inliersError,
	      inliers,
	      correspondences,
	      indicesCurrent,
	      indicesReference,
	      0,
	      numPoints
	      );
#endif

    H += damping*Eigen::Matrix3f::Identity();

    Eigen::Vector3f dt = H.ldlt().solve(-b);
    T = v2t(dt)*T;
    Eigen::Matrix2f R = T.linear();
    Eigen::Matrix2f E = R.transpose() * R;
    E.diagonal().array() -= 1;
    T.linear() -= 0.5 * R * E;
  }

  Solver2D::~Solver2D(){}
}
