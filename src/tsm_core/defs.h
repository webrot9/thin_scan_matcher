#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <Eigen/StdVector>
#include <iostream>
#include <unistd.h>
#include "opencv2/opencv.hpp"

namespace tsm {

  typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
  typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
  typedef std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > Matrix3fVector;
  typedef std::vector<Eigen::Matrix2f, Eigen::aligned_allocator<Eigen::Matrix2f> > Matrix2fVector;


  typedef Eigen::Matrix<float, 4, 3> Matrix4_3f;
  typedef Eigen::Matrix<float, 2, 3> Matrix2_3f;
  typedef Eigen::Matrix<float, 3, 3> Matrix3_3f;

  typedef Eigen::Matrix<float, 6, 6> Matrix6f;
  typedef Eigen::Matrix<float, 6, 1> Vector6f;

  typedef Eigen::Matrix<float, 9, 6> Matrix9_6f;
  typedef Eigen::Matrix<float, 9, 9> Matrix9f;
  typedef Eigen::Matrix<float, 9, 1> Vector9f;

  typedef Eigen::Matrix<float, 6, 3> Matrix6_3f;


  template <class T> 
  bool isNan(const T& m){
    for (int i=0; i< m.rows(); i++) {
      for (int j=0; j< m.cols(); j++) {
	float v = m(i,j);
	if ( isnan( v ) )
	  return true;
      }
    }
    return false;
  }


  inline Eigen::Isometry3f v2t(const Vector6f& t){
    Eigen::Isometry3f T;
    T.setIdentity();
    T.translation()=t.head<3>();
    float w=t.block<3,1>(3,0).squaredNorm();
    if (w<1) {
      w=sqrt(1-w);
      T.linear()=Eigen::Quaternionf(w, t(3), t(4), t(5)).toRotationMatrix();
    } else {
      T.linear().setIdentity();
    }
    return T;
  }

  inline Vector6f t2v(const Eigen::Isometry3f& t){
    Vector6f v;
    v.head<3>()=t.translation();
    Eigen::Quaternionf q(t.linear());
    v.block<3,1>(3,0)=q.matrix().block<3,1>(1,0);
    if (q.w()<0)
      v.block<3,1>(3,0) *= -1.0f;
    return v;
  }

  inline Eigen::Isometry2f v2t(const Eigen::Vector3f& t){
    Eigen::Isometry2f T;
    T.setIdentity();
    T.translation()=t.head<2>();
    float c = cos(t(2));
    float s = sin(t(2));
    T.linear() << c, -s, s, c;
    return T;
  }

  inline Eigen::Vector3f t2v(const Eigen::Isometry2f& t){
    Eigen::Vector3f v;
    v.head<2>()=t.translation();
    v(2) = atan2(t.linear()(1,0), t.linear()(0,0));
    return v;
  }


  inline Eigen::Matrix3f skew(const Eigen::Vector3f& p){
    Eigen::Matrix3f s;
    s << 
      0,  -p.z(), p.y(),
      p.z(), 0,  -p.x(), 
      -p.y(), p.x(), 0;
    return s;
  }

  inline Eigen::Matrix2f skew(const Eigen::Vector2f& p){
    Eigen::Matrix2f s;
    s << 
      0,  -p.y(),
      p.x(), 0;
    return s;
  }


  /** \typedef UnsignedCharImage
   * \brief An unsigned char cv::Mat.
   */
  typedef cv::Mat_<unsigned char> UnsignedCharImage;
  
  /** \typedef CharImage
   * \brief A char cv::Mat.
   */
  typedef cv::Mat_<char> CharImage;

  /** \typedef UnsignedShortImage
   * \brief An unsigned short cv::Mat.
   */
  typedef cv::Mat_<unsigned short> UnsignedShortImage;
  
  /** \typedef UnsignedIntImage
   * \brief An unsigned int cv::Mat.
   */
  typedef cv::Mat_<unsigned int> UnsignedIntImage;
  
  /** \typedef IntImage
   * \brief An int cv::Mat.
   */
  typedef cv::Mat_<int> IntImage;
  
  /** \typedef FloatImage
   * \brief A float cv::Mat.
   */
  typedef cv::Mat_<float> FloatImage;

  /** \typedef Float3Image
   * \brief A float cv::Mat.
   */
  typedef cv::Mat_<cv::Vec3f> Float3Image;

  /** \typedef DoubleImage
   * \brief A double cv::Mat.
   */
  typedef cv::Mat_<double> DoubleImage;
  
  /** \typedef RawDepthImage
   * \brief An unsigned char cv::Mat used to for depth images with depth values expressed in millimeters.
   */
  typedef UnsignedShortImage RawDepthImage;
  
  /** \typedef IndexImage
   * \brief An int cv::Mat used to save the indeces of the points of a depth image inside a vector of points.
   */
  typedef IntImage IndexImage;
  
  /** \typedef DepthImage
   * \brief A float cv::Mat used to for depth images with depth values expressed in meters.
   */
  typedef cv::Mat_< cv::Vec3b > RGBImage;

  typedef std::vector< cv::Vec3b > RGBVector;

  typedef std::vector<int> IntVector;

  typedef std::vector<float> FloatVector;

}
