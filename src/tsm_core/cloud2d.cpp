#include "cloud2d.h"
#include <string>
#include <stdexcept>
#include <GL/gl.h>

namespace tsm{
  void Cloud2D::add(const Cloud2D& other) {
    if (&other == this) {
      return;
    }

    size_t k = size();
    resize(k + other.size());

    for (size_t i = 0; i < other.size(); ++i) {
      at(k + i) = other.at(i);
    }
  }


  void Cloud2D::transformInPlace(const Eigen::Isometry2f& T) {
    Eigen::Matrix2f R = T.linear();

    for (size_t i = 0; i < size();++ i) {
      if (at(i).accumulator() <= 0) {
       	throw std::runtime_error("Negative Point Accumulator");
      }

      at(i).transformInPlace(T);
    }
  }

  void Cloud2D::transform(Cloud2D& other, const Eigen::Isometry2f& T) const {
    other.resize(size());
    Eigen::Matrix2f R = T.linear();

    for (size_t i = 0; i < size(); ++i) {
      other[i] = at(i).transform(T);
    }
  }

  //! clips to a max_range around a pose
  void Cloud2D::clip(float max_range, const Eigen::Isometry2f& pose) {
    Eigen::Isometry2f T = pose.inverse();
    max_range *= max_range;

    int k = 0;
    for (size_t i = 0; i < size(); ++i) {
      const RichPoint2D& p = at(i);
      Eigen::Vector2f other_p = T * p.point();

      if (other_p.squaredNorm() < max_range) {
	at(k) = p;
	k++;
      }
    }

    resize(k);
  }

  void Cloud2D::voxelize(Cloud2D& model, float res) {
    Cloud2D sparse_model;
    float ires = 1. / res;

    std::vector<IndexPair> voxels(model.size());

    for (int i = 0; i < model.size(); ++i){
      voxels[i] = IndexPair(model[i].point(), i , ires);
    }

    sparse_model.resize(model.size());
    std::sort(voxels.begin(), voxels.end());

    int k = -1;
    for (size_t i = 0; i < voxels.size(); ++i) { 
      IndexPair& pair = voxels[i];
      int idx = pair.index;

      if (k >= 0 && voxels[i].sameCell(voxels[i-1])) {
	sparse_model[k] += model[idx];
      } else {
	sparse_model[++k] = model[idx];
      } 
    }

    sparse_model.resize(k);

    for (size_t i = 0; i < sparse_model.size(); ++i) {
      if (sparse_model[i].accumulator() <= 0)
	throw std::runtime_error("Negative Point Accumulator");

      sparse_model[i].normalize();
    }

    model = sparse_model;
  }

  void Cloud2D::draw(RGBImage &img, cv::Vec3b color, bool draw_normals, Eigen::Isometry2f T,
  		     bool draw_pose_origin, float scale) const {
    std::vector<cv::Point2i> normal_to_draw, pt_to_draw;
    float max_value = std::numeric_limits<float>::min();

    for(Cloud2D::const_iterator it = begin(); it != end(); ++it) {
      Eigen::Vector2f pt = T * it->point();
      Eigen::Vector2f n = T.linear() * it->normal();
      float value = std::max(std::fabs(pt.x()), std::fabs(pt.y()));

      if (value > max_value)
  	max_value = value + 1;

      pt = pt * scale * 0.5;
      n = n * scale * 0.5;
      
      if (pt.x() != pt.x() || pt.y() != pt.y())
  	continue;

      pt_to_draw.push_back(cv::Point2i(static_cast<int>(pt.x()), static_cast<int>(pt.y())));
      normal_to_draw.push_back(cv::Point2i(static_cast<int>(n.x()), static_cast<int>(n.y())));
    }

    float max_dest_size = std::max(img.rows, img.cols);
    int img_size = std::max(max_value * scale, max_dest_size);

    RGBImage tmp = cv::Mat::zeros(img_size, img_size, CV_8UC3);
    for(size_t i = 0; i < pt_to_draw.size(); ++i) {
      cv::Point2i& pt = pt_to_draw[i];
      
      pt.x += img_size * 0.5;
      pt.y += img_size * 0.5;
      
      cv::Point2i& normal = normal_to_draw[i];

      if (draw_normals)
  	cv::line(tmp,
  		 pt,
  		 pt + normal,
  		 100
  		 );

      tmp.at<cv::Vec3b>(pt.y,pt.x) = color;
    }
    
    cv::circle(tmp,
  	       cv::Point(T.translation().x() * scale * 0.5 + img_size * 0.5,
  			 T.translation().y() * scale * 0.5 + img_size * 0.5),
  	       3,
  	       cv::Scalar(color[0],color[1],color[2]),
  	       CV_FILLED);
	       
    if (img.size().area() > 0) {
      float max_cols = std::max(img.cols, tmp.cols);
      cv::Rect roi = cv::Rect((tmp.cols - img.cols) * 0.5,
  			      (tmp.rows - img.rows) * 0.5,
  			      img.cols,
  			      img.rows);
      tmp(roi) += img;
    }

    img = tmp;
  }

  Cloud2D::~Cloud2D(){}


  void Cloud2D::draw(bool draw_normals,
		     Eigen::Isometry2f T,
		     bool draw_pose_origin, 
		     bool use_fans) const {
    glPushMatrix();
    Eigen::Vector3f t=t2v(T);
    glTranslatef(t.x(),
		 t.y(),
		 0);
    glRotatef(t.z()*180.0f/M_PI,0,0,1);


    glBegin(GL_POINTS);
    for(size_t i=0; i<size(); i++){
      const RichPoint2D& p=at(i);
      glNormal3f(0,0,1);
      glVertex3f(p.point().x(), p.point().y(), 0);
    }
    glEnd();

    glPushAttrib(GL_COLOR);
    if (use_fans) {
      glColor4f(.5, .5, .5, 0.1);
      glBegin(GL_TRIANGLE_FAN);
      glNormal3f(0,0,1);
      glVertex3f(0,0,0);
      for(size_t i=0; i<size(); i++){
	const RichPoint2D& p=at(i);
	glNormal3f(0,0,1);
	glVertex3f(p.point().x(), p.point().y(), 0);
      }
      glEnd();
    } 
    glPopAttrib();


    float nscale=0.1;
    if (draw_normals){
      glBegin(GL_LINES);
      for(size_t i=0; i<size(); i++){
	const RichPoint2D& p=at(i);
	glNormal3f(0,0,1);
	glVertex3f(p.point().x(), p.point().y(), 0);
	glVertex3f(p.point().x()+p.normal().x()*nscale, p.point().y()+p.normal().x()*nscale, 0);
      }
      glEnd();
    }
    glPopMatrix();
  }

}
