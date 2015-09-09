#include "cloud2d.h"
#include <string>
#include <stdexcept>
#include <GL/gl.h>
using namespace std;

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


    glPushAttrib(GL_COLOR);
    glBegin(GL_POINTS);
    for(size_t i=0; i<size(); i++){
      const RichPoint2D& p=at(i);
      if (p.color().z()>0) {
      	glColor3f(p.color().x(), p.color().y(), p.color().z());
	glNormal3f(0,0,1);
	glVertex3f(p.point().x(), p.point().y(), 0);
      } 
    }
    glEnd();
    glPopAttrib();
    
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
      glPushAttrib(GL_COLOR);
      glBegin(GL_LINES);
      for(size_t i=0; i<size(); i++){
	const RichPoint2D& p=at(i);
	glColor3f(p.color().x(), p.color().y(), p.color().z());
	glNormal3f(0,0,1);
	glVertex3f(p.point().x(), p.point().y(), 0);
	glVertex3f(p.point().x()+p.normal().x()*nscale, p.point().y()+p.normal().y()*nscale, 0);
      }
      glEnd();
      glPopAttrib();
    }
    glPopMatrix();
  }

}
