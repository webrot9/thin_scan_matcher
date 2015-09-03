#include "correspondence_finder2d.h"
#include <stdexcept>
#include <GL/gl.h>
namespace tsm {

  CorrespondenceFinder2D::CorrespondenceFinder2D(Projector2D* projector, Solver2D* solver) {
      _correspondences.clear();
      _projector = projector;
      _solver = solver;
      _max_squared_distance=0.3*0.3;
      _min_normal_cos=cos(M_PI/8);
    }

  void CorrespondenceFinder2D::init() {
    _correspondences.clear();
    _indices_current.clear();

    if(_projector == NULL || _solver == NULL) {
      throw std::runtime_error("Cannot init correspondences without a projector and a solver");
    }

    _projector->project(
			_projected_reference_ranges,
			_indices_reference,
			Eigen::Isometry2f::Identity(),
			*_solver->reference()
			);
  }

  void CorrespondenceFinder2D::compute() {
    _correspondences.clear();
    _indices_current.clear();

    if(_projector == NULL || _solver == NULL) {
      throw std::runtime_error("Cannot compute correspondences without a projector and a solver");
    }

    _projector->project(
			_projected_current_ranges,
			_indices_current,
			_solver->T(),
			*_solver->current()
			);

    size_t size = std::min(_indices_current.size(), _indices_reference.size());

    for (size_t i = 0; i < size; ++i) {
      int idx1 = _indices_current[i];
      int idx2 = _indices_reference[i];
       
      if (idx1 < 0 || idx2 < 0)
	continue;

      RichPoint2D pt_curr = (*_solver->current())[idx1];
      pt_curr.transformInPlace(_solver->T());
      RichPoint2D pt_ref = (*_solver->reference())[idx2];

      if((pt_curr.point() - pt_ref.point()).squaredNorm() < _max_squared_distance &&
	 pt_curr.normal().dot(pt_ref.normal()) > _min_normal_cos) {
	_correspondences.push_back(i);
      }
    }
  }

  void CorrespondenceFinder2D::drawCorrespondences(RGBImage &img, Eigen::Isometry2f T, float scale) const {
    _solver->current()->draw(img, cv::Vec3b(0,255,0), false, T, false, scale);
    _solver->reference()->draw(img, cv::Vec3b(0,0,255), false, Eigen::Isometry2f::Identity(), false, scale);
    
    T.linear()*=scale;
    for (size_t i = 0; i < _correspondences.size(); ++i) {
       int idx1 = _indices_current[_correspondences[i]];
       int idx2 = _indices_reference[_correspondences[i]];

       Eigen::Vector2f pt_curr = T*(*_solver->current())[idx1].point() * 0.5;
       Eigen::Vector2f pt_ref = T*(*_solver->reference())[idx2].point() * 0.5;

       float center_x = img.cols * 0.5;
       float center_y = img.rows * 0.5;
       cv::line(img,
  		cv::Point(pt_curr.x() + center_x, pt_curr.y() + center_y),
  		cv::Point(pt_ref.x() + center_x, pt_ref.y() + center_y),
  		cv::Scalar(255, 0, 0)
  		);
    }
  }
    

  void CorrespondenceFinder2D::drawCorrespondences(Eigen::Isometry2f T) const {
    glPushAttrib(GL_COLOR);
    glColor3f(0,1,0);
    _solver->current()->draw(false, T);
    glColor3f(0,0,1);
    _solver->reference()->draw(false);
    glColor3f(1,0,0);
    glBegin(GL_LINES);
    for (size_t i = 0; i < _correspondences.size(); ++i) {
      int idx1 = _indices_current[_correspondences[i]];
      int idx2 = _indices_reference[_correspondences[i]];

      Eigen::Vector2f pt_curr = T*(*_solver->current())[idx1].point();
      Eigen::Vector2f pt_ref = (*_solver->reference())[idx2].point();
      glNormal3f(0,0,1);
      glVertex3f(pt_ref.x(), pt_ref.y(),0);
      glNormal3f(0,0,1);
      glVertex3f(pt_curr.x(), pt_curr.y(),0);
    }
    glEnd();
    glPopAttrib();
  }
}

