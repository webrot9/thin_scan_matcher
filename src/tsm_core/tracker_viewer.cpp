#include "tracker_viewer.h"

// some macro helpers for identifying the version number of QGLViewer
// QGLViewer changed some parts of its API in version 2.6.
// The following preprocessor hack accounts for this.
#if (((QGLVIEWER_VERSION & 0xff0000) >> 16) >= 2 && ((QGLVIEWER_VERSION & 0x00ff00) >> 8) >= 6)
#define qglv_real qreal
#else
#define qglv_real float
#endif

namespace tsm {
  using namespace std;
  using namespace Eigen;
  
  class StandardCamera: public qglviewer::Camera {
  public:
    StandardCamera(): _standard(true) {}
  
    qglv_real zNear() const {
      if(_standard) { return 0.001f; } 
      else { return Camera::zNear(); } 
    }

    qglv_real zFar() const {
      if(_standard) { return 10000.0f; } 
      else { return Camera::zFar(); }
    }

    bool standard() const { return _standard; }  
    void setStandard(bool s) { _standard = s; }

  protected:
    bool _standard;
  };


  QKeyEvent* TrackerViewer::lastKeyEvent() {
    if (_last_key_event_processed)
      return 0;
    return &_last_key_event;
  }
  
  void TrackerViewer::keyEventProcessed() {
    _last_key_event_processed = true;
  }
  TrackerViewer::TrackerViewer(Tracker* tracker_) :
    _last_key_event(QEvent::None, 0, Qt::NoModifier){
    _last_key_event_processed = true;
    _tracker=tracker_;
  }

  void TrackerViewer::keyPressEvent(QKeyEvent *e) {
    QGLViewer::keyPressEvent(e);
      _last_key_event = *e;
      _last_key_event_processed=false;
  }

  void TrackerViewer::init() {
    // Init QGLViewer.
    QGLViewer::init();
    // Set background color light yellow.
    // setBackgroundColor(QColor::fromRgb(255, 255, 194));

    
    // Set background color white.
    setBackgroundColor(QColor::fromRgb(255, 255, 255));

    // Set some default settings.
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND); 
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_FLAT);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Don't save state.
    setStateFileName(QString::null);

    // Mouse bindings.
    setMouseBinding(Qt::RightButton, CAMERA, ZOOM);
    setMouseBinding(Qt::MidButton, CAMERA, TRANSLATE);
    setMouseBinding(Qt::ControlModifier, Qt::LeftButton, RAP_FROM_PIXEL);

    // Replace camera.
    qglviewer::Camera *oldcam = camera();
    qglviewer::Camera *cam = new StandardCamera();
    setCamera(cam);
    cam->setPosition(qglviewer::Vec(0.0f, 0.0f, 10.0f));
    cam->setUpVector(qglviewer::Vec(1.0f, 0.0f, 0.0f));
    cam->lookAt(qglviewer::Vec(0.0f, 0.0f, 0.0f));
    delete oldcam;
  }
  
  void TrackerViewer::draw(){
    if (!_tracker)
      return;
    Eigen::Vector3f t=t2v(_tracker->globalT());
    glPushMatrix();
    glTranslatef(t.x(), t.y(), 0);
    glRotatef(180.0f/M_PI*t.z(), 0,0,1);
      
    glPushAttrib(GL_COLOR|GL_POINT_SIZE);
    glPointSize(1);
    if (_tracker->reference()) {
      _tracker->reference()->draw(true);
    }

    glPointSize(3);
    if (_tracker->current()) {
      _tracker->current()->draw(false, _tracker->solver()->T().inverse(),false,true);
    }
    glPopAttrib();
    glPopMatrix();
  }

}
