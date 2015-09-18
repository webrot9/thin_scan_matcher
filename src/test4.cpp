
#include "globals/system_utils.h"

#include "txt_io/laser_message.h"
#include "txt_io/sensor_message_sorter.h"
#include "txt_io/message_reader.h"

// tsm_core
#include "tsm_core/defs.h"
#include "tsm_core/cloud2d.h"
#include "tsm_core/projector2d.h"
#include "tsm_core/correspondence_finder2d.h"
#include "tsm_core/cloud_processor.h"
#include "tsm_core/solver2d.h"
#include "tsm_core/tracker.h"

#include "qapplication.h"
#include "tsm_core/tracker_viewer.h"

// g2o
#include "g2o/stuff/command_args.h"
#include "g2o/apps/g2o_viewer/g2o_qglviewer.h"

#include "graph_slam/simple_graph.h"
#include "graph_slam/closures_finder.h"

#include <Eigen/Geometry> 

using namespace std;
using namespace txt_io;
using namespace system_utils;
using namespace g2o;

LaserMessage l;

const char* banner[]={
  "test1: runs the tracker with the laserscans of a dump file written with txt io",
  "process a number of laserscans equal to max_nodes separated by a certain distance",
  "the result is saved in a g2o graph",
  "",
  "Error: you must provide some parameter. Use 'test1 -h' for help. ",
  0
};

class IDoMyStuffTrigger: public SensorMessageSorter::Trigger {
public:
  IDoMyStuffTrigger(SensorMessageSorter* sorter, tsm::Tracker* tracker) : SensorMessageSorter::Trigger(sorter, 0){
    _projector = 0;
    _tracker = tracker;
    firstUse = true;
    _trans_step = 0.5;
    _rot_step = M_PI_4;
    prevTransf = Eigen::Isometry2f::Identity();
    prevOdom = Eigen::Isometry3f::Identity();
    _cf.setGraphMap(&_gmap);
  }

  virtual void action(std::tr1::shared_ptr<BaseSensorMessage> msg) {
    //cerr << msg->tag() << endl;

    LaserMessage* las = dynamic_cast<LaserMessage*>(msg.get());
    if(las) {
      cerr << msg->tag() << endl;
      if (firstUse){
	//First use: setting projector with laser info
	_projector = new tsm::Projector2D();
	_projector->setMaxRange(las->maxRange());
	_projector->setMinRange(las->minRange());
	_projector->setNumRanges(las->ranges().size());
	_projector->setFov(las->maxAngle()-las->minAngle());

	_tracker->setProjector(_projector);
	_cf.setProjector(_projector);

	prevOdom = las->odometry(); 
	//set first virtex in (0,0,0)
	las->setOdometry(Eigen::Isometry3f::Identity());
	_gmap.addVertex(las);
	_gmap.currentVertex()->setFixed(true);
	firstUse = false;
	prevTransf = Eigen::Isometry2f::Identity();
      }

      std::cerr << "Odom: " << las->odometry().translation().transpose() << std::endl;


      tsm::Cloud2D* cloud = new tsm::Cloud2D();
      _projector->unproject(*cloud, las->ranges());
      bool success = _tracker->update(cloud);

      Eigen::Isometry2f global_t = _tracker->globalT();	

      Eigen::Isometry2f displacement = prevTransf.inverse()*global_t;
      Eigen::Rotation2Dd rotation(0); 
      rotation.fromRotationMatrix(displacement.linear().block<2,2>(0,0));

      if ((sqrt(displacement.translation().x()*displacement.translation().x() + displacement.translation().y()*displacement.translation().y()) > _trans_step) || fabs(rotation.angle()) > _rot_step ){//|| !success){
	Eigen::Isometry3f odom = las->odometry(); 
	Eigen::Isometry3f odom_displacement = prevOdom.inverse()*odom;
	Eigen::Isometry2f odom_displacement2f = Eigen::Isometry2f::Identity();
	odom_displacement2f.translation().x() = odom_displacement.translation().x();
	odom_displacement2f.translation().y() = odom_displacement.translation().y();
	odom_displacement2f.linear() = odom_displacement.linear().block<2,2>(0,0);

	//create projector of 2*M_PI
	float fov = 2*M_PI; 
	int num_ranges = 720;
	tsm::Projector2D p;
	p.setFov(fov);
	p.setNumRanges(num_ranges);
	
	//project current cloud to ranges
	tsm::FloatVector reference_ranges;
	tsm::IntVector reference_indices;
	p.project(reference_ranges, reference_indices, Eigen::Isometry2f::Identity(), *_tracker->reference());
	//create laser data from ranges
	LaserMessage* las_msg = new LaserMessage();
	
	las_msg->setOdometry(odom);
	las_msg->setMinAngle(0);
	las_msg->setMaxAngle(fov);
	las_msg->setAngleIncrement(p.angleIncrement());
	las_msg->setMinRange(p.minRange());
	las_msg->setMaxRange(p.maxRange());
	las_msg->setRanges(reference_ranges);
	
	//include in g2o
	_gmap.addVertex(las);
	Matrix3d inf =  100 * Matrix3d::Identity();
	inf(2,2) *= 10;
	//if (success){
	  //include both relative displacement from odom and tracker
	  _gmap.addEdge(_gmap.previousVertex(), graphMap().currentVertex(), displacement, inf);
	  //inf *=0.1;
	  //_gmap.addEdge(_gmap.previousVertex(), graphMap().currentVertex(),  odom_displacement2f, inf);
	  //}
	  /*else{
	  //include just odometry with low information matrix
	  //inf *=0.1;
	  _gmap.addEdge(_gmap.previousVertex(), graphMap().currentVertex(),  odom_displacement2f, inf);
	  }*/

	prevTransf = global_t;
	_gmap.optimize(1);
	_cf.findClosures();
	_gmap.optimize(1);
	_gmap.saveGraph("aux.g2o"); 
	prevOdom = odom;
      }

    }
  }

  inline SimpleGraphMap graphMap(){return _gmap;}
  inline void setTranslationStep(float trans_step){_trans_step = trans_step;}
  inline void setRotationStep(float rot_step){_rot_step = rot_step;}
  
  inline ClosuresFinder& closuresFinder(){return _cf;}
protected:
  tsm::Projector2D* _projector;
  tsm::Tracker* _tracker;
  SimpleGraphMap _gmap;
  bool firstUse;
  
  float _trans_step, _rot_step;
  
  Eigen::Isometry2f prevTransf; 
  Eigen::Isometry3f prevOdom;
  ClosuresFinder _cf;
};


int main(int argc, char **argv){
  if (argc<2) {
    printBanner(banner);
    return 0;
  }

  CommandArgs arg;
  string dumpFilename, outgraphFilename;
  int max_count;
  bool use_gui;
  double bpr;
  int iterations;
  double inlier_distance;
  double min_correspondences_ratio;
  double local_map_clipping_range;
  double local_map_clipping_translation_threshold;
  float trans_step, rot_step;
  int cf_min_inliers;
  float cf_inlier_threshold;
  int skip;

  arg.param("bpr", bpr, 0.2, "tracker bad points ratio");
  arg.param("it", iterations, 10, "tracker iterations");
  arg.param("inlier_distance", inlier_distance, 0.1, "tracker inlier distance");
  arg.param("min_correspondences_ratio", min_correspondences_ratio, 0.3, "tracker minimum correspondences ratio");
  arg.param("local_map_clipping_range", local_map_clipping_range, 10.0, "tracker local map clipping range");
  arg.param("local_map_clipping_translation_threshold", local_map_clipping_translation_threshold, 5.0, "tracker local map clipping translation threshold");
  arg.param("transtep", trans_step, 0.5, "translation step threshold");
  arg.param("rotstep", rot_step, M_PI_4, "rotation step threshold");
  arg.param("cf_inlier_threshold", cf_inlier_threshold, 3., "inlier threshold to look for loop closures");
  arg.param("cf_min_inliers", cf_min_inliers, 6, "mininum number of inliers to look for loop closures");

  arg.param("usegui", use_gui, false, "displays gui");
  arg.param("maxcount", max_count, 0, "test finishes when <maxcount> laserscans have been processed (0=process all)");
  arg.param("skip", skip, 0, "skips the <skip>th first laserscans");
  arg.param("o", outgraphFilename, "out.g2o", "file where to save the output graph");
  arg.paramLeftOver("dump-file", dumpFilename, "", "input dump file in txt io format");
  arg.parseArgs(argc, argv);

  MessageReader reader;
  reader.open(dumpFilename);

  double init = getTime();

  //Init tracker
  tsm::Tracker tracker;
  tracker.setBpr(bpr);
  tracker.setIterations(iterations);
  tracker.setInlierDistance(inlier_distance);
  tracker.setMinCorrespondencesRatio(min_correspondences_ratio);
  tracker.setLocalMapClippingRange(local_map_clipping_range);
  tracker.setClipTranslationThreshold(local_map_clipping_translation_threshold);
  tsm::Solver2D solver;
  tracker.setSolver(&solver);

  QApplication* app=0;
  tsm::TrackerViewer* viewer=0;
  G2oQGLViewer* g2oviewer=0;

  if (use_gui) {
    app=new QApplication(argc, argv);
    viewer=new tsm::TrackerViewer(&tracker);
    viewer->init();
    viewer->show();
    g2oviewer=new G2oQGLViewer;
    g2oviewer->init();
    g2oviewer->show();
  }

  SensorMessageSorter* sorter = new SensorMessageSorter;
  sorter->setWriteBackEnabled(false);
  IDoMyStuffTrigger* idmst = new IDoMyStuffTrigger(sorter, &tracker);
  BaseMessage* msg=0;
  
  idmst->setTranslationStep(trans_step);
  idmst->setRotationStep(rot_step);
  idmst->closuresFinder().setMinInliers(cf_min_inliers);
  idmst->closuresFinder().setInlierThreshold(cf_inlier_threshold);

  if (use_gui)
    g2oviewer->graph = idmst->graphMap().graph();

  if (max_count == 0)
    max_count =std::numeric_limits<int>::max();
  int count = 0;
  int prevSize = idmst->graphMap().graph()->vertices().size();

  while ((count < max_count) && (msg = reader.readMessage()) ) {
    if (!skip){
      txt_io::BaseSensorMessage* sensor_msg = dynamic_cast<txt_io::BaseSensorMessage*>(msg);
      sorter->insertMessage(sensor_msg);
      
      if (use_gui) {
	viewer->updateGL();
	int currSize = idmst->graphMap().graph()->vertices().size();
	if (currSize != prevSize) {
	  g2oviewer->setUpdateDisplay(true);
	  g2oviewer->updateGL();
	  prevSize = currSize;
	}
	app->processEvents();
	usleep(10000);
      }

      //for testing
      LaserMessage* las = dynamic_cast<LaserMessage*>(sensor_msg);
      if (las)
	count++;    
    }else
      skip--;
  }

  sorter->flush();
  double finish = getTime() - init;
  cerr << "Total Laserscans: " << count << ". Final graph size: " << idmst->graphMap().graph()->vertices().size() << ". Took " << finish << " seconds" << endl;
  cerr << "Saving file...";
  idmst->graphMap().saveGraph(outgraphFilename.c_str());
  cerr << "Finished" << endl;
}
