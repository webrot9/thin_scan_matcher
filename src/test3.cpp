
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

// g2o
#include "g2o/stuff/command_args.h"

#include "graph_slam/simple_graph.h"
#include "graph_slam/closures_finder.h"

#include <Eigen/Geometry> 

using namespace std;
using namespace txt_io;
using namespace system_utils;
using namespace g2o;

LaserMessage l;

const char* banner[]={
  "test3: runs the tracker with the laserscans of a dump file written with txt io",
  "process a number of laserscans equal to max_nodes separated by a certain distance",
  "looks for possible loop closures",
  "the result is saved in a g2o graph",
  "",
  "Error: you must provide some parameter. Use 'test3 -h' for help. ",
  0
};

class IDoMyStuffTrigger: public SensorMessageSorter::Trigger {
public:
  IDoMyStuffTrigger(SensorMessageSorter* sorter, tsm::Tracker& tracker) : SensorMessageSorter::Trigger(sorter, 0){
    _projector = 0;
    _tracker = tracker;
    firstUse = true;
    prevTransf = Eigen::Isometry2f::Identity();
    prevOdom = Eigen::Isometry3f::Identity();
    _cf.setGraphMap(&_gmap);
  }

  virtual void action(std::tr1::shared_ptr<BaseSensorMessage> msg) {
    //cerr << msg->tag() << endl;

    LaserMessage* las = dynamic_cast<LaserMessage*>(msg.get());
    if(las) {
      cerr << msg->tag() << endl;
      Eigen::Isometry2f global_t = Eigen::Isometry2f::Identity();
      if (firstUse){
	//First use: setting projector with laser info
	_projector = new tsm::Projector2D();
	_projector->setMaxRange(las->maxRange());
	_projector->setMinRange(las->minRange());
	_projector->setNumRanges(las->ranges().size());
	_projector->setFov(las->maxAngle()-las->minAngle());

	_tracker.setProjector(_projector);
	_cf.setProjector(_projector);

	tsm::Cloud2D* cloud = new tsm::Cloud2D();
	_projector->unproject(*cloud, las->ranges());
	_tracker.update(cloud);
	global_t = _tracker.globalT();	
	std::cerr << "FIRST USE tracker result: " << global_t.matrix() << std::endl;

	_gmap.addVertex(las);
	_gmap.currentVertex()->setFixed(true);
	firstUse = false;
	prevOdom = las->odometry();
	prevTransf = global_t;
      }else{
	Eigen::Isometry3f displacement = prevOdom.inverse()*las->odometry();
	Eigen::Rotation2Dd rotation(0); 
	rotation.fromRotationMatrix(displacement.linear().block<2,2>(0,0));
	float trans_step = 0.25;
	float rot_step = M_PI_4*0.5; //M_PI_4;
	if ((sqrt(displacement.translation().x()*displacement.translation().x() + displacement.translation().y()*displacement.translation().y()) > trans_step) || rotation.angle() > rot_step){
	  Eigen::Isometry3f odom = las->odometry(); 
	  
	  tsm::Cloud2D* cloud = new tsm::Cloud2D();
	  _projector->unproject(*cloud, las->ranges());
	  
	  Eigen::Isometry2f initial_guess = Eigen::Isometry2f::Identity();
	  initial_guess.translation().x() = displacement.translation().x();
	  initial_guess.translation().y() = displacement.translation().y();
	  initial_guess.linear() = displacement.linear().block<2,2>(0,0);

	  Eigen::Rotation2Dd rotation(0); 
	  rotation.fromRotationMatrix(initial_guess.linear());
	  std::cerr << "Initial guess: " << initial_guess.translation().x() << " " << initial_guess.translation().y() << " " << rotation.angle() << std::endl;

	  bool success = _tracker.update(cloud, initial_guess);
	  if (success){
	    global_t = _tracker.globalT();

	    rotation.fromRotationMatrix(global_t.linear());
	    std::cerr << "Result: " << global_t.translation().x() << " " << global_t.translation().y() << " " << rotation.angle() << std::endl;
	  
	    _gmap.addVertex(las);
	    Matrix3d inf =  100 * Matrix3d::Identity();
	    inf(2,2) = 1000;
  	    _gmap.addEdge(_gmap.previousVertex(), graphMap().currentVertex(), prevTransf.inverse()*global_t,inf);
	    _gmap.optimize(1);
	    prevTransf = global_t;
	  }
	  else{
	    //Trust the odometry
	    _gmap.addVertex(las);
	    Matrix3d inf =  100 * Matrix3d::Identity();
	    _gmap.addEdge(_gmap.previousVertex(), _gmap.currentVertex(), initial_guess, inf);
	    _gmap.optimize(1);
	  }
	  	  
	  prevOdom = las->odometry();
      
	  _cf.findClosures();
	  _gmap.optimize(1);
	  //_gmap.saveGraph("aux.g2o");  
	}	
      }
    }
  }

  inline SimpleGraphMap graphMap(){return _gmap;}

protected:
  tsm::Projector2D* _projector;
  tsm::Tracker _tracker;
  SimpleGraphMap _gmap;
  bool firstUse;
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
  double bpr;
  int iterations;
  double inlier_distance;
  double min_correspondences_ratio;
  double local_map_clipping_range;
  double local_map_clipping_translation_threshold;

  arg.param("bpr", bpr, 0.2, "tracker bad points ratio"); //1
  arg.param("it", iterations, 10, "tracker iterations"); 
  arg.param("inlier_distance", inlier_distance, 0.5, "tracker inlier distance"); //2
  arg.param("min_correspondences_ratio", min_correspondences_ratio, 0.3, "tracker minimum correspondences ratio");
  arg.param("local_map_clipping_range", local_map_clipping_range, 10.0, "tracker local map clipping range");
  arg.param("local_map_clipping_translation_threshold", local_map_clipping_translation_threshold, 5.0, "tracker local map clipping translation threshold");
  arg.param("maxcount", max_count, 0, "test finishes when <maxcount> laserscans have been processed (0=process all)");
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

  SensorMessageSorter* sorter = new SensorMessageSorter;
  sorter->setWriteBackEnabled(false);
  IDoMyStuffTrigger* idmst = new IDoMyStuffTrigger(sorter, tracker);
  BaseMessage* msg=0;

  if (max_count == 0)
    max_count =std::numeric_limits<int>::max();
  int count = 0;
  while ((count < max_count) && (msg = reader.readMessage()) ) {
    txt_io::BaseSensorMessage* sensor_msg = dynamic_cast<txt_io::BaseSensorMessage*>(msg);
    sorter->insertMessage(sensor_msg);

    //for testing
    LaserMessage* las = dynamic_cast<LaserMessage*>(sensor_msg);
    if (las)
      count++;    
  }

  sorter->flush();
  double finish = getTime() - init;
  cerr << "Total Laserscans: " << count << ". Final graph size: " << idmst->graphMap().graph()->vertices().size() << ". Took " << finish << " seconds" << endl;
  cerr << "Saving file...";
  idmst->graphMap().saveGraph(outgraphFilename.c_str());
  cerr << "Finished" << endl;
}
