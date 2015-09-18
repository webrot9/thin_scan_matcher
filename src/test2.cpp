
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

#include "graph_slam/simple_graph.h"

#include <Eigen/Geometry> 

using namespace std;
using namespace txt_io;
using namespace system_utils;
using namespace g2o;

LaserMessage l;

const char* banner[]={
  "test2: runs the tracker with the laserscans of a dump file written with txt io",
  "process a number of sequential laserscans equal to max_nodes",
  "the result is saved in a g2o graph",
  "",
  "Error: you must provide some parameter. Use 'test2 -h' for help. ",
  0
};

class IDoMyStuffTrigger: public SensorMessageSorter::Trigger {
public:
  IDoMyStuffTrigger(SensorMessageSorter* sorter, tsm::Tracker* tracker) : SensorMessageSorter::Trigger(sorter, 0){
    _projector = 0;
    _tracker = tracker;
    firstUse = true;
    prevTransf = Eigen::Isometry2f::Identity();
  }

  virtual void action(std::tr1::shared_ptr<BaseSensorMessage> msg) {
    cerr << msg->tag() << endl;

    LaserMessage* las = dynamic_cast<LaserMessage*>(msg.get());
    if(las) {
      if (firstUse){
	//First use: setting projector with laser info
	_projector = new tsm::Projector2D();
	_projector->setMaxRange(las->maxRange());
	_projector->setMinRange(las->minRange());
	_projector->setNumRanges(las->ranges().size());
	_projector->setFov(las->maxAngle()-las->minAngle());

	_tracker->setProjector(_projector);
      }

      tsm::Cloud2D* cloud = new tsm::Cloud2D();
      _projector->unproject(*cloud, las->ranges());

      _tracker->update(cloud);
      Eigen::Isometry2f global_t = _tracker->globalT();

      //update g2o map
      if (firstUse){
	_gmap.addVertex(las);
	_gmap.currentVertex()->setFixed(true);
	firstUse = false;
      }else{
	_gmap.addVertex(las);
	Matrix3d inf =  100 * Matrix3d::Identity();
	_gmap.addEdge(_gmap.previousVertex(), _gmap.currentVertex(), prevTransf.inverse()*global_t, inf);
      }
      //_gmap.saveGraph("aux.g2o"); 

      prevTransf = global_t;
    }
  }

  inline SimpleGraphMap graphMap(){return _gmap;}

protected:
  tsm::Projector2D* _projector;
  tsm::Tracker* _tracker;
  SimpleGraphMap _gmap;
  bool firstUse;
  Eigen::Isometry2f prevTransf; 
  
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
  int skip;

  arg.param("bpr", bpr, 0.2, "tracker bad points ratio");
  arg.param("it", iterations, 10, "tracker iterations");
  arg.param("inlier_distance", inlier_distance, 0.1, "tracker inlier distance");
  arg.param("min_correspondences_ratio", min_correspondences_ratio, 0.3, "tracker minimum correspondences ratio");
  arg.param("local_map_clipping_range", local_map_clipping_range, 10.0, "tracker local map clipping range");
  arg.param("local_map_clipping_translation_threshold", local_map_clipping_translation_threshold, 5.0, "tracker local map clipping translation threshold");
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
  if (use_gui) {
    app=new QApplication(argc, argv);
    viewer=new tsm::TrackerViewer(&tracker);
    viewer->init();
    viewer->show();
  }

  SensorMessageSorter* sorter = new SensorMessageSorter;
  sorter->setWriteBackEnabled(false);
  IDoMyStuffTrigger* idmst = new IDoMyStuffTrigger(sorter, &tracker);
  BaseMessage* msg=0;
  
  if (max_count == 0)
    max_count =std::numeric_limits<int>::max();
  int count = 0;
  while ((count < max_count) && (msg = reader.readMessage()) ) {
    if (!skip){
      txt_io::BaseSensorMessage* sensor_msg = dynamic_cast<txt_io::BaseSensorMessage*>(msg);
      sorter->insertMessage(sensor_msg);
      
      if (use_gui) {
	viewer->updateGL();
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
  cerr << "Laserscans: " << count << ". Took " << finish << " seconds" << endl;
  cerr << "Saving file...";
  idmst->graphMap().saveGraph(outgraphFilename.c_str());
  cerr << "Finished" << endl;
}
