
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
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/data/robot_laser.h"
#include "g2o/stuff/command_args.h"

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

typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

class SimpleGraphMap{
public:
  SimpleGraphMap(){
    _currentId = 0;
    _graph = new SparseOptimizer();
    //Init graph
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);
    _graph->setAlgorithm(solver);
    _graph->setVerbose(false);

    _currentVertex = 0;
    _previousVertex = 0;
  };
  
  void addVertex(LaserMessage* las_msg){
    VertexSE2* v = new VertexSE2;
    v->setId(_currentId);

    Eigen::Isometry3f odom = las_msg->odometry(); 

    Vector2d translation(odom.translation().x(), odom.translation().y());
    Eigen::Rotation2Dd rotation(0); 
    rotation.fromRotationMatrix(odom.linear().block<2,2>(0,0));

    SE2 pose;
    pose.setTranslation(translation);
    pose.setRotation(rotation);

    v->setEstimate(pose);

    //Transform LaserMessage to RobotLaser
    LaserParameters lparams(0, las_msg->ranges().size(), las_msg->minAngle(), las_msg->angleIncrement(), las_msg->maxRange(), 0.1, 0);
    SE2 trobotlaser(0, 0, 0); //TODO
    lparams.laserPose = trobotlaser;
    
    RobotLaser* rlaser = new RobotLaser;
    rlaser->setLaserParams(lparams);
    rlaser->setOdomPose(pose);
    std::vector<double> ranges(las_msg->ranges().begin(), las_msg->ranges().end());

    rlaser->setRanges(ranges);
    rlaser->setTimestamp(las_msg->timestamp());
    rlaser->setLoggerTimestamp(rlaser->timestamp());
    rlaser->setHostname("hostname");
    v->setUserData(rlaser);
    
    std::cerr << "Adding Vertex: " << v->estimate().translation().x() << " " << v->estimate().translation().y() << " " << v->estimate().rotation().angle() << std::endl;

    _graph->addVertex(v);
    _currentId++;
    _previousVertex = _currentVertex;
    _currentVertex = v;
  };

  void addEdge(VertexSE2* v1, VertexSE2* v2, Eigen::Isometry2f rel_transf){
    EdgeSE2 *e = new EdgeSE2;
    e->vertices()[0] = v1;
    e->vertices()[1] = v2;
      
    // std::cerr << "Vertex 1: " << v1->estimate().translation().x() << " " << v1->estimate().translation().y() << " " << v1->estimate().rotation().angle() << std::endl;
    // std::cerr << "Vertex 2: " << v2->estimate().translation().x() << " " << v2->estimate().translation().y() << " " << v2->estimate().rotation().angle() << std::endl;

    SE2 displacement(rel_transf.cast<double>());
    // std::cerr << "Displacement: " << displacement.translation().x() << " " << displacement.translation().y() << " " << displacement.rotation().angle() << std::endl;
    e->setMeasurement(displacement);
  
    Matrix3d inf =  100 * Matrix3d::Identity();
    inf(2,2) = 1000;
    
    e->setInformation(inf);
    _graph->addEdge(e);
  };
  
  void optimize(int nrunnings){
    _graph->initializeOptimization();
    _graph->optimize(nrunnings);
  };

  bool saveGraph(const char *filename){
    return _graph->save(filename);
  };

  inline SparseOptimizer* graph() {return _graph;}
  inline VertexSE2* currentVertex() {return _currentVertex;}
  inline VertexSE2* previousVertex() {return _previousVertex;}

protected:
  SparseOptimizer * _graph;
  int _currentId;
  VertexSE2 *_currentVertex, *_previousVertex;
};

class IDoMyStuffTrigger: public SensorMessageSorter::Trigger {
public:
  IDoMyStuffTrigger(SensorMessageSorter* sorter, tsm::Tracker* tracker) : SensorMessageSorter::Trigger(sorter, 0){
    _projector = 0;
    _tracker = tracker;
    firstUse = true;
    prevTransf = Eigen::Isometry2f::Identity();
    prevOdom = Eigen::Isometry3f::Identity();
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

	_tracker->setProjector(_projector);

	tsm::Cloud2D* cloud = new tsm::Cloud2D();
	_projector->unproject(*cloud, las->ranges());
	_tracker->update(cloud);
	global_t = _tracker->globalT();	
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
	float trans_step = 0.3;
	float rot_step = M_PI_4;
	if ((sqrt(displacement.translation().x()*displacement.translation().x() + displacement.translation().y()*displacement.translation().y()) > trans_step) || rotation.angle() > rot_step){
	  Eigen::Isometry3f odom = las->odometry(); 
	  std::cerr << "ODOM: \n" << odom.matrix() << std::endl;
	  
	  tsm::Cloud2D* cloud = new tsm::Cloud2D();
	  _projector->unproject(*cloud, las->ranges());
	  
	  Eigen::Isometry2f initial_guess = Eigen::Isometry2f::Identity();
	  initial_guess.translation().x() = displacement.translation().x();
	  initial_guess.translation().y() = displacement.translation().y();
	  initial_guess.linear() = displacement.linear().block<2,2>(0,0);

	  Eigen::Rotation2Dd rotation(0); 
	  rotation.fromRotationMatrix(initial_guess.linear());
	  std::cerr << "Initial guess: " << initial_guess.translation().x() << " " << initial_guess.translation().y() << " " << rotation.angle() << std::endl;
	  rotation.fromRotationMatrix(prevTransf.linear());
	  std::cerr << "Previous Transf: " << prevTransf.translation().x() << " " << prevTransf.translation().y() << " " << rotation.angle() << std::endl;

	  bool success = _tracker->update(cloud, initial_guess);
	  if (success){
	    global_t = _tracker->globalT();
	    
	    rotation.fromRotationMatrix(global_t.linear());
	    std::cerr << "Result: " << global_t.translation().x() << " " << global_t.translation().y() << " " << rotation.angle() << std::endl;
	    
	    _gmap.addVertex(las);
	    _gmap.addEdge(_gmap.previousVertex(), _gmap.currentVertex(), prevTransf.inverse()*global_t);
	    //_gmap.optimize(1);
	    //_gmap.saveGraph("prueba.g2o");  
	    
	    prevTransf = global_t;
	  }
	  else{
	    //Trust the odometry
	    _gmap.addVertex(las);
	    _gmap.addEdge(_gmap.previousVertex(), _gmap.currentVertex(), initial_guess);
	  }
	  prevOdom = las->odometry();
	}	
      }
    }
  }

  inline SimpleGraphMap graphMap(){return _gmap;}

protected:
  tsm::Projector2D* _projector;
  tsm::Tracker* _tracker;
  SimpleGraphMap _gmap;
  bool firstUse;
  Eigen::Isometry2f prevTransf; 
  Eigen::Isometry3f prevOdom; 
  
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

  arg.param("bpr", bpr, 0.2, "tracker bad points ratio");
  arg.param("it", iterations, 10, "tracker iterations");
  arg.param("inlier_distance", inlier_distance, 0.5, "tracker inlier distance");
  arg.param("min_correspondences_ratio", min_correspondences_ratio, 0.3, "tracker minimum correspondences ratio");
  arg.param("local_map_clipping_range", local_map_clipping_range, 10.0, "tracker local map clipping range");
  arg.param("local_map_clipping_translation_threshold", local_map_clipping_translation_threshold, 5.0, "tracker local map clipping translation threshold");
  arg.param("usegui", use_gui, false, "displays gui");
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
  }

  sorter->flush();
  double finish = getTime() - init;
  cerr << "Total Laserscans: " << count << ". Final graph size: " << idmst->graphMap().graph()->vertices().size() << ". Took " << finish << " seconds" << endl;
  cerr << "Saving file...";
  idmst->graphMap().saveGraph(outgraphFilename.c_str());
  cerr << "Finished" << endl;
}
