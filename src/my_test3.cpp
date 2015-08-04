
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

//graph_slam
#include "graph_slam/vertices_finder.h"

#include <Eigen/Geometry> 

using namespace std;
using namespace txt_io;
using namespace system_utils;
using namespace g2o;

LaserMessage l;

const char* banner[]={
  "txt_io_dump_reader_app: a simple example on reading a dump file written with txt io",
  " it reads sequentially all elements in the file",
  " instantiates the objects and prints the class name",
  "",
  "usage: my_test <dump_file> max_nodes",
  0
};

tsm::Cloud2D* cloudFromVertex(tsm::Projector2D* projector, OptimizableGraph::Vertex* vertex){
  tsm::Cloud2D* vcloud = 0;

  RobotLaser* laserv = dynamic_cast<RobotLaser*>(vertex->userData());
  if (laserv){
    vcloud = new tsm::Cloud2D();
    tsm::FloatVector floatranges(laserv->ranges().begin(), laserv->ranges().end());
    projector->unproject(*vcloud, floatranges);
  }
  return vcloud;
}

tsm::Cloud2D* cloudFromVSet(tsm::Projector2D* projector, OptimizableGraph::VertexSet& vset, OptimizableGraph::Vertex* refVertex){
  tsm::Cloud2D* finalCloud = new tsm::Cloud2D();
  
  VertexSE2* refVertexSE2 = dynamic_cast<VertexSE2*>(refVertex);
  for (OptimizableGraph::VertexSet::iterator it = vset.begin(); it != vset.end(); it++){

    VertexSE2 *vertex = dynamic_cast<VertexSE2*>(*it);

    tsm::Cloud2D* vcloud = cloudFromVertex(projector, vertex);
    
    if (vcloud){
      SE2 reltransf = refVertexSE2->estimate().inverse()*vertex->estimate();
      Eigen::Isometry2d reltransf2d = reltransf.toIsometry();

      vcloud->transformInPlace(reltransf2d.cast<float>());
      finalCloud->add(*vcloud);
    }
  }

  // tsm::RGBImage img;
  // finalCloud->draw(img, cv::Vec3b(0,255,0), false, Eigen::Isometry2f::Identity(), false, 30);
  // cv::imshow( "Merged Cloud", img );
  // cv::waitKey(0); 
  
  return finalCloud;
}


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
    _projector = 0;
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
    SE2 displacement(rel_transf.cast<double>());
    std::cerr << "Adding Edge between : " << v1->id() << " and " << v2->id() << ". Displacement: " << displacement.translation().x() << " " << displacement.translation().y() << " " << displacement.rotation().angle() << std::endl;

    EdgeSE2 *e = new EdgeSE2;
    e->vertices()[0] = v1;
    e->vertices()[1] = v2;
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

  void findConstraints(){
    cerr << "\nFinding constraints"  << endl;
    VerticesFinder vf(_graph);
    
    OptimizableGraph::VertexSet vset;
    vf.findVerticesScanMatching(currentVertex(), vset);
    
    std::set<OptimizableGraph::VertexSet> setOfVSet;
    vf.findSetsOfVertices(vset, setOfVSet);

    OptimizableGraph::EdgeSet loopClosingEdges;
    for (std::set<OptimizableGraph::VertexSet>::iterator it = setOfVSet.begin(); it != setOfVSet.end(); it++) {
    
      OptimizableGraph::VertexSet myvset = *it;
    
      OptimizableGraph::Vertex* closestV = vf.findClosestVertex(myvset, currentVertex()); 

      if (abs(currentVertex()->id() - closestV->id()) > 10){
	
  	//try to match
  	//Parameters (fixed by the moment)
  	double bpr = 0.2;
  	int iterations = 10;
  	double inlier_distance = .5;
  	double min_correspondences_ratio = 0.3;
  	double local_map_clipping_range = 10.0;
  	double local_map_clipping_translation_threshold = 5.0;

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

	tsm::Projector2D p;
	p.setFov(2*M_PI);
	tracker.setProjector(&p);

	tsm::Cloud2D* cvset = cloudFromVSet(_projector, myvset, closestV);
	tsm::Cloud2D* cvertex = cloudFromVertex(_projector, currentVertex());

	tracker.setReference(cvset);

	VertexSE2* refv = (VertexSE2*) closestV;
	VertexSE2* curv = (VertexSE2*) currentVertex();
	SE2 initguessSE2 = refv->estimate().inverse()*curv->estimate();
	Eigen::Isometry2d initguess2d = initguessSE2.toIsometry();
	
	std::cerr << "ReferenceVertex: " << refv->estimate().translation().x() << " " << refv->estimate().translation().y() << " " << refv->estimate().rotation().angle() << std::endl;
	std::cerr << "CurrentVertex: " << curv->estimate().translation().x() << " " << curv->estimate().translation().y() << " " << curv->estimate().rotation().angle() << std::endl;

	Eigen::Rotation2Dd rotation(0); 
	rotation.fromRotationMatrix(initguess2d.linear());
	std::cerr << "Initial guess: " << initguess2d.translation().x() << " " << initguess2d.translation().y() << " " << rotation.angle() << std::endl;
	tracker.setCurrent(cvertex);

	
	bool success = tracker.match(initguess2d.cast<float>());

	if (success)
	  addEdge(refv, curv, tracker.solver()->T());
	else
	  cerr << "Loop closure rejected." << endl;

	// loopClosingEdges.insert(ne);

      }
    }
  }

  bool saveGraph(const char *filename){
    return _graph->save(filename);
  };

  inline SparseOptimizer* graph() {return _graph;}
  inline VertexSE2* currentVertex() {return _currentVertex;}
  inline VertexSE2* previousVertex() {return _previousVertex;}

  inline void setProjector(tsm::Projector2D* projector){_projector = projector;}
protected:
  SparseOptimizer * _graph;
  int _currentId;
  VertexSE2 *_currentVertex, *_previousVertex;
  tsm::Projector2D* _projector;
};

class IDoMyStuffTrigger: public SensorMessageSorter::Trigger {
public:
  IDoMyStuffTrigger(SensorMessageSorter* sorter, tsm::Tracker& tracker) : SensorMessageSorter::Trigger(sorter, 0){
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

	_tracker.setProjector(_projector);
	_gmap.setProjector(_projector);

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
	if ((sqrt(displacement.translation().x()*displacement.translation().x() + displacement.translation().y()*displacement.translation().y()) > .3) || rotation.angle() > M_PI_4){
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

	  bool success = _tracker.update(cloud, initial_guess);
	  if (success){
	    global_t = _tracker.globalT();

	    rotation.fromRotationMatrix(global_t.linear());
	    std::cerr << "Result: " << global_t.translation().x() << " " << global_t.translation().y() << " " << rotation.angle() << std::endl;
	  
	    _gmap.addVertex(las);
	    _gmap.addEdge(_gmap.previousVertex(), graphMap().currentVertex(), prevTransf.inverse()*global_t);
	    _gmap.optimize(1);
	    prevTransf = global_t;
	  }
	  else{
	    //Trust the odometry
	    _gmap.addVertex(las);
	    _gmap.addEdge(_gmap.previousVertex(), _gmap.currentVertex(), initial_guess);
	  }
	  	  
	  prevOdom = las->odometry();
      
	  _gmap.findConstraints();
	  _gmap.saveGraph("prueba.g2o");  
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
  
};


int main(int argc, char **argv){
  if (argc<3 || ! strcmp(argv[1],"-h")) {
    printBanner(banner);
    return 0;
  }

  MessageReader reader;
  reader.open(argv[1]);

  double init = getTime();

  //Parameters (fixed by the moment)
  double bpr = 0.2;
  int iterations = 10;
  double inlier_distance = .5;
  double min_correspondences_ratio = 0.3;
  double local_map_clipping_range = 10.0;
  double local_map_clipping_translation_threshold = 5.0;

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

  int max_count = atoi(argv[2]);
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
  idmst->graphMap().saveGraph("prueba.g2o");
  cerr << "Finished" << endl;
}
