#include "simple_graph.h"

typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

SimpleGraphMap::SimpleGraphMap(){
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
}
  
VertexSE2* SimpleGraphMap::createVertex(LaserMessage* las_msg){
  VertexSE2* v = new VertexSE2;

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
    
  // Add ellipse data information
  VertexEllipse *ellipse = new VertexEllipse;
  Matrix3f cov = Matrix3f::Zero(); //last vertex has zero covariance
  ellipse->setCovariance(cov);
  v->addUserData(ellipse);

  return v;
}

void SimpleGraphMap::addVertex(LaserMessage* las_msg){
  VertexSE2* v = createVertex(las_msg);
  v->setId(_currentId);

  std::cerr << "Adding Vertex: " << v->estimate().translation().x() << " " << v->estimate().translation().y() << " " << v->estimate().rotation().angle() << std::endl;
  _graph->addVertex(v);
  _currentId++;
  _previousVertex = _currentVertex;
  _currentVertex = v;
}

EdgeSE2* SimpleGraphMap::createEdge(VertexSE2* v1, VertexSE2* v2, Eigen::Isometry2f rel_transf, Eigen::Matrix3d inf){
  SE2 displacement(rel_transf.cast<double>());

  EdgeSE2 *e = new EdgeSE2;
  e->vertices()[0] = v1;
  e->vertices()[1] = v2;
  e->setMeasurement(displacement);
  e->setInformation(inf);

  return e;
}

void SimpleGraphMap::addEdge(VertexSE2* v1, VertexSE2* v2, Eigen::Isometry2f rel_transf, Eigen::Matrix3d inf){
  EdgeSE2 *e = createEdge(v1,v2,rel_transf,inf);

  std::cerr << "Adding Edge between : " << v1->id() << " and " << v2->id() << ". Displacement: " << e->measurement().translation().x() << " " << e->measurement().translation().y() << " " << e->measurement().rotation().angle() << std::endl;
  _graph->addEdge(e);
}
  
void SimpleGraphMap::optimize(int nrunnings){
  _graph->initializeOptimization();
  _graph->optimize(nrunnings);

  //Update vertex data
  updateLaserData();
  updateEllipseData();

}

RobotLaser* SimpleGraphMap::findLaserData(OptimizableGraph::Vertex* v){
  HyperGraph::Data* d = v->userData();

  while (d){
    RobotLaser* robotLaser = dynamic_cast<RobotLaser*>(d);
    if (robotLaser){
      return robotLaser;
    }else{
      d = d->next();
    }
  }

  return 0;
}

void SimpleGraphMap::updateLaserData(){
  for (OptimizableGraph::VertexIDMap::const_iterator it = _graph->vertices().begin(); it != _graph->vertices().end(); ++it) {
    VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);
    RobotLaser* robotLaser = findLaserData(v);
    if (robotLaser) 
      robotLaser->setOdomPose(v->estimate());
  }
}

VertexEllipse* SimpleGraphMap::findEllipseData(OptimizableGraph::Vertex* v){
  HyperGraph::Data* d = v->userData();
  while (d){
    VertexEllipse* ellipse = dynamic_cast<VertexEllipse*>(d);
    if (ellipse){
      return ellipse;
    }else{
      d = d->next();
    }
  }
  return 0;
}

void SimpleGraphMap::updateEllipseData(){
  //Covariance computation
  CovarianceEstimator ce(_graph);
  OptimizableGraph::VertexSet vset;
  for (OptimizableGraph::VertexIDMap::iterator it=_graph->vertices().begin(); it!=_graph->vertices().end(); ++it) {
    OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*) (it->second);
    vset.insert(v);
  }
  ce.setVertices(vset);
  ce.setGauge(_currentVertex);
  ce.compute();
  
  for (OptimizableGraph::VertexIDMap::iterator it=_graph->vertices().begin(); it!=_graph->vertices().end(); ++it) {
    VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);
    VertexEllipse* ellipse = findEllipseData(v);
    if (ellipse && (v != currentVertex())){
      MatrixXd PvX = ce.getCovariance(v);
      Matrix3d Pv = PvX;
      Matrix3f Pvf = Pv.cast<float>();
      ellipse->setCovariance(Pvf);
      ellipse->clearMatchingVertices();
    }else {
      if(ellipse && v == currentVertex()){
	ellipse->clearMatchingVertices();
	for (size_t i = 0; i<ellipse->matchingVerticesIDs().size(); i++){
	  int id = ellipse->matchingVerticesIDs()[i];
	  VertexSE2* vid = dynamic_cast<VertexSE2*>(_graph->vertex(id));
	  SE2 relativetransf = _currentVertex->estimate().inverse() * vid->estimate();
	  ellipse->addMatchingVertex(relativetransf.translation().x(), relativetransf.translation().y());
	}
      }
    }
  }
}

bool SimpleGraphMap::saveGraph(const char *filename){
  return _graph->save(filename);
}
