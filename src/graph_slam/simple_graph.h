#ifndef _SIMPLE_GRAPH_H_
#define _SIMPLE_GRAPH_H_

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
#include "g2o/types/data/vertex_ellipse.h"

// graph_slam
#include "graph_manipulator.h"

// txt_io
#include "txt_io/laser_message.h"

using namespace txt_io;
using namespace g2o;

class SimpleGraphMap{
public:
  SimpleGraphMap();
  
  VertexSE2* createVertex(LaserMessage* las_msg);
  void addVertex(LaserMessage* las_msg);

  EdgeSE2* createEdge(VertexSE2* v1, VertexSE2* v2, Eigen::Isometry2f rel_transf, Eigen::Matrix3d inf);
  void addEdge(VertexSE2* v1, VertexSE2* v2, Eigen::Isometry2f rel_transf, Eigen::Matrix3d inf);
  
  void optimize(int nrunnings);

  RobotLaser* findLaserData(OptimizableGraph::Vertex* v);
  void updateLaserData();

  VertexEllipse* findEllipseData(OptimizableGraph::Vertex* v);
  void updateEllipseData();

  bool saveGraph(const char *filename);

  inline SparseOptimizer* graph() {return _graph;}
  inline VertexSE2* currentVertex() {return _currentVertex;}
  inline VertexSE2* previousVertex() {return _previousVertex;}

protected:
  SparseOptimizer * _graph;
  int _currentId;
  VertexSE2 *_currentVertex, *_previousVertex;
};

#endif
