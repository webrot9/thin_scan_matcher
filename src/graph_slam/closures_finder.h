
#include "closure_buffer.h"
#include "closure_checker.h"
#include "vertices_finder.h"
#include "graph_manipulator.h"
#include "simple_graph.h"

#include "tsm_core/projector2d.h"
#include "tsm_core/tracker.h"
#include "tsm_core/cloud_processor.h"

class ClosuresFinder{
public:
  ClosuresFinder();

  void findClosures();

  inline SimpleGraphMap* graphMap(){return _gmap;}
  inline void setGraphMap(SimpleGraphMap* gmap){_gmap = gmap;}

  inline tsm::Projector2D* projector(){return _projector;}
  inline void setProjector(tsm::Projector2D* projector){_projector = projector;}

  inline int windowLoopClosure(){return _windowLoopClosure;}
  inline void setWindowLoopClosure(int windowLoopClosure){_windowLoopClosure = windowLoopClosure;}
  inline float inlierThreshold(){return _inlierThreshold;}
  inline void setInlierThreshold(int inlierThreshold){_inlierThreshold = inlierThreshold;}
  inline int minInliers(){return _minInliers;}
  inline void setMinInliers(int minInliers){_minInliers = minInliers;}


  tsm::Cloud2D* cloudFromVertex(tsm::Projector2D* projector, OptimizableGraph::Vertex* vertex);
  tsm::Cloud2D* cloudFromVSet(tsm::Projector2D* projector, OptimizableGraph::VertexSet& vset, OptimizableGraph::Vertex* refVertex);
  tsm::Cloud2D* cloudFromVSet1(tsm::Projector2D* projector, OptimizableGraph::VertexSet& vset, OptimizableGraph::Vertex* refVertex);

protected:
  void checkCovariance(OptimizableGraph::VertexSet& vset);
  void checkClosures();

  int _windowLoopClosure;
  float _inlierThreshold;
  int _minInliers;
    
  SimpleGraphMap* _gmap;
  tsm::Projector2D* _projector;
  ClosureBuffer _closures;
};
