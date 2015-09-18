
#include "closures_finder.h"


struct MatcherResult {
  MatcherResult(const Eigen::Isometry2f& transformation, const double& score, 
		const Eigen::Matrix3f& informationMatrix = Eigen::Matrix3f::Identity()){
    this->transformation = transformation;
    this->score = score;
    this->informationMatrix = informationMatrix;
  }
  Eigen::Isometry2f transformation;
  double score;
  Eigen::Matrix3f informationMatrix;
};

struct MatcherResultScoreComparator {
  bool operator() (const MatcherResult& mr1, const MatcherResult& mr2){
    return mr1.score>mr2.score;
  } 
};

tsm::Cloud2D* ClosuresFinder::cloudFromVertex(tsm::Projector2D* projector, OptimizableGraph::Vertex* vertex){
  tsm::Cloud2D* vcloud = 0;

  RobotLaser* laserv = _gmap->findLaserData(vertex);
  if (laserv){
    vcloud = new tsm::Cloud2D();
    tsm::FloatVector floatranges(laserv->ranges().begin(), laserv->ranges().end());
    projector->unproject(*vcloud, floatranges);
  }
  return vcloud;
}

tsm::Cloud2D* ClosuresFinder::cloudFromVSet1(tsm::Projector2D* projector, OptimizableGraph::VertexSet& vset, OptimizableGraph::Vertex* refVertex){
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
  return finalCloud;
}

tsm::Cloud2D* ClosuresFinder::cloudFromVSet(tsm::Projector2D* projector, OptimizableGraph::VertexSet& vset, OptimizableGraph::Vertex* refVertex){
  tsm::CloudProcessor cloud_processor;
  tsm::Cloud2D* finalCloud = new tsm::Cloud2D();
  
  VertexSE2* refVertexSE2 = dynamic_cast<VertexSE2*>(refVertex);
  for (OptimizableGraph::VertexSet::iterator it = vset.begin(); it != vset.end(); it++){

    VertexSE2 *vertex = dynamic_cast<VertexSE2*>(*it);

    tsm::Cloud2D* vcloud = cloudFromVertex(projector, vertex);
    
    if (vcloud){
      SE2 reltransf = refVertexSE2->estimate().inverse()*vertex->estimate();
      Eigen::Isometry2d reltransf2d = reltransf.toIsometry();
      vcloud->transformInPlace(reltransf2d.cast<float>());
      
      tsm::FloatVector current_ranges, reference_ranges;
      tsm::IntVector current_indices, reference_indices;
      
      projector->project(reference_ranges, reference_indices, Eigen::Isometry2f::Identity(), *finalCloud);
      projector->project(current_ranges, current_indices, Eigen::Isometry2f::Identity(), *vcloud);
      
      cloud_processor.merge(reference_ranges, reference_indices, *finalCloud,
			    current_ranges, current_indices, *vcloud,
			     1.f, 0.5f);
    }
  }
  return finalCloud;
}

ClosuresFinder::ClosuresFinder(){
  _windowLoopClosure = 10;
  _inlierThreshold = 3.0;
  _minInliers = 6;
  
  _projector = 0;
  _gmap = 0;
}

void ClosuresFinder::findClosures(){
  if (! _gmap || !_projector)
    throw std::runtime_error("Cannot find constraints without a map and a projector");

  cerr << "\nFinding constraints"  << endl;
  VerticesFinder vf(_gmap->graph());
    
  OptimizableGraph::VertexSet vset;
  vf.findVerticesScanMatching(_gmap->currentVertex(), vset);
    
  checkCovariance(vset);
  std::set<OptimizableGraph::VertexSet> setOfVSet;
  vf.findSetsOfVertices(vset, setOfVSet);

  OptimizableGraph::EdgeSet loopClosingEdges;
  for (std::set<OptimizableGraph::VertexSet>::iterator it = setOfVSet.begin(); it != setOfVSet.end(); it++) {
    
    OptimizableGraph::VertexSet myvset = *it;
    
    //Update ellipse data for current vertex
    VertexEllipse* ellipse = _gmap->findEllipseData(_gmap->currentVertex());
    if (ellipse){
      for (OptimizableGraph::VertexSet::iterator itv = myvset.begin(); itv != myvset.end(); itv++){
	VertexSE2 *vertex = (VertexSE2*) *itv;
	SE2 relativetransf = _gmap->currentVertex()->estimate().inverse() * vertex->estimate();
	ellipse->addMatchingVertex(relativetransf.translation().x(), relativetransf.translation().y());
	ellipse->addMatchingVertexID(vertex->id());
      }
    }

    OptimizableGraph::Vertex* closestV = vf.findClosestVertex(myvset, _gmap->currentVertex()); 

    if (abs(_gmap->currentVertex()->id() - closestV->id()) > 10){
	
      //try to match
      //Parameters (fixed by the moment)
      double bpr = 1;
      int iterations = 10;
      double inlier_distance = 2.;
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

      tsm::Cloud2D* cvset = cloudFromVSet1(_projector, myvset, closestV);
      tsm::Cloud2D* cvertex = cloudFromVertex(_projector, _gmap->currentVertex());

      /*
      tsm::RGBImage img;
      cvset->draw(img, cv::Vec3b(0,255,0), false, Eigen::Isometry2f::Identity(), false, 20);
      cv::imshow( "Cloud with merge", img);

      tsm::RGBImage img2;
      tsm::Cloud2D* cvset2 = cloudFromVSet1(_projector, myvset, closestV);
      cvset2->draw(img2, cv::Vec3b(0,255,0), false, Eigen::Isometry2f::Identity(), false, 20);
      cv::imshow( "Cloud without merge", img2);
      
      cv::waitKey(0);
      */
      tracker.setReference(cvset);
      tracker.setCurrent(cvertex);

      VertexSE2* refv = (VertexSE2*) closestV;
      VertexSE2* curv = (VertexSE2*) _gmap->currentVertex();
	
      std::vector<MatcherResult> mresvec;

      for (OptimizableGraph::VertexSet::iterator itv = myvset.begin(); itv != myvset.end(); itv++){
	VertexSE2* v = (VertexSE2*) (*itv);

	SE2 initguessSE2 = refv->estimate().inverse()*v->estimate();
	Eigen::Isometry2d initguess2d = initguessSE2.toIsometry();
	  
	Eigen::Rotation2Dd rotation(0); 
	rotation.fromRotationMatrix(initguess2d.linear());
	//std::cerr << endl << "Initial guess: " << initguess2d.translation().x() << " " << initguess2d.translation().y() << " " << rotation.angle() << std::endl;

	bool success = tracker.match(initguess2d.cast<float>());
	if (success){
	  MatcherResult mr(tracker.solver()->T(), tracker.inliersRatio());
	  mresvec.push_back(mr);
	}

	Eigen::Rotation2Dd rotationPI(M_PI); 
	initguess2d.linear() = initguess2d.linear()*rotationPI;
	rotation.fromRotationMatrix(initguess2d.linear());
	success = tracker.match(initguess2d.cast<float>());
	if (success){
	  MatcherResult mr(tracker.solver()->T(), tracker.inliersRatio());
	  mresvec.push_back(mr);
	}
      }

      if (mresvec.size()){
	MatcherResultScoreComparator comp;
	std::sort(mresvec.begin(), mresvec.end(), comp);
	  
	//Introducing results in closure checker
	//for (std::vector<MatcherResult>::iterator itmr = mresvec.begin(); itmr != mresvec.end(); itmr++){
	//Eigen::Isometry2f res = (*itmr).transformation;
	Eigen::Isometry2f bestResult = mresvec[0].transformation;
	cerr << "BEST SCORE:" << mresvec[0].score << endl;
	cerr << "Loop closure between " << refv->id() << " and " << curv->id() << " accepted." << endl;
	    
	Matrix3d inf =  1000 * Matrix3d::Identity();
	inf(2,2) = 10000;
	EdgeSE2 *ne = _gmap->createEdge(refv, curv, bestResult, inf);
	loopClosingEdges.insert(ne);
	//}
      }else
	cerr << "Loop closure between " << refv->id() << " and " << curv->id() << " rejected." << endl;
	
    }
  }
  if (loopClosingEdges.size()){
    _closures.addEdgeSet(loopClosingEdges);
    _closures.addVertex(_gmap->currentVertex());
  }
  checkClosures();
}

void ClosuresFinder::checkCovariance(OptimizableGraph::VertexSet& vset){
  ///////////////////////////////////
  // we need now to compute the marginal covariances of all other vertices w.r.t the newly inserted one

  CovarianceEstimator ce(_gmap->graph());
  ce.setVertices(vset);
  ce.setGauge(_gmap->currentVertex());
  ce.compute();

  OptimizableGraph::VertexSet tmpvset = vset;
  for (OptimizableGraph::VertexSet::iterator it = tmpvset.begin(); it != tmpvset.end(); it++){
    VertexSE2 *vertex = (VertexSE2*) *it;
    
    MatrixXd Pv = ce.getCovariance(vertex);
    Matrix2d Pxy; Pxy << Pv(0,0), Pv(0,1), Pv(1,0), Pv(1,1);
    SE2 delta = vertex->estimate().inverse() * _gmap->currentVertex()->estimate();	
    Vector2d hxy (delta.translation().x(), delta.translation().y());
    double perceptionRange =1;
    if (hxy.x()-perceptionRange>0) 
      hxy.x() -= perceptionRange;
    else if (hxy.x()+perceptionRange<0)
      hxy.x() += perceptionRange;
    else
      hxy.x() = 0;

    if (hxy.y()-perceptionRange>0) 
      hxy.y() -= perceptionRange;
    else if (hxy.y()+perceptionRange<0)
      hxy.y() += perceptionRange;
    else
      hxy.y() = 0;
    
    double d2 = hxy.transpose() * Pxy.inverse() * hxy;
    if (d2 > 5.99)
      vset.erase(*it);
 
  }
}

void ClosuresFinder::checkClosures(){
  LoopClosureChecker lcc;
  if (_closures.checkList(_windowLoopClosure)){
    cout << endl << "Loop Closure Checking." << endl;
    lcc.init(_closures.vertices(), _closures.edgeSet(), _inlierThreshold);
    lcc.check();
      
    cout << "Best Chi2 = " << lcc.chi2() << endl;
    cout << "Inliers = " << lcc.inliers() << endl;
    if (lcc.inliers() >= _minInliers){
      LoopClosureChecker::EdgeDoubleMap results = lcc.closures();
      cout << "Results:" << endl;
      for (LoopClosureChecker::EdgeDoubleMap::iterator it= results.begin(); it!= results.end(); it++){
	EdgeSE2* e = (EdgeSE2*) (it->first);
	cout << "Edge from: " << e->vertices()[0]->id() << " to: " << e->vertices()[1]->id() << ". Chi2 = " << it->second <<  endl;

	if (it->second < _inlierThreshold){
	  cout << "Is an inlier. Adding to Graph" << endl;
	  _gmap->graph()->addEdge(e);
	}
      }
    }
  }
  _closures.updateList(_windowLoopClosure);

}

