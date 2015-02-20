#include "cloud2d.h"
#include <string>
#include <stdexcept>

//#define _NAN_CHECK_
//#define _GO_PARALLEL_

namespace tsm{

  using namespace std;

  void Cloud2D::add(const Cloud2D& other){
    if (&other == this)
      return;
    size_t k = size();
    resize(k+other.size());
    for (size_t i = 0; i<other.size(); i++){
      //cerr << "size: " << size() << " k+i:" << k+i << endl;
      at(k+i) = other.at(i);
    }
  }


  void Cloud2D::transformInPlace(const Eigen::Isometry2f& T){
    Eigen::Matrix2f R=T.linear();
    for (size_t i=0; i<size(); i++){
      if (at(i).accumulator() <= 0) {
      	cerr << "size: " << size() << endl;
      	cerr << "index: " << i << endl;
      	cerr << "p: " << at(i).point().transpose() << endl;
      	cerr << "n: " << at(i).normal().transpose() << endl;
      	cerr << "cvi: " << at(i).accumulator() << endl;
       	throw std::runtime_error("cum numm");
      }
      at(i).transformInPlace(T);
    }
  }

  void Cloud2D::transform(Cloud2D& other, const Eigen::Isometry2f& T) const{
    other.resize(size());
    Eigen::Matrix2f R=T.linear();
    for (size_t i=0; i<size(); i++){
      other[i]=at(i).transform(T);
    }
  }


#ifdef _DEBUG_MERGE_

#define checkCumVal(x,msg)						\
  if (x.cumulativeValues[x##Idx]<=0){					\
    cerr << "idx: " << x##Idx << endl;					\
      cerr << "p:" << x.pointAccumulators[x##Idx].transpose() << endl;	\
      cerr << "n:" << x.normalAccumulators[x##Idx].transpose() << endl; \
      cerr << "c:" << x.cumulativeValues[x##Idx]<< endl;		\
      throw std::runtime_error(msg);					\
  }
#else
#define checkCumVal(x,msg)
#endif

  void merge(FloatVector& destBuffer, IntVector& destIndices, Cloud2D& dest,
	     FloatVector& srcBuffer, IntVector& srcIndices, Cloud2D& src, 
	     float normalThreshold, float distanceThreshold) {
#ifdef _DEBUG_MERGE_
    cerr << "Merge: " << endl;
    cerr << "destSize: " << dest.size() << " srcSize: " << src.size() << endl;
#endif //_DEBUG_MERGE_
    
    normalThreshold = cos(normalThreshold);
    int newPoints = 0;

    for (size_t c = 0; c < std::min(destBuffer.size(), srcBuffer.size()); ++c) {
      int& srcIdx = srcIndices[c];
      int& destIdx = destIndices[c];

      // add a point if it appars only in the src and are undefined in the dest
      if (srcIdx < 0 || destIdx < 0) {
	if (srcIdx > -1 && destIdx <0)
	  ++newPoints;
	continue;
      }

      checkCumVal(dest, "before dest");
      checkCumVal(src, "before src");

      float destDepth = destBuffer[c];
      float srcDepth = srcBuffer[c];
      //float davg = 0.5 * (destDepth + srcDepth);
      float distance = (destDepth - srcDepth);// / davg;

      // if a new point appears a lot in front an old one add it
      if (distance > distanceThreshold) {
	destIdx = -1;
	++newPoints;
	continue;
      }

      // if a new point appears a lot behind an old replace the old
      if (distance < -distanceThreshold) {
	dest[destIdx] = src[srcIdx];
	srcIdx = -1;
	destIdx = -1;
	continue;
      }

      // if the normals do not match do nothing
      Eigen::Vector2f destNormal = dest[destIdx].normal();
      Eigen::Vector2f srcNormal = src[srcIdx].normal();

      if (!dest[destIdx].isNormalized())
	destNormal /= dest[destIdx].accumulator();

      if (!src[srcIdx].isNormalized())
	srcNormal /= src[srcIdx].accumulator();

      if (destNormal.dot(srcNormal) < normalThreshold) {
	destIdx = -1;
	srcIdx = -1;
	continue;
      }

      // merge the points
      dest[destIdx] += src[srcIdx];
      srcIdx = -1;

      checkCumVal(dest, "merge in bounds");
    }

#ifdef _DEBUG_MERGE_
    cerr << "dest expected final size: " << dest.size()+newPoints << endl;
#endif //_DEBUG_MERGE_

    // recompute all the touched points
    for (size_t c = 0; c < destBuffer.size(); c++) {
      int& destIdx = destIndices[c];

      if (destIdx < 0)
	continue;

      dest[destIdx].normalize();
    }
  
    int k = dest.size();
    dest.resize(dest.size() + newPoints);

    for (size_t c = 0; c < srcBuffer.size(); c++) {
      int& srcIdx = srcIndices[c];

      if (srcIdx < 0)
	continue;

      dest[k] = src[srcIdx];
      k++;
    }
  
#ifdef _DEBUG_MERGE_
    cerr << "expected size: " << dest.size() << " newPoints: " << newPoints << " finalSize: " << k << endl;
#endif //_DEBUG_MERGE_
  }

  struct IndexPair {
    int x, y, index;
    IndexPair(){
      x = y = 0;
      index = -1;
    }

    IndexPair(const Eigen::Vector2f& v, int idx, float ires){
      x = (int) (ires*v.x());
      y = (int) (ires*v.y());
      index = idx;
    }
    bool operator < (const IndexPair& o) const {
      if (x<o.x)
	return true;
      if (x>o.x)
	return false;
      if (y<o.y)
	return true;
      if (y>o.y)
	return false;
      if (index<o.index)
	return true;
      return false;
    }

    bool sameCell( const IndexPair& o) const {
      return x == o.x && y == o.y;
    }
  };

  //! clips to a maxRange around a pose
  void Cloud2D::clip(float range, const Eigen::Isometry2f& pose) {
    Eigen::Isometry2f T = pose.inverse();
    range *=range;
    int k = 0;
    for (size_t i=0; i<size(); i++){
      const RichPoint2D& p = at(i);
      Eigen::Vector2f other_p = T*p.point();
      if (other_p.squaredNorm()<range) {
	at(k) = p;
	k++;
      }
    }
    resize(k);
  }

  void Cloud2D::draw(UnsignedCharImage &img, bool draw_normals, Eigen::Isometry2f T, bool draw_pose_origin) {
    std::vector<cv::Point> pt_to_draw;
    std::vector<cv::Point> normal_to_draw;
    int scale = 55;
    float max_value = std::numeric_limits<float>::min();

    for(Cloud2D::iterator it = this->begin(); it != this->end(); ++it) {
      Eigen::Vector2f pt = T * it->point();
      Eigen::Vector2f n = T.linear() * it->normal();
      float value = std::max(std::fabs(pt.x()), std::fabs(pt.y()));

      if (value > max_value)
	max_value = value + 1;

      pt *= scale * 0.5;
      n *= scale * 0.5;
      pt_to_draw.push_back(cv::Point(pt.x(), pt.y()));
      normal_to_draw.push_back(cv::Point(n.x(), n.y()));
    }
    
    float max_dest_size = std::max(img.rows, img.cols);
    float img_size = std::max(max_value * scale, max_dest_size);

    UnsignedCharImage tmp = cv::Mat::zeros(img_size, img_size, CV_8UC1);
    
    for(size_t i = 0; i < pt_to_draw.size(); ++i) {
      cv::Point& pt = pt_to_draw[i];

      pt.x += img_size * 0.5;
      pt.y += img_size * 0.5;

      cv::Point& normal = normal_to_draw[i];

      if (draw_normals)
	cv::line(
		 tmp,
		 pt,
		 pt + normal,
		 100
		 );

      tmp.at<uchar>(pt.y, pt.x) = 255;
    }
   
    cv::circle(tmp,
	       cv::Point(
			 T.translation().x() * scale * 0.5 + img_size * 0.5,
			 T.translation().y() * scale * 0.5 + img_size * 0.5
			 ),
	       3,
	       150,
	       CV_FILLED);

    if (img.size().area() > 0) {
      float max_cols = std::max(img.cols, tmp.cols);
      cv::Rect roi = cv::Rect(
			      (tmp.cols - img.cols) * 0.5,
			      (tmp.rows - img.rows) * 0.5,
			      img.cols,
			      img.rows);
      tmp(roi) += img;
    }

    img = tmp;
  }
  

  void voxelize(Cloud2D& model, float res) {
    //cerr << "voxelize: " << model.size() << " -> ";
    float ires = 1./res;
    std::vector<IndexPair> voxels(model.size());
    for (int i=0; i<(int)model.size(); i++){
      voxels[i] = IndexPair(model[i].point(), i , ires);
      //cerr << i << " " << model.points[i].transpose() << endl;
      //cerr << voxels[i].x << " " << voxels[i].y << " " << voxels[i].x <<  endl;
    }
    Cloud2D sparseModel;
    sparseModel.resize(model.size());
    std::sort(voxels.begin(), voxels.end());
    int k = -1;
    for (size_t i=0; i<voxels.size(); i++) { 
      IndexPair& pair = voxels[i];
      int idx = pair.index;
      if (k>=0  && voxels[i].sameCell(voxels[i-1])) {
	sparseModel[k] += model[idx];
      } else {
	k++;
	sparseModel[k] = model[idx];
      } 
      //cerr << voxels[i].x << " " << voxels[i].y << " " << voxels[i].x <<  endl;
    }
    sparseModel.resize(k);
    for (size_t i = 0; i<sparseModel.size(); i++) {
      if (sparseModel[i].accumulator() <=0)
	cerr << "Il male sia con te" <<endl;
      sparseModel[i].normalize();
    }
    model = sparseModel;
    //cerr << sparseModel.size() << endl;
  }


}
