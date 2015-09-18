#include "cloud_processor.h"
#include <stdexcept>
using namespace std;
namespace tsm {

    struct IndexPair {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      IndexPair() {
	x = y = 0;
	index = -1;
      }
      
      // Resolution is in cm
      IndexPair(const Eigen::Vector2f& v, int idx, float ires) {
	x = static_cast<int>(ires * v.x());
	y = static_cast<int>(ires * v.y());
	index = idx;
      }

      inline bool operator<(const IndexPair& o) const {
	if (x < o.x)
	  return true;
	if (x > o.x)
	  return false;
	if (y < o.y)
	  return true;
	if (y > o.y)
	  return false;
	if (index < o.index)
	  return true;

	return false;
      }

      inline bool sameCell(const IndexPair& o) const {
	return x == o.x && y == o.y;
      }

      int x, y, index;
    };


  void CloudProcessor::voxelize(Cloud2D& model, float res) {
    Cloud2D sparse_model;
    float ires = 1. / res;

    std::vector<IndexPair> voxels(model.size());

    for (int i = 0; i < model.size(); ++i){
      voxels[i] = IndexPair(model[i].point(), i , ires);
    }
    
    sparse_model.resize(model.size());
    std::sort(voxels.begin(), voxels.end());

    int k = -1;
    for (size_t i = 0; i < voxels.size(); ++i) { 
      IndexPair& pair = voxels[i];
      int idx = pair.index;

      if (k >= 0 && voxels[i].sameCell(voxels[i-1])) {
	sparse_model[k] += model[idx];
      } else {
	sparse_model[++k] = model[idx];
      } 
    }

    sparse_model.resize(k);

    for (size_t i = 0; i < sparse_model.size(); ++i) {
      if (sparse_model[i].accumulator() <= 0)
	throw std::runtime_error("Negative Point Accumulator");

      sparse_model[i].normalize();
    }

    model = sparse_model;
  }

  void CloudProcessor::merge(FloatVector& dest_ranges, IntVector& dest_indices, Cloud2D& dest,
				  FloatVector& src_ranges, IntVector& src_indices, Cloud2D& src,
				  float normal_threshold, float distance_threshold) {
    int new_points = 0;
    normal_threshold = cos(normal_threshold);

    for (size_t c = 0; c < std::min(dest_ranges.size(), src_ranges.size()); ++c) {
      int& src_idx = src_indices[c];
      int& dest_idx = dest_indices[c];

      if (dest_idx<0){
	if (src_idx > -1) {
	  new_points++;
	}
	continue;
      }
      if (src_idx<0)
	continue;


      float dest_depth = dest_ranges[c];
      float src_depth = src_ranges[c];
      float distance = (dest_depth - src_depth); // / 0.5 * (destDepth + srcDepth);

      // if a new point appears a lot in front an old one add it
      if (distance > distance_threshold) {
	// dest point does not need to be renormalized, since we don't touch it
	dest_idx = -1;
	++new_points;
	continue;
      }

      // if a new point appears a lot behind an old replace the old
      if (distance < -distance_threshold) {
	dest[dest_idx] = src[src_idx];
	// invalidate both indices in order to do not consider src point later
	// and do not normalize dest point (we don't need it)
	src_idx = dest_idx = -1;
	continue;
      }

      // if the normals do not match do nothing
      Eigen::Vector2f dest_normal = dest[dest_idx].normal();
      Eigen::Vector2f src_normal = src[src_idx].normal();

      if (!dest[dest_idx].isNormalized())
	dest_normal /= dest[dest_idx].accumulator();

      if (!src[src_idx].isNormalized())
	src_normal /= src[src_idx].accumulator();

      if (dest_normal.dot(src_normal) < normal_threshold) {
	// if normals are not good just do nothing and skip the points
	dest_idx = src_idx = -1;
	continue;
      }

      // otherwise merge the points
      dest[dest_idx] += src[src_idx];
      src_idx = -1;
    }

    // finally recompute all the accumulated points
    for (size_t c = 0; c < dest_ranges.size(); ++c) {
      int& dest_idx = dest_indices[c];

      if (dest_idx < 0)
	continue;

      dest[dest_idx].normalize();
    }
 
    // and add the remaining ones from src 
    int k = dest.size();
    dest.resize(dest.size() + new_points);

    for (size_t c = 0; c < src_ranges.size(); ++c) {
      int& src_idx = src_indices[c];

      if (src_idx < 0)
	continue;

      dest[k] = src[src_idx];
      dest[k].setColor(Eigen::Vector3f(0,0,1));
      k++;
    }
  }
}
