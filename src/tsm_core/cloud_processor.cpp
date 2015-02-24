#include "cloud_processor.h"

namespace tsm {
  void CloudProcessor::merge(FloatVector& dest_ranges, IntVector& dest_indices, Cloud2D& dest,
				  FloatVector& src_ranges, IntVector& src_indices, Cloud2D& src,
				  float normal_threshold, float distance_threshold) {
    int new_points = 0;
    normal_threshold = cos(normal_threshold);

    for (size_t c = 0; c < std::min(dest_ranges.size(), src_ranges.size()); ++c) {
      int& src_idx = src_indices[c];
      int& dest_idx = dest_indices[c];

      if (src_idx < 0 || dest_idx < 0) {
	// add a point if it appears only in the src and are undefined in the dest
	if (src_idx > -1 && dest_idx < 0) {
	  ++new_points;
	}

	continue;
      }

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
      k++;
    }
  }
}
