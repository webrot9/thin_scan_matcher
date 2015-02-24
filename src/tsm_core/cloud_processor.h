#pragma once

#include "defs.h"
#include "cloud2d.h"

namespace tsm {
  class CloudProcessor {
  public:
    CloudProcessor() {}
    ~CloudProcessor() {}

    //! Merges src in dest:
    //! it requires the index image of dest and of src, seen from the same point
    //! and also the depth buffers (ranges).
    //! The clouds should be aligned, then
    //! points that are closer than distanceThreshold are merged based on the scaling values
    //! if the normals are compatible
    void merge(
	       FloatVector& dest_ranges,
	       IntVector& dest_indices,
	       Cloud2D& dest,
	       FloatVector& src_ranges,
	       IntVector& src_indices,
	       Cloud2D& src, 
	       float normal_threshold = 1,
	       float distance_threshold = 0.2
	       );

  };
}
