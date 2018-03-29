#ifndef MICVISION_LOCATION_COMPUTED_MAP_H_
#define MICVISION_LOCATION_COMPUTED_MAP_H_
#include <micvision_location/grid_map.h>

#include <deque>

namespace micvision {
class Windows {
 public:
  void AddValue(const signed char value) {
    while ( !non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back() ) {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }

  void RemoveValue(const signed char value) {
    if ( value == non_ascending_maxima_.front() ) {
      non_ascending_maxima_.pop_front();
    }
  }

  signed char GetMaximum() const {
    return non_ascending_maxima_.front();
  }

 private:
  std::deque<signed char> non_ascending_maxima_;
};

struct ComputedMap {
  ComputedMap(GridMap map, int width);

  signed char GetValue( const int &x, const int &y ) const;

  int wide_width;
  int wide_height;

  std::vector<signed char> data;
};

class ComputedMapStack {
 public:
  ComputedMapStack(const GridMap& map);
  int max_depth() const { return computed_maps_.size() - 1; }
  const ComputedMap& Get(int index) {
    return computed_maps_[index];
  }

 private:
  std::vector<ComputedMap> computed_maps_;
};
};
#endif  // end MICVISION_LOCATION_COMPUTED_MAP_H_
