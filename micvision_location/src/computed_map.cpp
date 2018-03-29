#include <micvision_location/computed_map.h>

namespace micvision {
ComputedMap::ComputedMap(GridMap map, const int width)
    : wide_width(map.getWidth() + width - 1),
      wide_height(map.getHeight() + width - 1),
      data(wide_width * wide_height) {
  const int stride = wide_width;
  std::vector<signed char> values(wide_width*map.getHeight());

  for ( int y = 0; y != map.getHeight(); ++y ) {
    Windows current_values;
    current_values.AddValue(map.getRawData(0, y));

    for ( int x = -width + 1; x != 0; ++x ) {
      values[x+width-1+y*stride] = current_values.GetMaximum();
    }
    for ( int x = 0; x < map.getWidth() - width; ++x ) {
      values[x+width-1+y*stride] = current_values.GetMaximum();
      current_values.RemoveValue(map.getRawData(x, y));
      current_values.AddValue(map.getRawData(x+width, y));
    }

    for ( int x = map.getWidth() - width; x != map.getWidth(); ++x ) {
      values[x+width-1+y*stride] = current_values.GetMaximum();
      current_values.RemoveValue(map.getRawData(x, y));
    }
  }

  for ( int x = 0; x != wide_width; ++x ) {
    Windows current_values;
    current_values.AddValue(values[x]);
    for ( int y = -width + 1; y != 0; ++y ) {
      data[x+(y+width-1)*stride] = current_values.GetMaximum();
      if ( y + width < map.getHeight() ) {
        current_values.AddValue( values[x + (y+width) * stride] );
      }
    }

    for ( int y = 0; y < map.getHeight() - width; ++y ) {
      data[x+(y+width-1)*stride] = current_values.GetMaximum();
      current_values.RemoveValue(values[x+y*stride]);
      current_values.AddValue(values[x+(y+width)*stride]);
    }

    for ( int y = map.getHeight()-width; y != map.getHeight(); ++y ) {
      data[x+(y+width-1)*stride] = current_values.GetMaximum();
      current_values.RemoveValue(values[x+y*stride]);
    }
  }
}

signed char ComputedMap::GetValue(const int &x, const int &y) const {
  return data[x + y * wide_width];
}

ComputedMapStack::ComputedMapStack(const GridMap &map) {
  const int depth = 4;
  const int max_width = 1 << (depth - 1);  // 4 for max depth
  computed_maps_.reserve(depth);

  for ( int i = 0; i != depth; ++i ) {
    const int width = 1 << i;
    computed_maps_.emplace_back(map, width);

    char fn[4096];
    sprintf(fn, "/home/tyu/__%d.txt", i);
    FILE *fp = fopen(fn, "w");
    int width_ = Get(i).wide_width;
    int height_ = Get(i).wide_height;
    for ( int v = 0; v < height_; v++ ) {
      for ( int u = 0; u < width_; u++ ) {
        fprintf(fp, "%d ", Get(i).data[u + width_*v]);
      }
      fprintf(fp, "\n");
    }
    fclose(fp);
  }

}
}
