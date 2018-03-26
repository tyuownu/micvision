#ifndef GRID_MAP_H_
#define GRID_MAP_H_

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

class GridMap {
 public:
  void update(nav_msgs::OccupancyGrid grid) {
    occupancy_grid_ = grid;
    map_width_ = occupancy_grid_.info.width;
    map_height_ = occupancy_grid_.info.height;
    ROS_DEBUG("Got new map of size %d x %d", map_width_, map_height_);
  }

  unsigned int getWidth() {return map_width_;}
  unsigned int getHeight() {return map_height_;}
  unsigned int getSize() {return map_width_ * map_height_;}
  double getResolution() {return occupancy_grid_.info.resolution;}
  double getOriginX() {return occupancy_grid_.info.origin.position.x;}
  double getOriginY() {return occupancy_grid_.info.origin.position.y;}

  signed char getLethalCost() {return lethal_cost_;}
  void setLethalCost(signed char c) {lethal_cost_ = c;}

  const nav_msgs::OccupancyGrid& getMap() const {return occupancy_grid_;}

  // Get the array index from the given x,y coordinates
  bool getIndex(const unsigned int x, const unsigned int y, unsigned int &i) {
    if ( x >= map_width_ || y >= map_height_ ) {
      return false;
    }
    i = y * map_width_ + x;
    return true;
  }
  
  // Get the x,y coordinates from the given array index
  bool getCoordinates(unsigned int &x, unsigned int &y, const unsigned int i) {
    if ( i >= map_width_ * map_height_ ) {
      ROS_ERROR("getCoords() failed!");
      return false;
    }
    y = i / map_width_;
    x = i % map_width_;
    return true;
  }

  // Index based methods
  signed char getData(unsigned int index) {
    if ( index < map_width_ * map_height_ )
      return occupancy_grid_.data[index];
    else
      return -1;
  }

  bool setData(unsigned int index, signed char value) {
    if ( index >= map_width_ * map_height_ ) {
      return false;
    }
    occupancy_grid_.data[index] = value;
    return true;
  }

  bool isFree(unsigned int index) {
    signed char value = getData(index);
    if ( value >= 0 && value < lethal_cost_ ) return true;
    return false;
  }

  bool isFrontier(unsigned int index) {
    int y = index / map_width_;
    int x = index % map_width_;

    if ( getData(x-1, y-1) == -1 ) return true;
    if ( getData(x-1, y  ) == -1 ) return true;
    if ( getData(x-1, y+1) == -1 ) return true;
    if ( getData(x  , y-1) == -1 ) return true;
    if ( getData(x  , y+1) == -1 ) return true;
    if ( getData(x+1, y-1) == -1 ) return true;
    if ( getData(x+1, y  ) == -1 ) return true;
    if ( getData(x+1, y+1) == -1 ) return true;

    return false;
  }

  /** Gets indices of all free neighboring cells with given offset */
  std::vector<unsigned int> getFreeNeighbors(unsigned int index,
      int offset = 1) {
    std::vector<unsigned int> neighbors;

    if ( offset < 0 ) offset *= -1;
    int y = index / map_width_;
    int x = index % map_width_;

    for ( int i = -offset; i <= offset; i++ )
      for ( int j = -offset; j <= offset; j++ )
        if ( getIndex(x+i, y+j, index) && isFree(index) )
          neighbors.push_back(index);

    return neighbors;
  }

  /** Gets indices of all neighboring cells */
  std::vector<unsigned int> getNeighbors(unsigned int index,
      bool diagonal = false) {
    std::vector<unsigned int> neighbors;

    int y = index / map_width_;
    int x = index % map_width_;
    unsigned int i;
    if ( getIndex(x-1, y,   i) ) neighbors.push_back(i);
    if ( getIndex(x+1, y,   i) ) neighbors.push_back(i);
    if ( getIndex(x,   y-1, i) ) neighbors.push_back(i);
    if ( getIndex(x,   y+1, i) ) neighbors.push_back(i);

    if ( diagonal ) {
      if ( getIndex(x-1, y-1, i) ) neighbors.push_back(i);
      if ( getIndex(x-1, y+1, i) ) neighbors.push_back(i);
      if ( getIndex(x+1, y-1, i) ) neighbors.push_back(i);
      if ( getIndex(x+1, y+1, i) ) neighbors.push_back(i);
    }
    return neighbors;
  }

  // Coordinate based methods
  signed char getData(int x, int y) {
    if ( x < 0 ||x >= (int)map_width_ || y < 0 || y >= (int)map_height_ )
      return 100;
    else
      return occupancy_grid_.data[y*map_width_ + x];
  }

  bool setData(int x, int y, signed char value) {
    if ( x < 0 ||x >= (int)map_width_ || y < 0 || y >= (int)map_height_ ) {
      return false;
    }
    occupancy_grid_.data[y*map_width_ + x] = value;
    return true;
  }

  bool isFree(int x, int y) {
    signed char value = getData(x, y);
    if ( value >= 0 && value < lethal_cost_ ) return true;
    return false;
  }

  float getBoundaryX() {
    return getOriginX() + getWidth() * getResolution();
  }

  float getBoundaryY() {
    return getOriginY() + getHeight() * getResolution();
  }

 private:
  nav_msgs::OccupancyGrid occupancy_grid_;
  unsigned int map_width_;
  unsigned int map_height_;
  signed char lethal_cost_;
};

#endif  // end GRID_MAP_H_
