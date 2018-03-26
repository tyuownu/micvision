#ifndef MICVISION_LOCATION_H_
#define MICVISION_LOCATION_H_
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <micvision_location/LocationAction.h>
#include <micvision_location/commands.h>
#include <micvision_location/grid_map.h>

#include <queue>

namespace micvision {
typedef actionlib::SimpleActionServer<micvision_location::LocationAction> LocationActionServer;

struct CellData {
 public:
  CellData(double d, unsigned int i, unsigned int x, unsigned int y):
      distance(d), index(i), x(x), y(y) {}
  double distance;
  unsigned int index, x, y;

  friend bool operator<(const CellData &c1, const CellData &c2) {
    return c1.distance > c2.distance;
  }
};

class MicvisionLocation {
 public:
    MicvisionLocation();
    ~MicvisionLocation();

    void mapCallback(const nav_msgs::OccupancyGrid& map);
    void scanCallback(const sensor_msgs::LaserScan& scan);
    void receiveLocationGoal(
        const micvision_location::LocationGoal::ConstPtr &goal);
    bool getMap();

    // Inflation radius relative
    void computeCaches();
    void inflateMap();
    void enqueueObstacle(unsigned int index, unsigned int x, unsigned int y);
    inline double distanceLookup(unsigned int mx, unsigned int my,
                                 unsigned int sx, unsigned int sy);
    inline signed char costLookup(unsigned int mx, unsigned int my,
                                  unsigned int sx, unsigned int sy);

 private:
    bool has_new_map_;
    double inflation_radius_;
    double robot_radius_;
    signed char cost_obstacle_;

    unsigned int cell_inflation_radius_;

    GridMap current_map_;

    ros::Publisher position_publisher_;
    ros::Subscriber map_sub_;
    ros::Subscriber scan_sub_;
    LocationActionServer *location_action_server_;

    ros::ServiceClient get_map_client_;

    // Inflation relative
    signed char **cached_costs_;
    double **cached_distances_;
    unsigned char *inflation_markers_;
    std::priority_queue<CellData> inflation_queue_;
};
}  // namespace micvision
#endif  // end MICVISION_LOCATION_H_
