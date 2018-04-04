#ifndef MICVISION_LOCATION_H_
#define MICVISION_LOCATION_H_
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// #include <micvision_location/LocationAction.h>
#include <micvision_location/commands.h>
#include <micvision_location/grid_map.h>

#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose2D.h>

#include <queue>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

const double PI_2 = 2*M_PI;

namespace micvision {
typedef std::vector<Eigen::Vector3f> PointCloud;
typedef std::vector<Eigen::Vector2i> PointCloudUV;

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


struct LaserScanSample {
  PointCloudUV point_cloud;
  std::vector<int> indices;
  int min_x, max_x;
  int min_y, max_y;
};

class MicvisionLocation {
 public:
    MicvisionLocation();
    ~MicvisionLocation();

    void mapCallback(const nav_msgs::OccupancyGrid& map);
    void scanCallback(const sensor_msgs::LaserScan& scan);
    bool receiveLocation(
        std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res );
    bool getMap();

    // Inflation radius relative
    void computeCaches();
    void inflateMap();

    LaserScanSample transformPointCloud(const Eigen::Quaternionf& transform);
    void scoreLaserScanSamples();
    double scoreASample(const LaserScanSample& sample,
                        const int x, const int y);

    void debugAPosition(const geometry_msgs::Pose2D &pose);


 private:
    // private function
    void handleLaserScan();

    inline double distanceLookup(const unsigned int mx,
                                 const unsigned int my,
                                 const unsigned int sx,
                                 const unsigned int sy);

    inline signed char costLookup(const unsigned int mx,
                                  const unsigned int my,
                                  const unsigned int sx,
                                  const unsigned int sy);

    void enqueueObstacle(const unsigned int index,
                         const unsigned int x,
                         const unsigned int y);

    bool has_new_map_;
    double inflation_radius_;
    double robot_radius_;
    signed char cost_obstacle_;

    // laserscan relative
    bool handling_lasescan_;
    PointCloud point_cloud_;

    int laserscan_circle_step_;
    int range_step_;   // unit: resolution
    double laserscan_anglar_step_;

    unsigned int cell_inflation_radius_;
    std::vector<LaserScanSample> laserscan_samples_;
    // min and max valid distance for laserscan
    double min_valid_range_, max_valid_range_;

    // best result
    double best_angle_;
    Eigen::Vector2i best_position_;

    GridMap current_map_;

    ros::Publisher position_publisher_;
    ros::Subscriber map_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber debug_position_sub_;
    ros::ServiceServer location_server_;

    ros::ServiceClient get_map_client_;

    // Inflation relative
    signed char **cached_costs_;
    double **cached_distances_;
    unsigned char *inflation_markers_;
    std::priority_queue<CellData> inflation_queue_;

    // for quick search
    std::vector<std::pair<bool, signed char> > inflated_map_data_;
    int width_, height_;
    double resolution_;
};
}  // namespace micvision
#endif  // end MICVISION_LOCATION_H_
