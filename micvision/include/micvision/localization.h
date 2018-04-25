#ifndef MICVISION_LOCALIZATION_H_
#define MICVISION_LOCALIZATION_H_

// ros
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose2D.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

// micvision_localization
#include <micvision/commands.h>
#include <micvision/grid_map.h>
#include <micvision/LocalizationConfig.h>


// std
#include <queue>
#include <vector>

// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace micvision {
constexpr double PI_2 = 2*M_PI;
constexpr double RADIAN_PRE_DEGREE = M_PI/180;
typedef std::vector<Eigen::Vector3f> PointCloud;
typedef std::vector<Pixel> PointCloudUV;
typedef micvision::LocalizationConfig Config;
typedef dynamic_reconfigure::Server<Config> LocalizationConfigServer;
typedef dynamic_reconfigure::Server<Config>::CallbackType CallbackType;

struct CellData {
 public:
  CellData() = delete;
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

class MicvisionLocalization {
 public:
    MicvisionLocalization();
    ~MicvisionLocalization();

    void debugAPosition(const geometry_msgs::Pose2D &pose);

    void tracking();
 private:
    // private function
    void mapCallback(const nav_msgs::OccupancyGrid& map);
    void scanCallback(const sensor_msgs::LaserScan& scan);
    bool receiveLocalization(std_srvs::Trigger::Request &req,
                         std_srvs::Trigger::Response &res);
    bool getMap();

    // Inflation radius relative
    void computeCaches();
    void inflateMap();

    LaserScanSample transformPointCloud(const Eigen::Quaternionf& transform);
    void scoreLaserScanSamples();
    double scoreASample(const LaserScanSample& sample,
                        const int x, const int y);
    void handleLaserScan();

    inline double distanceLookup(const unsigned int mx,
                                 const unsigned int my,
                                 const unsigned int sx,
                                 const unsigned int sy) const;

    inline signed char costLookup(const unsigned int mx,
                                  const unsigned int my,
                                  const unsigned int sx,
                                  const unsigned int sy) const;

    void enqueueObstacle(const unsigned int index,
                         const unsigned int x,
                         const unsigned int y);

    void reconfigureCB(Config &config, uint32_t level);

    bool validPosition(const int uv, const int index);

    std::vector<Pixel> bresenham(const Pixel &start, const Pixel &end);

    void odomCallback(const nav_msgs::Odometry &odom);


 private:
    bool has_new_map_ = true;
    double inflation_radius_ = 0.3;
    unsigned int cell_inflation_radius_;
    /// Not used
    double robot_radius_ = 0.2;
    /// to define the obstacle cost
    /// TODO:
    /// Maybe we can translate this to double 1.0?
    signed char cost_obstacle_ = 100;

    // laserscan relative
    /// If we are in handling the laserscan
    /// we should not update point_cloud_
    bool handling_lasescan_ = false;
    /// point in a scan, this may have bugs for real lidar
    /// TODO: fix on real lidar
    PointCloud point_cloud_;

    /// the step for we construct point_cloud_
    int laserscan_circle_step_ = 6;
    int range_step_ = 3;   // unit: resolution
    int laserscan_anglar_step_ = 6;  // unit: degree

    std::vector<LaserScanSample> laserscan_samples_;
    // min and max valid distance for laserscan
    double min_valid_range_ = 0.0f, max_valid_range_ = 10.0f;

    // best result
    double best_angle_;
    Eigen::Vector2i best_position_;

    /// the inflated map we got
    GridMap current_map_;
    // for quick search
    std::vector<std::pair<bool, double> > inflated_map_data_;
    int width_, height_;
    double resolution_;

    ros::Publisher position_publisher_;
    ros::Subscriber map_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber debug_position_sub_;
    ros::ServiceServer localization_server_;

    ros::ServiceClient get_map_client_;

    // Inflation relative
    signed char **cached_costs_ = nullptr;
    double **cached_distances_ = nullptr;
    unsigned char *inflation_markers_ = nullptr;
    std::priority_queue<CellData> inflation_queue_;

    /// quick score
    /// we using quick score to define this point is reseanable
    int quick_score_num_ = 8;
    bool quick_score_ = true;

    LocalizationConfigServer *dynamic_srv_;

    tf::TransformListener tf_listener_;
    std::string map_frame_;
    std::string robot_frame_;
    double tracking_frequency_;
    double current_position_score_ = 0.0f;

    // odom relative
    bool big_angle_twist_ = false;
};
}  // namespace micvision
#endif  // end MICVISION_LOCALIZATION_H_
