#include <nav_msgs/GridCells.h>

#include <micvision/exploration.h>

#include <set>
#include <map>
#include <mutex>
#include <utility>

namespace micvision {
using Queue = std::multimap<double, unsigned int>;
using Entry = std::pair<double, unsigned int>;

constexpr int EXPLORATION_TARGET_SET = 1;
constexpr int EXPLORATION_FINISHED = 2;
constexpr int EXPLORATION_WAITING = 3;
constexpr int EXPLORATION_FAILED = 4;

// mutexs
static std::mutex map_mutex;
static std::mutex scan_mutex;

int findExplorationTarget(GridMap* map,
                          const unsigned int start,
                          unsigned int &goal) {
  // Create some workspace for the wavefront algorithm
  const unsigned int map_size = map->getSize();
  double* plan = new double[map_size];
  for ( unsigned int i = 0; i < map_size; i++ ) {
    plan[i] = -1;
  }

  // Initialize the queue with the robot position
  Queue queue;
  Entry start_point(0.0, start);
  queue.insert(start_point);
  plan[start] = 0;

  Queue::iterator next;
  double distance;
  const double linear = map->getResolution();
  bool found_frontier = false;
  int cell_count = 0;

  // Do full search with weightless Dijkstra-Algorithm
  while ( !queue.empty() ) {
    cell_count++;
    // Get the nearest cell from the queue
    next = queue.begin();
    distance = next->first;
    const unsigned int index = next->second;
    queue.erase(next);

    // Add all adjacent cells
    if ( map->isFrontier(index) ) {
      // We reached the border of the map, which is unexplored terrain as well:
      found_frontier = true;
      goal = index;
      break;
    } else {
      unsigned int ind[4];

      ind[0] = index - 1;                // left
      ind[1] = index + 1;                // right
      ind[2] = index - map->getWidth();  // up
      ind[3] = index + map->getWidth();  // down

      for ( unsigned int it = 0; it < 4; it++ ) {
        const unsigned int i = ind[it];
        if ( map->isFree(i) && plan[i] == -1 ) {
          queue.insert(Entry(distance+linear, i));
          plan[i] = distance+linear;
        }
      }
    }
  }

  ROS_DEBUG("Checked %d cells.", cell_count);
  delete[] plan;

  if ( found_frontier ) {
    return EXPLORATION_TARGET_SET;
  } else {
    if ( cell_count > 50 )
      return EXPLORATION_FINISHED;
    else
      return EXPLORATION_FAILED;
  }
}

// to calculate two Points' distance
int PixelDistance(const Pixel& p1, const Pixel& p2) {
  return sqrt(pow((p1(0)-p2(0)), 2) + pow((p1(1)-p2(1)), 2));
}
MicvisionExploration::MicvisionExploration() {
  ros::NodeHandle robot_node;

  stop_server_ =
      robot_node.advertiseService(STOP_SERVICE,
                                  &MicvisionExploration::receiveStop, this);
  pause_server_ =
      robot_node.advertiseService(PAUSE_SERVICE,
                                  &MicvisionExploration::receivePause, this);

  ros::NodeHandle robot_node_pravite("~/");

  robot_node_pravite.param("map_frame", map_frame_, std::string("map"));
  robot_node_pravite.param("robot_frame", robot_frame_, std::string("robot"));
  robot_node_pravite.param("update_frequency", update_frequency_, 1.0);
  robot_node_pravite.param("explore_action_topic", explore_action_topic_,
                           std::string(EXPLORATION_ACTION));

  // Apply tf_prefix to all used frame-id's
  robot_frame_ = tf_listener_.resolve(robot_frame_);
  map_frame_ = tf_listener_.resolve(map_frame_);

  exploration_action_server_ =
      new Server(explore_action_topic_,
                 boost::bind(&MicvisionExploration::receiveExplorationGoal,
                             this, _1), false);
  exploration_action_server_->start();

  goal_publisher_ = robot_node.advertise<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 2);
  map_sub_ = robot_node.subscribe("/move_base_node/global_costmap/costmap", 1,
                                  &MicvisionExploration::mapCallback, this);
  ros::Duration(1.0).sleep();
  scan_sub_ = robot_node.subscribe("/scan", 1,
                                   &MicvisionExploration::scanCallback, this);

  stop_publisher_ = robot_node.advertise<actionlib_msgs::GoalID>(
      "/move_base/cancel", 2);
}

MicvisionExploration::~MicvisionExploration() {
  delete exploration_action_server_;
}

bool MicvisionExploration::receiveStop(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res) {
  is_stopped_ = true;
  res.success = true;
  res.message = "Exploration received stop signal.";

  actionlib_msgs::GoalID stop;
  stop.id = "";
  stop.stamp.sec = 0;
  stop.stamp.nsec = 0;
  stop_publisher_.publish(stop);
  return true;
}

bool MicvisionExploration::receivePause(std_srvs::Trigger::Request &req,
                                        std_srvs::Trigger::Response &res) {
  if ( is_paused_ ) {
    is_paused_ = false;
    res.success = false;
    res.message = "Exploration continues.";
  } else {
    is_paused_ = true;
    res.success = true;
    res.message = "Exploration pauses.";
  }
  return true;
}

void MicvisionExploration::stop() {
  is_paused_ = false;
  is_stopped_ = false;
}

void MicvisionExploration::receiveExplorationGoal(
    const micvision::ExplorationGoal::ConstPtr &goal) {
  ros::Rate loop_rate(update_frequency_);
  int index = 100;
  while ( true ) {
    // Check if we are asked to preempt
    if ( !ros::ok() || exploration_action_server_->isPreemptRequested()
        || is_stopped_ ) {
      ROS_INFO("Exploration has been preempted externally.");
      exploration_action_server_->setPreempted();
      stop();
      return;
    }

    // Where are we now
    if ( !setCurrentPosition() ) {
      ROS_ERROR("Exploration failed, could not get current position.");
      exploration_action_server_->setAborted();
      stop();
      return;
    }

    map_mutex.lock();
    scan_mutex.lock();
    auto goal_index = current_map_.getSize();
    goal_pixel_ = world2pixel(goal_point_);
    if ( index++ > 20 || current_map_.getData(goal_pixel_) != -1 ) {
      index = 0;
      ROS_DEBUG("exploration: start = %u, end = %u.",
               start_index_, goal_index);
      int result =
          findExplorationTarget(&current_map_, start_index_, goal_index);
      ROS_DEBUG("exploration: start = %u, end = %u.",
               start_index_, goal_index);
      ROS_DEBUG("start: x = %u, y = %u", robot_pixel_(0), robot_pixel_(1));

      ROS_DEBUG("start: x = %f, y = %f", robot_point_(0), robot_point_(1));

      // Point goal_point;

      bool goal_is_too_close = false;
      if ( goal_index == current_map_.getSize() ) {
        ROS_INFO("The goal is out of bround.");
        goal_point_ = robot_point_;
      } else {
        current_map_.getCoordinates(goal_pixel_, goal_index);
        /*
         *std::cout << "start: " << robot_pixel_(0)
         *    << ", " << robot_pixel_(1) << ";  stop: "
         *    << goal_pixel_(0) << ", " << goal_pixel_(1) << std::endl;
         */

        if ( PixelDistance(robot_pixel_, goal_pixel_) <= 9 ) {
          index = 0;
          goal_is_too_close = true;
          findBestDirection();
          goal_point_ = pixel2world(goal_pixel_);
        } else {
          goal_point_ = pixel2world(goal_pixel_);
        }
      }
      ROS_INFO("goal: x = %f, y = %f", goal_point_(0), goal_point_(1));

      geometry_msgs::PoseStamped posestamped_goal;
      posestamped_goal.header.stamp = ros::Time::now();
      posestamped_goal.header.frame_id = "map";
      posestamped_goal.pose.position.x = goal_point_(0);
      posestamped_goal.pose.position.y = goal_point_(1);
      posestamped_goal.pose.orientation =
          tf::createQuaternionMsgFromYaw(robot_direction_);
      goal_publisher_.publish(posestamped_goal);
    }

    // Sleep remaining time
    scan_mutex.unlock();
    map_mutex.unlock();
    ros::spinOnce();
    loop_rate.sleep();
    if ( loop_rate.cycleTime() > ros::Duration(1.0 / update_frequency_) )
      ROS_WARN("Missed desired rate of %.2fHz! Loop took %.4f seconds!",
               update_frequency_, loop_rate.cycleTime().toSec());
  }
}

bool MicvisionExploration::setCurrentPosition() {
  tf::StampedTransform transform;
  try {
    tf_listener_.lookupTransform(map_frame_, robot_frame_,
                                 ros::Time(0), transform);
  } catch ( tf::TransformException ex ) {
    ROS_ERROR("Could not get robot position: %s", ex.what());
    return false;
  }
  robot_point_ = Point(transform.getOrigin().x(), transform.getOrigin().y());
  robot_direction_ = getYaw(transform.getRotation());

  robot_pixel_ = world2pixel(robot_point_);
  unsigned int i;

  if ( !current_map_.getIndex(robot_pixel_, i) ) {
    ROS_ERROR("Is the robot out of the map?");
    return false;
  }
  start_index_ = i;
  return true;
}

void MicvisionExploration::mapCallback(
    const nav_msgs::OccupancyGrid& global_map) {
  map_mutex.lock();
  current_map_.update(global_map);
  current_map_.setLethalCost(80);
  map_mutex.unlock();
}

void MicvisionExploration::scanCallback(const sensor_msgs::LaserScan& scan) {
  setCurrentPosition();
  scan_mutex.lock();
  if ( true ) {
    angle_min_ = scan.angle_min;
    angle_max_ = scan.angle_max;
    angle_increment_ = scan.angle_increment;
    scan_size_ = scan.ranges.size();
    auto angle = angle_min_;
    indices_.clear();
    indices_.reserve(scan.ranges.size());
    for ( int i = 0; i < scan.ranges.size(); ++i ) {
      const auto s = scan.ranges[i];
      if ( std::isinf(s) || s >= 9.5 ) {
        indices_.emplace_back(1);

        if ( i == scan.ranges.size() - 1 ) {
          auto left = i;
          // to the small part
          while ( indices_[left] ) {
            auto k = left;
            indices_[i]++;
            while ( k < i ) {
              indices_[k] = indices_[i];
              ++k;
            }
            --left;
          }

          auto right = 0;
          while ( indices_[right] ) {
            auto k = 0;
            indices_[i]++;
            while ( k <= right ) {
              indices_[k] = indices_[i];
              ++k;
            }
            ++right;
          }
          for ( left = left + 1; left <= i; left++ ) {
            indices_[left] = indices_[i];
          }
        } else {
          auto j = i-1;
          while ( j >= 0 && indices_[j] ) {
            auto k = j;
            indices_[i]++;
            while ( k < i ) {
              indices_[k] = indices_[i];
              ++k;
            }
            --j;
          }
        }
      } else {
        indices_.emplace_back(0);
      }

      angle += angle_increment_;
    }
  }
  scan_mutex.unlock();
}

/*
 *int MicvisionExploration::safeIndex(int input) const {
 *  auto result = input >= 0 ? input : input + scan_size_;
 *  result = result < scan_size_ ? result : result-scan_size_;
 *  return result;
 *}
 */

void MicvisionExploration::findBestDirection() {
  auto best_score = 0;
  Pixel far_away_pixel(-1, -1);
  for ( int e = 0; e < indices_.size(); e++ ) {
    if ( indices_[e] > 0 ) {
      const auto angle = angle_min_ + e * angle_increment_;

      Pixel pixel_end =
          world2pixel(robot_point_ +
                      Point(9.0f * cos(angle + robot_direction_),
                            9.0f * sin(angle + robot_direction_)));

      // Debug
      /*
       *ROS_INFO("e: %d, end: %d, %d", e, pixel_end(0), pixel_end(1));
       *ROS_INFO("end point: %f, %f", 9.0f * cos(angle), 9.0f * sin(angle));
       */
      std::vector<Pixel> line = bresenham(pixel_end);
      // Debug
      /*
       *for ( int i = 0; i < line.size(); i++ ) {
       *  if ( i % 5 == 0 )
       *  ROS_INFO("line: %d, %d", line[i](0), line[i](1));
       *}
       */

      auto unknow_num = 0;
      Pixel far_pixel(-1, -1);
      for ( const auto & p : line ) {
        const auto data = current_map_.getData(p);
        if ( -1 == data ) {
          unknow_num++;
          far_pixel = p;
        } else if ( data >= 50 ) {
          break;
        }
      }
      ROS_DEBUG("index: %d, score: %d", e, unknow_num);

      if ( unknow_num > best_score ) {
        ROS_DEBUG("update best score, e: %d", e);
        best_score = unknow_num;
        far_away_pixel = far_pixel;
        angles_ = angle;
      }
    }
  }
  ROS_DEBUG("far_away_pixel: %d, %d, angle: %f",
           far_away_pixel(0), far_away_pixel(1), angles_);
  if ( far_away_pixel(0) != -1 || far_away_pixel(1) != -1 )
    goal_pixel_ = far_away_pixel;
}

Pixel MicvisionExploration::world2pixel(const Point& point) const {
  Point p;
  p << (point(0) - current_map_.getOriginX()) / current_map_.getResolution(),
       (point(1) - current_map_.getOriginY()) / current_map_.getResolution();

  return Pixel(p(0), p(1));
}

Point MicvisionExploration::pixel2world(const Pixel& pixel) const {
  Point p;
  p << pixel(0) * current_map_.getResolution() + current_map_.getOriginX(),
       pixel(1) * current_map_.getResolution() + current_map_.getOriginY();

  return p;
}

std::vector<Pixel> MicvisionExploration::bresenham(const Pixel& end) {
  int x = robot_pixel_(0), y = robot_pixel_(1);
  int dx = abs(end(0) - x);
  int dy = abs(end(1) - y);

  const int ux = end(0) > x ? 1 : -1;
  const int uy = end(1) > y ? 1 : -1;

  bool exchange = false;      // exchange dx and dy?
  if ( dy > dx ) {
    std::swap(dx, dy);
    exchange = true;
  }

  int p = 2 * dy - dx;
  std::vector<Pixel> result;

  for ( int i = 0; i <= dx; ++i ) {
    // ROS_INFO("x: %d, y: %d", x, y);
    if ( x <= 5 || x >= current_map_.getWidth() - 5 ||
        y <= 5 || y >= current_map_.getHeight() - 5 )
      break;
    result.push_back(Pixel(x, y));
    if ( p >= 0 ) {
      if ( !exchange )
        y += uy;
      else
        x += ux;
      p -= 2*dx;
    }

    if ( !exchange )
      x += ux;
    else
      y += uy;
    p += 2*dy;
  }
  return result;
}
}  // end namespace micvision
