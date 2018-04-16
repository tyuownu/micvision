#include <nav_msgs/GridCells.h>

#include <micvision/exploration.h>

#include <set>
#include <map>


namespace micvision {
using Queue = std::multimap<double, unsigned int>;
using Entry = std::pair<double, unsigned int>;

constexpr int EXPLORATION_TARGET_SET = 1;
constexpr int EXPLORATION_FINISHED = 2;
constexpr int EXPLORATION_WAITING = 3;
constexpr int EXPLORATION_FAILED = 4;
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
      robot_node.advertiseService(PAUSE_EXPLORATION_SERVICE,
                                  &MicvisionExploration::receivePause, this);

  stop_exploration_server_ = 
      robot_node.advertiseService(STOP_EXPLORATION_SERVICE,
                                  &MicvisionExploration::receiveStopExploration,
                                  this);

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

  receive_new_map_ = true;
  is_stopped_ = false;
  is_paused_ = false;
  goal_publisher_ = robot_node.advertise<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 2);
  stop_publisher_ = robot_node.advertise<actionlib_msgs::GoalID>("/move_base/cancel",2);
  map_sub_ = robot_node.subscribe("/move_base_node/global_costmap/costmap", 1,
                                  &MicvisionExploration::mapCallback, this);
  ros::Duration(1.0).sleep();
  scan_sub_ = robot_node.subscribe("/base_scan", 1,
                                   &MicvisionExploration::scanCallback, this);
}

MicvisionExploration::~MicvisionExploration() {
  delete exploration_action_server_;
}

bool MicvisionExploration::receiveStopExploration(std_srvs::Trigger::Request &req,
                                                  std_srvs::Trigger::Response &res) {
  is_stopped_ = true;
  res.success = true;
  res.message = "Exploration received stop signal.";
  return true;
}

bool MicvisionExploration::receiveStop(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res) {
  is_stopped_ = true;
  res.success = true;
  res.message = "Navigator received stop signal.";

  actionlib_msgs::GoalID stop_signal;
  stop_signal.id = "";
  stop_signal.stamp.sec = 0;
  stop_signal.stamp.nsec = 0;
  stop_publisher_.publish(stop_signal);

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

bool MicvisionExploration::preparePlan() {
  // Where am I?
  if ( !setCurrentPosition() ) return false;

  // Clear robot footprint in map

  return true;
}

void MicvisionExploration::stop() {
  is_paused_ = false;
  is_stopped_ = false;
}

void MicvisionExploration::receiveExplorationGoal(
    const micvision::ExplorationGoal::ConstPtr &goal) {
  ros::Rate loop_rate(update_frequency_);
  ros::Rate long_rate(0.2);
  while ( true ) {
    // Check if we are asked to preempt
    receive_new_map_ = false;
    if ( !ros::ok() || exploration_action_server_->isPreemptRequested()
        || is_stopped_ ) {
      ROS_INFO("Exploration has been preempted externally.");
      exploration_action_server_->setPreempted();
      stop();
      receive_new_map_ = true;
      return;
    }

    // Where are we now
    if ( !setCurrentPosition() ) {
      ROS_ERROR("Exploration failed, could not get current position.");
      exploration_action_server_->setAborted();
      stop();
      receive_new_map_ = true;
      return;
    }

    goal_index_ = current_map_.getSize();
    if ( preparePlan() ) {
      ROS_INFO("exploration: start = %u, end = %u.",
               start_index_, goal_index_);
      int result =
          findExplorationTarget(&current_map_, start_index_, goal_index_);
      ROS_INFO("exploration: start = %u, end = %u.",
               start_index_, goal_index_);
      ROS_INFO("start: x = %u, y = %u", robot_pixel_(0), robot_pixel_(1));
      Pixel goal_pixel;

      ROS_INFO("start: x = %f, y = %f", robot_point_(0), robot_point_(1));

      Point goal_point;

      bool no_vaild_goal = false;
      if ( goal_index_ == current_map_.getSize() ) {
        goal_point = robot_point_;
      } else {
        current_map_.getCoordinates(goal_pixel, goal_index_);
        std::cout << "start: " << robot_pixel_(0)
            << ", " << robot_pixel_(1) << ";  stop: "
            << goal_pixel(0) << ", " << goal_pixel(1) << std::endl;

        if ( PixelDistance(robot_pixel_, goal_pixel) <= 0.5 ) {
          // TODO: what need to do is to optimize it
          double x, y;
          x = robot_pixel_(0) * current_map_.getResolution() +
              current_map_.getOriginX() +
              (longest_distance_ - 1) * cos(angles_);
          std::cout << "x = " << x << ", boundx: "
              << current_map_.getBoundaryX() << std::endl;
          if ( x <= current_map_.getOriginX() )
            x = current_map_.getOriginX() + 1;
          else if ( x >= current_map_.getBoundaryX() )
            x = current_map_.getBoundaryX() - 1;

          y = robot_pixel_(1) * current_map_.getResolution() +
              current_map_.getOriginY() +
              (longest_distance_ - 1) * sin(angles_);
          if ( y <= current_map_.getOriginY() )
            y = current_map_.getOriginY() + 1;
          else if ( y >= current_map_.getBoundaryX() )
            y = current_map_.getBoundaryX() - 1;
          goal_point << x, y;

          no_vaild_goal = true;
          ROS_INFO("longest_distance_: %f, angles_: %f",
                   longest_distance_, angles_);
        } else {
          goal_point = pixel2world(goal_pixel);
        }
      }
      ROS_INFO("goal: x = %f, y = %f", goal_point(0), goal_point(1));

      geometry_msgs::PoseStamped posestamped_goal;
      posestamped_goal.header.stamp = ros::Time::now();
      posestamped_goal.header.frame_id = "map";
      posestamped_goal.pose.position.x = goal_point(0);
      posestamped_goal.pose.position.y = goal_point(1);
      posestamped_goal.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      goal_publisher_.publish(posestamped_goal);

      if ( no_vaild_goal ) {
        ros::Rate long_rate(0.25 * 2 / longest_distance_);
        long_rate.sleep();
      }
    }
    receive_new_map_ = true;

    // Sleep remaining time
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
  // double world_theta = getYaw(transform.getRotation());

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
  if ( receive_new_map_ ) {
    current_map_.update(global_map);
    current_map_.setLethalCost(80);
  }
}

void MicvisionExploration::scanCallback(const sensor_msgs::LaserScan& scan) {
  ROS_DEBUG("scanCallback");
  // TODO: to be fulfill
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


int MicvisionExploration::scoreLine(double angle, double range) {
  ROS_DEBUG("scoreLine");
  // TODO: to be fulfill
  return 0;
}

std::vector<Pixel> MicvisionExploration::bresenham(const Pixel& end) {
  int x = 0, y = 0;
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
    result.push_back(Pixel(x, y));
    if ( p >= 0 ) {
      if ( !exchange ) y += uy;
      else x += ux;
      p -= 2*dx;
    }

    if ( !exchange ) x += ux;
    else y += uy;
    p += 2*dy;
  }
  return result;
}
}  // end namespace micvision
