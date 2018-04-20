#include <nav_msgs/GridCells.h>

#include <micvision/exploration.h>

#include <set>
#include <map>
#include <limits>
#include <mutex>

#define DISTANCE_THRESHOLD      40      // threshold for the distance between the goal and robot position
#define SEARCH_DEEPER_INTERVAL  16      // step interval for search deeper situation
#define FIND_SECTOR_INTERVAL    50      // step interval for find sector situation

namespace micvision {
using Queue = std::multimap<double, unsigned int>;
using Entry = std::pair<double, unsigned int>;

// mutex
std::mutex mtx;

constexpr int EXPLORATION_TARGET_SET = 1;
constexpr int EXPLORATION_FINISHED = 2;
constexpr int EXPLORATION_WAITING = 3;
constexpr int EXPLORATION_FAILED = 4;
int findExplorationTarget(GridMap* map,
                          const unsigned int start,
                          unsigned int &goal) {
  // Create some workspace for the wavefront algorithm
  const unsigned int map_size = map->getSize();
  double* plan;
  try {
      plan = new double[map_size];
  } catch(const std::bad_alloc&) {
     return -10;   // allocate failure
  }
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
  goal_publisher_ = robot_node.advertise<move_base_msgs::MoveBaseActionGoal/*geometry_msgs::PoseStamped*/>(
      "/move_base/goal", 2);
  stop_publisher_ = robot_node.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 2);
  map_sub_ = robot_node.subscribe("/move_base_node/global_costmap/costmap", 1,
                                  &MicvisionExploration::mapCallback, this);
  ros::Duration(1.0).sleep();
  scan_sub_ = robot_node.subscribe("/scan", 1,
                                   &MicvisionExploration::scanCallback, this);
  extern_goal_sub_ = robot_node.subscribe("/move_base_simple/goal", 1, 
                                          &MicvisionExploration::externGoalCallback,
                                          this);
  count_ = 0;
  interval_ = 16;
  goal_point_ = Point(100.0, 100.0);
  exploration_running_ = false;
}

MicvisionExploration::~MicvisionExploration() {
  delete exploration_action_server_;
}

bool MicvisionExploration::receiveStopExploration(std_srvs::Trigger::Request &req,
                                                  std_srvs::Trigger::Response &res) {
  if ( exploration_running_ ) {
    is_stopped_ = true;
  }
  res.success = true;
  res.message = "Exploration received stop signal.";
  return true;
}

bool MicvisionExploration::receiveStop(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res) {
  if ( exploration_running_ ) {
    is_stopped_ = true;
  }
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
  if ( !setCurrentPosition() ) {
    exploration_running_ = false;
    return false;
}

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
  exploration_running_ = true;
  // so we can search for a goal immediately 
  count_ = interval_;
  while ( true ) {
    // Check if we are asked to preempt
    receive_new_map_ = false;
    if ( !ros::ok() || exploration_action_server_->isPreemptRequested()
        || is_stopped_ ) {
      exploration_action_server_->setPreempted();
      stop();
      receive_new_map_ = true;
      exploration_running_ = false;
      return;
    }
    mtx.lock();
    // to get the current map coordinates of the last goal,then we can determine whether it is spotted
    Pixel goal_pixel;
    goal_pixel = world2pixel(goal_point_);

    if ( count_ == interval_ || (current_map_.getIndex(goal_pixel, goal_index_)
         && !current_map_.isFrontier(goal_index_)) ) {
      count_ = 0;
      // Where are we now
      if ( !setCurrentPosition() ) {
        ROS_ERROR("Exploration failed, could not get current position.");
        exploration_action_server_->setAborted();
        stop();
        receive_new_map_ = true;
        exploration_running_ = false;
        mtx.unlock();
        return;
      }

      if ( preparePlan() ) {
        int result =
            findExplorationTarget(&current_map_, start_index_, goal_index_);
        if ( goal_index_ == current_map_.getSize() || result == -10 ) {
          goal_point_ = robot_point_;
        } else {
          current_map_.getCoordinates(goal_pixel, goal_index_);
          // if the distance between the goal and the robot position is lesser
          // than a threshold, we look for a sector or search deeper.
          if ( PixelDistance(robot_pixel_, goal_pixel) <= DISTANCE_THRESHOLD ) {
            if ( !findSector() )
              searchDeeper();
          } else {
            goal_point_ = pixel2world(goal_pixel);
          }
        }
        
        move_base_msgs::MoveBaseActionGoal move_base_action_goal;
        move_base_action_goal.header.stamp  = ros::Time::now();
        move_base_action_goal.goal_id.stamp.sec = 0;
        move_base_action_goal.goal_id.stamp.nsec = 0;
        move_base_action_goal.goal_id.id = "";
        geometry_msgs::PoseStamped posestamped_goal;
        posestamped_goal.header.stamp = ros::Time::now();
        posestamped_goal.header.frame_id = "map";
        posestamped_goal.pose.position.x = goal_point_(0);
        posestamped_goal.pose.position.y = goal_point_(1);
        posestamped_goal.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        move_base_action_goal.goal.target_pose = posestamped_goal;
        if ( !is_stopped_ )
          goal_publisher_.publish(move_base_action_goal/*posestamped_goal*/);
      }
    }
    receive_new_map_ = true;
    count_++;
    mtx.unlock();
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
  robot_theta_ = getYaw(transform.getRotation());

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
  mtx.lock();
  if ( receive_new_map_ ) {
    current_map_.update(global_map);
    current_map_.setLethalCost(80);
  }
  mtx.unlock();
}

void MicvisionExploration::scanCallback(const sensor_msgs::LaserScan& scan) {
  ROS_DEBUG("scanCallback");
  // TODO: to be fulfill
  // to copy the scan
  scan_ = scan;
}

Pixel MicvisionExploration::world2pixel(const Point& point) const {
  Point p;
  p << (point(0) - current_map_.getOriginX()) / current_map_.getResolution(),
       (point(1) - current_map_.getOriginY()) / current_map_.getResolution();

  return Pixel(p(0), p(1));
}


void MicvisionExploration::externGoalCallback(const geometry_msgs::PoseStamped& extern_goal) {
  if ( exploration_running_ ) {
    is_stopped_ = true;
  }
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

bool MicvisionExploration::findSector() {
  const int inf_threshold = 4;
  int inf_count = 0;
  int start_i = 0, end_i = 0;
  double sector_bound = 0.0;

  bool sector_found = false;

  Pixel start_pixel;

  unsigned int length = 0;
  if ( scan_.ranges[0] >= scan_.range_max 
      || scan_.ranges[0] == std::numeric_limits<float>::infinity()) {
    while ( scan_.ranges[start_i] >= scan_.range_max
           || scan_.ranges[start_i] == std::numeric_limits<float>::infinity() ) {
      start_i++;
      inf_count++;
    }
    end_i = scan_.ranges.size() - 1;
    while ( scan_.ranges[end_i] >= scan_.range_max
           || scan_.ranges[end_i] == std::numeric_limits<float>::infinity() ) {
      end_i--;
      inf_count++;
    }
    if ( inf_count > inf_threshold ) {
      const int step_length = 5;
      const int step_max = 20;
      const int sector_threshold = 5;
      unsigned int curr_index = 0;
      unsigned int step_count = 1;

      double direction = (start_i + end_i - scan_.ranges.size()) / 2 * scan_.angle_increment
                          + robot_theta_ + scan_.angle_min;
      current_map_.getCoordinates(start_pixel, start_index_);
      current_map_.getIndex(start_pixel(0) + step_length * std::cos(direction),
                            start_pixel(1) + step_length * std::sin(direction),
                            curr_index);
      // to determine the depth of the sector
      while ( current_map_.isFrontier(curr_index) && step_count < step_max ) {
        step_count++;
        current_map_.getIndex(start_pixel(0) + step_length * step_count * std::cos(direction),
                              start_pixel(1) + step_length * step_count * std::sin(direction),
                              curr_index);
      }
      if ( step_count > sector_threshold ) {
        if ( scan_.ranges[end_i] > scan_.ranges[start_i] )
          sector_bound = end_i * scan_.angle_increment + robot_theta_ + scan_.angle_min;
        else
          sector_bound = start_i * scan_.angle_increment + robot_theta_ + scan_.angle_min;
        length = step_count * step_length;
        sector_found = true;
      }
    }
  }
  start_i = end_i = inf_count = 0;
  if ( !sector_found ) {
    for ( int i = 0; i < scan_.ranges.size(); i++ ) {
      if ( (scan_.ranges[i] >= scan_.range_max || scan_.ranges[i] == std::numeric_limits<float>::infinity())
           && i < scan_.ranges.size() - 1 )
        inf_count++;
      else {
        if ( inf_count > inf_threshold ) {
          end_i = i;
          double direction = (start_i + end_i) / 2 * scan_.angle_increment + robot_theta_ + scan_.angle_min;
          const int step_length = 5;
          const int step_max = 20;
          const int sector_threshold = 5;
          unsigned int curr_index = 0;
          unsigned int step_count = 1;
          current_map_.getCoordinates(start_pixel, start_index_);
          current_map_.getIndex(start_pixel(0) + step_length * std::cos(direction),
                                start_pixel(1) + step_length * std::sin(direction),
                                curr_index);
          // to determine the depth of the sector
          while ( current_map_.isFrontier(curr_index) && step_count < step_max) {
            step_count++;
            current_map_.getIndex(start_pixel(0) + step_length * step_count * std::cos(direction),
                                  start_pixel(1) + step_length * step_count * std::sin(direction),
                                  curr_index);
          }
          if ( step_count > sector_threshold ) {
            if ( scan_.ranges[start_i] >= scan_.range_max
                || scan_.ranges[start_i] == std::numeric_limits<float>::infinity() ) {
              start_i--;
            } 
            if ( scan_.ranges[end_i] > scan_.ranges[start_i] )
              sector_bound = end_i * scan_.angle_increment + robot_theta_ + scan_.angle_min;
            else
              sector_bound = start_i * scan_.angle_increment + robot_theta_ + scan_.angle_min;
            sector_found = true;
            length = step_count * step_length;
            break;
          }
        }
        inf_count = 0;
        start_i = i;
      }
    }
  }
  if ( sector_found ) {
    current_map_.getIndex(start_pixel(0) + length * std::cos(sector_bound),
                          start_pixel(1) + length * std::sin(sector_bound),
                          goal_index_);
    updateGoalCoordinates(goal_index_);
    interval_ = FIND_SECTOR_INTERVAL;
    return true;
  }
  else
    return false;
}

void MicvisionExploration::searchDeeper() {
  unsigned int ind[5];

  ind[0] = goal_index_ - 1;                        // left
  ind[1] = goal_index_ + 1;                        // right
  ind[2] = goal_index_ - current_map_.getWidth();  // up
  ind[3] = goal_index_ + current_map_.getWidth();  // down
  ind[4] = goal_index_;                            // origin

  int curr_longest = 0;
  const int step_length = 5;
  const int step_max = 10;
  for ( int i = 0; i < 5; i++ ) {
    unsigned int curr_index = ind[i];
    if ( !current_map_.isFrontier(curr_index) )
      continue;
    Pixel start_pixel;
    // to get the coordinates of the position of the robot
    current_map_.getCoordinates(start_pixel, start_index_);
    Pixel goal_pixel;
    // to get the coordinates of the nearest frontier
    current_map_.getCoordinates(goal_pixel, curr_index);
    // to calculate the heading direction
    double direction = std::atan2(int(goal_pixel(1)) - int(start_pixel(1)),
                                  int(goal_pixel(0)) - int(start_pixel(0)));
    // to search deeper
    int step = 0;
    unsigned int oldIndex = curr_index;
    Pixel curr_pixel;
    while ( current_map_.isFrontier(curr_index) && step < step_max ) {
      step++;
      oldIndex = curr_index;
      current_map_.getCoordinates(curr_pixel, oldIndex);
      current_map_.getIndex(curr_pixel(0) + step_length * std::cos(direction),
                            curr_pixel(1) + step_length * std::sin(direction),
                            curr_index);
    }
    int curr_distance = (curr_pixel(0) - start_pixel(0)) * (curr_pixel(0) - start_pixel(0))
                         + (curr_pixel(1) - start_pixel(1)) * (curr_pixel(1) - start_pixel(1));
    if ( curr_distance > curr_longest ) {
      goal_index_ = oldIndex;
      curr_longest = curr_distance;
    }
  }
  updateGoalCoordinates(goal_index_);
  interval_ = SEARCH_DEEPER_INTERVAL;
}

void MicvisionExploration::updateGoalCoordinates(const unsigned int& goal) {
  Pixel goal_pixel;
  current_map_.getCoordinates(goal_pixel, goal);
  goal_point_ = pixel2world(goal_pixel);
}

}  // end namespace micvision
