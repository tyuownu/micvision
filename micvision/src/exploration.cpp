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
                          unsigned int start,
                          unsigned int &goal) {
  // Create some workspace for the wavefront algorithm
  unsigned int map_size = map->getSize();
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
  double linear = map->getResolution();
  bool found_frontier = false;
  int cell_count = 0;

  // Do full search with weightless Dijkstra-Algorithm
  while ( !queue.empty() ) {
    cell_count++;
    // Get the nearest cell from the queue
    next = queue.begin();
    distance = next->first;
    unsigned int index = next->second;
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
        unsigned int i = ind[it];
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
MicvisionExploration::MicvisionExploration() {
  ros::NodeHandle robot_node;

  stop_server_ = robot_node.advertiseService(STOP_SERVICE,
                                             &MicvisionExploration::receiveStop, this);
  pause_server_ = robot_node.advertiseService(PAUSE_SERVICE,
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

  has_new_map_ = false;
  is_stopped_ = false;
  is_paused_ = false;
  goal_publisher_ = robot_node.advertise<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 2);
  map_sub_ = robot_node.subscribe("/move_base_node/global_costmap/costmap", 1,
                                  &MicvisionExploration::mapCallback, this);
  ros::Duration(1.0).sleep();
  scan_sub_ = robot_node.subscribe("/base_scan", 1,
                                   &MicvisionExploration::scanCallback, this);
}

MicvisionExploration::~MicvisionExploration() {
  delete exploration_action_server_;
}

bool MicvisionExploration::receiveStop(std_srvs::Trigger::Request &req,
                                       std_srvs::Trigger::Response &res) {
  is_stopped_ = true;
  res.success = true;
  res.message = "Exploration received stop signal.";
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

    goal_point_ = current_map_.getSize();
    if ( preparePlan() ) {
      ROS_INFO("exploration: start = %u, end = %u.",
               start_point_, goal_point_);
      int result =
          findExplorationTarget(&current_map_, start_point_, goal_point_);
      ROS_INFO("exploration: start = %u, end = %u.",
               start_point_, goal_point_);
      unsigned int x_start = 0, y_start = 0;
      current_map_.getCoordinates(x_start, y_start, start_point_);
      ROS_INFO("start: x = %u, y = %u", x_start, y_start);
      unsigned int x_stop = 0, y_stop = 0;


      double x_ = x_start * current_map_.getResolution() +
          current_map_.getOriginX();
      double y_ = y_start * current_map_.getResolution() +
          current_map_.getOriginY();
      ROS_INFO("start: x = %f, y = %f", x_, y_);

      double x, y;
      bool no_vaild_goal = false;
      if ( goal_point_ == current_map_.getSize() ) {
        x = x_start * current_map_.getResolution() +
            current_map_.getOriginX();
        y = y_start * current_map_.getResolution() +
            current_map_.getOriginY();
      } else {
        current_map_.getCoordinates(x_stop, y_stop, goal_point_);
        std::cout << "start: " << x_start << ", " << y_start << ";  stop: "
            << x_stop << ", " << y_stop << std::endl;

        if ( ((x_start - x_stop) * (x_start - x_stop) +
              (y_start - y_stop) * (y_start - y_stop)) <= 25 ) {
          x = x_start * current_map_.getResolution() +
              current_map_.getOriginX() +
              (longest_distance_ - 1) * cos(angles_);
          std::cout << "x = " << x << ", boundx: "
              << current_map_.getBoundaryX() << std::endl;
          if ( x <= current_map_.getOriginX() )
            x = current_map_.getOriginX() + 1;
          else if ( x >= current_map_.getBoundaryX() )
            x = current_map_.getBoundaryX() - 1;

          y = y_start * current_map_.getResolution() +
              current_map_.getOriginY() +
              (longest_distance_ - 1) * sin(angles_);
          if ( y <= current_map_.getOriginY() )
            y = current_map_.getOriginY() + 1;
          else if ( y >= current_map_.getBoundaryX() )
            y = current_map_.getBoundaryX() - 1;

          no_vaild_goal = true;
          ROS_INFO("longest_distance_: %f, angles_: %f",
                   longest_distance_, angles_);
        } else {
          x = x_stop * current_map_.getResolution() +
              current_map_.getOriginX();
          y = y_stop * current_map_.getResolution() +
              current_map_.getOriginY();
        }
      }
      ROS_INFO("goal: x = %f, y = %f", x, y);

      geometry_msgs::PoseStamped goal_base;
      goal_base.header.stamp = ros::Time::now();
      goal_base.header.frame_id = "map";
      goal_base.pose.position.x = x;
      goal_base.pose.position.y = y;
      goal_base.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      goal_publisher_.publish(goal_base);

      if ( no_vaild_goal ) {
        ros::Rate long_rate(0.25 * 2 / longest_distance_);
        long_rate.sleep();
      }
    }
    has_new_map_ = false;

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
  double world_x = transform.getOrigin().x();
  double world_y = transform.getOrigin().y();
  double world_theta = getYaw(transform.getRotation());

  unsigned int current_x = (world_x - current_map_.getOriginX()) /
      current_map_.getResolution();
  unsigned int current_y = (world_y - current_map_.getOriginY()) /
      current_map_.getResolution();
  unsigned int i;

  if ( !current_map_.getIndex(current_x, current_y, i) ) {
    if ( has_new_map_ || !current_map_.getIndex(current_x, current_y, i) ) {
      ROS_ERROR("Is the robot out of the map?");
      return false;
    }
  }
  start_point_ = i;
  return true;
}

void MicvisionExploration::mapCallback(
    const nav_msgs::OccupancyGrid& global_map) {
  if ( !has_new_map_ ) {
    current_map_.update(global_map);
    current_map_.setLethalCost(80);
    has_new_map_ = true;
  }
}

void MicvisionExploration::scanCallback(const sensor_msgs::LaserScan& scan) {
  double angle = scan.angle_min;
  int index = 0;

  int highest_score = -1;

  bool in_inf_range = false;
  double best_range = 0.0, best_angle = 0.0;
  int INDEX = 1;
  while ( angle <= scan.angle_max ) {
    int score;
    if ( !std::isinf(scan.ranges[index]) ) {
      if ( scan.ranges[index] < scan.range_max ) {
        if ( in_inf_range ) {
          score = scoreLine(angle -
                            INDEX * scan.angle_increment, scan.ranges[index]);
          if ( score > highest_score ) {
            highest_score = score;
            best_range = scan.ranges[index];
            best_angle = angle - INDEX * scan.angle_increment;
          }
          /*
           *std::cout << "in range angle: " << angle - INDEX*scan.angle_increment
           *  << ", range: " << scan.ranges[index]
           *  << ", score: " << scoreLine(angle - INDEX*scan.angle_increment, scan.ranges[index]) << std::endl;
           */

          in_inf_range = false;
        }
        score = scoreLine(angle, scan.ranges[index]);
        if ( score > highest_score ) {
          highest_score = score;
          best_range = scan.ranges[index];
          best_angle = angle;
        }
        /*
         *std::cout << "angle: " << angle
         *  << ", range: " << scan.ranges[index]
         *  << ", score: " << scoreLine(angle, scan.ranges[index]) << std::endl;
         */
      } else {
        if ( index == 0 ) in_inf_range = true;
        if ( !in_inf_range ) {
          score = scoreLine(angle + INDEX * scan.angle_increment,
                            scan.ranges[index - 1]);
          if ( score > highest_score ) {
            highest_score = score;
            best_range = scan.ranges[index - 1];
            best_angle = angle + INDEX * scan.angle_increment;
          }
          /*
           *std::cout << "in_inf_range angle: " << angle + INDEX * scan.angle_increment
           *  << ", range: " << scan.ranges[index - 1]
           *  << ", score: " << scoreLine(angle + INDEX*scan.angle_increment, scan.ranges[index - 1]) << std::endl;
           */
          in_inf_range = true;
        }
      }
    } else {
      if ( index == 0 ) in_inf_range = true;

      if ( !in_inf_range ) {
        std::cout << "in_inf_range angle: " << angle
            << ", range: " << scan.ranges[index-1]
            << ", score: " << scoreLine(angle, scan.ranges[index-1])
            << std::endl;
      }
    }

    index++;
    angle += scan.angle_increment;
  }

  longest_distance_ = best_range;
  angles_ = best_angle;
  ROS_INFO_STREAM("range: " << longest_distance_
                  << ", angle: " << angles_
                  << ", score: " << highest_score);

}

/*
   static void walkAlongTheLongestRay(double distance, double angle,
   double cur_x, double cur_y,
   double &x, double &y) {
   x = cur_x + distance * cos(angle);
   y = cur_y + distance * sim(angle);
   }
   */

enum QUADRANT {X_NEGTIVE, THRID, Y_NEGITIVE,
  FOURTH, X_POSITIVE, FIRST, Y_POSITIVE, SECOND};
int MicvisionExploration::scoreLine(double angle, double range) {
  int score = 0;
  setCurrentPosition();
  //if ( preparePlan() ) {
  unsigned int x_start = 0, y_start = 0;
  current_map_.getCoordinates(x_start, y_start, start_point_);

  /*
     int x_stop = 0, y_stop = 0;
     current_map_.getCoordinates(x_stop, y_stop, stop_point_);
     */

  double x_start_d = x_start * current_map_.getResolution() +
      current_map_.getOriginX();
  double y_start_d = y_start * current_map_.getResolution() +
      current_map_.getOriginY();
  double x_stop_d = x_start_d + range * cos(angle);
  double y_stop_d = y_start_d + range * sin(angle);
  unsigned int x_stop = (x_stop_d - current_map_.getOriginX()) /
      current_map_.getResolution();
  unsigned int y_stop = (y_stop_d - current_map_.getOriginY()) /
      current_map_.getResolution();
  /*
   *std::cout << "start x: " << x_start << ", y: " << y_start
   *  << "; stop x: " << x_stop << ", y: " << y_stop << std::endl;
   */

  int x_step = 0, y_step = 0;
  if ( x_stop > x_start )
    x_step = 1;
  else if ( x_stop < x_start )
    x_step = -1;

  if ( y_stop > y_start )
    y_step = 1;
  else if ( y_stop < y_start )
    y_step = -1;

  enum QUADRANT quadrant = FIRST;

  if ( x_step == -1  && y_step == 0 )
    quadrant = X_NEGTIVE;
  else if ( x_step == -1 && y_step == -1 )
    quadrant = THRID;
  else if ( x_step == 0 && y_step == -1 )
    quadrant = Y_NEGITIVE;
  else if ( x_step == 1 && y_step == -1 )
    quadrant = FOURTH;
  else if ( x_step == 1 && y_step == 0 )
    quadrant = X_POSITIVE;
  else if ( x_step == 1 && y_step == 1 )
    quadrant = FIRST;
  else if ( x_step == 0 && y_step == 1 )
    quadrant = Y_POSITIVE;
  else if ( x_step == -1 && y_step == 1 )
    quadrant = SECOND;



  int x = x_start, y = y_start;
  bool reach_end = false;

  while ( true ) {
    if ( current_map_.getData(x, y) == -1 ) score++;
    int right_up, right_down, left_up, left_down;
    switch ( quadrant ) {

      case X_NEGTIVE:
        if ( x < x_stop ) reach_end = true;
        x += x_step;
        break;

      case THRID:
        if ( x < x_stop || y < y_stop ) reach_end = true;
        left_down = (x - int(x_start) + x_step) * (int(y_start) - int(y_stop))
            - (int(x_start) - int(x_stop)) * (y - int(y_start) + y_step);
        left_up = (x - int(x_start) + x_step) * (int(y_start) - int(y_stop)) -
            (int(x_start) - int(x_stop)) * (y - int(y_start));
        right_down = (x - int(x_start)) * (int(y_start) - int(y_stop)) -
            (int(x_start) - int(x_stop)) * (y - int(y_start) + y_step);
        /*
         *std::cout << "x: " << x << ", y: " << y
         *  << ", left_down: " << left_down
         *  << ", left_up: " << left_up
         *  << ", right_down: " << right_down
         *  << std::endl;;
         */
        if ( left_down * right_down > 0 )
          x += x_step;
        else if ( left_down * left_up > 0 )
          y += y_step;
        else {
          x += x_step; y += y_step;
        }
        break;

      case Y_NEGITIVE:
        if ( y < y_stop ) reach_end = true;
        y += y_step;
        break;

      case FOURTH:
        if ( x > x_stop || y < y_stop ) reach_end = true;
        right_down = (x - int(x_start) + x_step) * (int(y_stop) - int(y_start))
            - (int(x_stop) - int(x_start)) * (y - int(y_start) + y_step);
        right_up = (x - int(x_start) + x_step) * (int(y_stop) - int(y_start))
            - (int(x_stop) - int(x_start)) * (y - int(y_start));
        left_down = (x - int(x_start)) * (int(y_stop) - int(y_start))
            - (int(x_stop) - int(x_start)) * (y - int(y_start) + y_step);
        /*
         *std::cout << "x: " << x << ", y: " << y
         *  << ", right_down: " << right_down
         *  << ", right_up: " << right_up
         *  << ", left_down: " << left_down
         *  << std::endl;;
         */
        if ( right_down * left_down > 0 )
          x += x_step;
        else if ( right_down * right_up > 0 )
          y += y_step;
        else {
          x += x_step; y += y_step;
        }
        break;

      case X_POSITIVE:
        if ( x > x_stop ) reach_end = true;
        x += x_step;
        break;

      case FIRST:
        if ( x > x_stop || y > y_stop ) reach_end = true;
        right_up = (x - int(x_start) + x_step) * (int(y_stop) - int(y_start))
            - (int(x_stop) - int(x_start)) * (y - int(y_start) + y_step);
        right_down = (x - int(x_start) + x_step) * (int(y_stop) - int(y_start))
            - (int(x_stop) - int(x_start)) * (y - int(y_start));
        left_up = (x - int(x_start)) * (int(y_stop) - int(y_start)) -
            (int(x_stop) - int(x_start)) * (y - int(y_start) + y_step);
        /*
         *std::cout << "x: " << x << ", y: " << y
         *  << ", right_up: " << right_up
         *  << ", right_down: " << right_down
         *  << ", left_up: " << left_up
         *  << std::endl;;
         */
        if ( right_up * right_down > 0 )
          y += y_step;
        else if ( right_up * left_up > 0 )
          x += x_step;
        else {
          x += x_step; y += y_step;
        }
        break;

      case Y_POSITIVE:
        if ( y > y_stop ) reach_end = true;
        y += y_step;
        break;

      case SECOND:
        if ( x < x_stop || y > y_stop ) reach_end = true;
        left_up = (x - int(x_start) + x_step) * (int(y_start) - int(y_stop)) -
            (int(x_start) - int(x_stop)) * (y - int(y_start) + y_step);
        left_down = (x - int(x_start) + x_step) * (int(y_start) - int(y_stop)) -
            (int(x_start) - int(x_stop)) * (y - int(y_start));
        right_up = (x - int(x_start)) * (int(y_start) - int(y_stop)) -
            (int(x_start) - int(x_stop)) * (y - int(y_start) + y_step);
        /*
         *std::cout << "x: " << x << ", y: " << y
         *  << ", left_up: " << left_up
         *  << ", left_down: " << left_down
         *  << ", right_up: " << right_up
         *  << std::endl;;
         */
        if ( left_up * left_down > 0 )
          y += y_step;
        else if ( left_up * right_up > 0 )
          x += x_step;
        else {
          x += x_step; y += y_step;
        }
        break;
    }

    if ( reach_end ) break;

    /*
     *std::cout << "x: " << x
     *  << ", y: " << y
     *  << ", score: " << score << std::endl;
     */
  }

  //}

  return score;

}
}  // end namespace micvision
