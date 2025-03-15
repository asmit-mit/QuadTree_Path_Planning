#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <path_planner/AstarPlanner.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

using std::vector;

ros::Publisher marker_pub;
std::string global_frame_id = "map";
std::vector<std::pair<int, int>> current_path;
std::vector<std::vector<int>> current_grid;
nav_msgs::OccupancyGrid::ConstPtr current_map;
bool path_available = false;

void visualizePath() {
  if (!path_available || current_path.empty() || !current_map) {
    return;
  }

  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker path_line;
  path_line.header.frame_id = global_frame_id;
  path_line.header.stamp = ros::Time::now();
  path_line.ns = "path";
  path_line.id = 0;
  path_line.type = visualization_msgs::Marker::LINE_STRIP;
  path_line.action = visualization_msgs::Marker::ADD;

  path_line.scale.x = 0.1;

  path_line.color.r = 1.0;
  path_line.color.g = 0.0;
  path_line.color.b = 1.0;
  path_line.color.a = 1.0;

  path_line.pose.orientation.w = 1.0;

  double half_cell = current_map->info.resolution / 2.0;

  for (const auto &point : current_path) {
    geometry_msgs::Point p;
    p.x = point.first * current_map->info.resolution +
          current_map->info.origin.position.x + half_cell;
    p.y = point.second * current_map->info.resolution +
          current_map->info.origin.position.y + half_cell;
    p.z = 0.1;

    path_line.points.push_back(p);
  }

  visualization_msgs::Marker start_point;
  start_point.header.frame_id = global_frame_id;
  start_point.header.stamp = ros::Time::now();
  start_point.ns = "endpoints";
  start_point.id = 1;
  start_point.type = visualization_msgs::Marker::SPHERE;
  start_point.action = visualization_msgs::Marker::ADD;

  start_point.pose.position.x =
      current_path.front().first * current_map->info.resolution +
      current_map->info.origin.position.x + half_cell;
  start_point.pose.position.y =
      current_path.front().second * current_map->info.resolution +
      current_map->info.origin.position.y + half_cell;
  start_point.pose.position.z = 0.1;
  start_point.pose.orientation.w = 1.0;

  start_point.scale.x = 1.0;
  start_point.scale.y = 1.0;
  start_point.scale.z = 1.0;

  start_point.color.r = 0.0;
  start_point.color.g = 0.0;
  start_point.color.b = 1.0;
  start_point.color.a = 1.0;

  visualization_msgs::Marker end_point = start_point;
  end_point.id = 2;
  end_point.pose.position.x =
      current_path.back().first * current_map->info.resolution +
      current_map->info.origin.position.x + half_cell;
  end_point.pose.position.y =
      current_path.back().second * current_map->info.resolution +
      current_map->info.origin.position.y + half_cell;

  end_point.color.r = 1.0;
  end_point.color.g = 0.0;
  end_point.color.b = 0.0;

  marker_array.markers.push_back(path_line);
  marker_array.markers.push_back(start_point);
  marker_array.markers.push_back(end_point);

  marker_pub.publish(marker_array);
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  ROS_INFO("Received Goal data:");
  ROS_INFO("X: %f Y: %f", msg->pose.position.x, msg->pose.position.y);

  int end_x = (msg->pose.position.x - current_map->info.origin.position.x) /
              current_map->info.resolution;
  int end_y = (msg->pose.position.y - current_map->info.origin.position.y) /
              current_map->info.resolution;

  if (end_x < 0 || end_x >= current_map->info.width || end_y < 0 ||
      end_y >= current_map->info.height) {
    ROS_INFO("Out of bounds pose, cannot plan a path");
  }
  ROS_INFO("Map X: %d Y: %d", end_x, end_y);

  AstarPlanner planner(current_grid);

  current_path = planner.plan(0, 0, end_x, end_y);
  path_available = true;
  ROS_INFO("Planned path with %ld steps", current_path.size());
}

void timerCallback(const ros::TimerEvent &) { visualizePath(); }

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  ROS_INFO("Received map metadata:");
  ROS_INFO("Width: %d, Height: %d", msg->info.width, msg->info.height);
  current_map = msg;

  int width = msg->info.width;
  int height = msg->info.height;
  vector<vector<int>> grid(height, vector<int>(width));

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int index = y * width + x;
      grid[y][x] = msg->data[index];
    }
  }

  current_grid = grid;

  ROS_INFO("Map built successfully.");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_visualizer_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("global_frame_id", global_frame_id, "map");
  double publish_rate;
  private_nh.param<double>("publish_rate", publish_rate, 10.0);

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("path_markers", 1);

  ros::Timer timer =
      nh.createTimer(ros::Duration(1.0 / publish_rate), timerCallback);

  ros::Subscriber map_sub = nh.subscribe("/map", 10, mapCallback);
  ros::Subscriber goal_sub =
      nh.subscribe("/move_base_simple/goal", 10, goalCallback);

  ROS_INFO("Path visualizer started. Publishing at %.1f Hz", publish_rate);

  ros::spin();
  return 0;
}
