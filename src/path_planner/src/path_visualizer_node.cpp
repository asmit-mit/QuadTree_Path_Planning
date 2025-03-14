#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <path_planner/astar_planner.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <visualization_msgs/marker_array.h>

using std::vector;

ros::Publisher marker_pub;
std::string global_frame_id = "map";
std::vector<std::pair<int, int>> current_path;
nav_msgs::OccupancyGrid::ConstPtr current_map;
bool path_available = false;

void visualizePath() {
  if (!path_available || current_path.empty() || !current_map) {
    return;
  }

  visualization_msgs::marker_array marker_array;

  visualization_msgs::Marker pathLine;
  pathLine.header.frame_id = global_frame_id;
  pathLine.header.stamp = ros::Time::now();
  pathLine.ns = "path";
  pathLine.id = 0;
  pathLine.type = visualization_msgs::Marker::LINE_STRIP;
  pathLine.action = visualization_msgs::Marker::ADD;

  pathLine.scale.x = 0.05;

  pathLine.color.r = 0.0;
  pathLine.color.g = 1.0;
  pathLine.color.b = 0.0;
  pathLine.color.a = 1.0;

  pathLine.pose.orientation.w = 1.0;

  for (const auto &point : current_path) {
    geometry_msgs::Point p;
    p.x = point.first * current_map->info.resolution +
          current_map->info.origin.position.x;
    p.y = point.second * current_map->info.resolution +
          current_map->info.origin.position.y;
    p.z = 0.1;

    pathLine.points.push_back(p);
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
      current_map->info.origin.position.x;
  start_point.pose.position.y =
      current_path.front().second * current_map->info.resolution +
      current_map->info.origin.position.y;
  start_point.pose.position.z = 0.1;
  start_point.pose.orientation.w = 1.0;

  start_point.scale.x = 0.2;
  start_point.scale.y = 0.2;
  start_point.scale.z = 0.2;

  start_point.color.r = 0.0;
  start_point.color.g = 0.0;
  start_point.color.b = 1.0;
  start_point.color.a = 1.0;

  visualization_msgs::Marker end_point = start_point;
  end_point.id = 2;
  end_point.pose.position.x =
      current_path.back().first * current_map->info.resolution +
      current_map->info.origin.position.x;
  end_point.pose.position.y =
      current_path.back().second * current_map->info.resolution +
      current_map->info.origin.position.y;

  end_point.color.r = 1.0;
  end_point.color.g = 0.0;
  end_point.color.b = 0.0;

  marker_array.markers.push_back(pathLine);
  marker_array.markers.push_back(start_point);
  marker_array.markers.push_back(end_point);

  marker_pub.publish(marker_array);
}

void timerCallback(const ros::TimerEvent &) { visualizePath(); }

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  ROS_INFO("Received map metadata:");
  ROS_INFO("Width: %d, Height: %d", msg->info.width, msg->info.height);

  int width = msg->info.width;
  int height = msg->info.height;
  vector<vector<int>> grid(height, vector<int>(width));

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int index = y * width + x;
      grid[y][x] = msg->data[index];
    }
  }

  ROS_INFO("Map built successfully.");

  AstarPlanner planner(grid);

  std::vector<std::pair<int, int>> path =
      planner.plan(0, 0, grid[0].size() - 1, grid.size() - 1);

  ROS_INFO("Planned path with %ld steps.", path.size());

  if (!path.empty()) {
    current_path = path;
    current_map = msg;
    path_available = true;
    ROS_INFO("Path planning successful, ready for visualization.");
  } else {
    ROS_WARN("Path planning failed, no path to visualize.");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_visualizer_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("global_frame_id", global_frame_id, "map");
  double publish_rate;
  private_nh.param<double>("publish_rate", publish_rate, 10.0);

  marker_pub =
      nh.advertise<visualization_msgs::marker_array>("path_markers", 1);

  ros::Timer timer =
      nh.createTimer(ros::Duration(1.0 / publish_rate), timerCallback);

  ros::Subscriber map_sub = nh.subscribe("/map", 10, mapCallback);

  ROS_INFO("Path visualizer started. Publishing at %.1f Hz", publish_rate);

  ros::spin();
  return 0;
}
