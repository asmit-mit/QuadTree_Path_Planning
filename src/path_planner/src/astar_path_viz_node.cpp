#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <path_planner/AstarPlanner.h>
#include <path_planner/AstarQuadTreePlanner.h>
#include <quadtrees/QuadTree.h>
#include <ros/ros.h>
#include <string>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

using std::vector;

ros::Publisher marker_pub;
ros::Publisher quadtree_marker_pub;
std::string global_frame_id = "map";
std::vector<std::pair<int, int>> current_path;
std::vector<std::pair<int, int>> current_path_quadtree;
std::vector<std::vector<int>> current_grid;
nav_msgs::OccupancyGrid::ConstPtr current_map;

QuadTree *quadtree = nullptr;

void visualizePath(std::vector<std::pair<int, int>> path, ros::Publisher pub) {
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

  for (const auto &point : path) {
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
      path.front().first * current_map->info.resolution +
      current_map->info.origin.position.x + half_cell;
  start_point.pose.position.y =
      path.front().second * current_map->info.resolution +
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
  end_point.pose.position.x = path.back().first * current_map->info.resolution +
                              current_map->info.origin.position.x + half_cell;
  end_point.pose.position.y =
      path.back().second * current_map->info.resolution +
      current_map->info.origin.position.y + half_cell;

  end_point.color.r = 1.0;
  end_point.color.g = 0.0;
  end_point.color.b = 0.0;

  marker_array.markers.push_back(path_line);
  marker_array.markers.push_back(start_point);
  marker_array.markers.push_back(end_point);

  pub.publish(marker_array);
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
    return;
  }
  ROS_INFO("Map X: %d Y: %d", end_x, end_y);

  AstarPlanner planner(current_grid);
  AstarQuadTreePlanner planenrq(quadtree);

  /* current_path = planner.plan(0, 0, end_x, end_y); */
  current_path_quadtree = planenrq.plan(0, 0, end_x, end_y);
  ROS_INFO("General Astar planned path with %ld steps", current_path.size());
  ROS_INFO("QuadTree Astar planned path with %ld steps",
           current_path_quadtree.size());
}

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

  if (!quadtree) {
    quadtree = new QuadTree();
  }

  quadtree->build(grid);
  current_grid = grid;

  ROS_INFO("Built map with %dx%d in %d node QuadTree.", width, height,
           quadtree->getNumLeaves());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "astar_path_viz_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("global_frame_id", global_frame_id, "map");
  double publish_rate;
  private_nh.param<double>("publish_rate", publish_rate, 10.0);

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("path_markers", 1);
  quadtree_marker_pub =
      nh.advertise<visualization_msgs::MarkerArray>("quadtree_path_markers", 1);

  ros::Subscriber map_sub = nh.subscribe("/map", 10, mapCallback);
  ros::Subscriber goal_sub =
      nh.subscribe("/move_base_simple/goal", 10, goalCallback);

  ROS_INFO("Path visualizer started. Publishing at %.1f Hz", publish_rate);

  ros::Rate rate(publish_rate);
  while (ros::ok()) {
    if (current_path.size() != 0)
      visualizePath(current_path, marker_pub);
    if (current_path_quadtree.size() != 0)
      visualizePath(current_path_quadtree, quadtree_marker_pub);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
