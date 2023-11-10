
#include "planner.h"

#include <cstddef>
#include <queue>
#include <set>
#include <utility>

namespace simple_planner
{

const MapIndex neighbors[8] = { {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
const int8_t kObstacleValue = 100;
const int8_t coststep = 1; 
const int8_t coststart = 0; 

Planner::Planner(ros::NodeHandle& nh) :
 nh_(nh)
{
  while(!map_server_client_.waitForExistence(ros::Duration(1))) {
    ROS_INFO_STREAM("Wait map server");
  }
  ROS_INFO_STREAM("Service connected");
}

void Planner::on_pose(const nav_msgs::Odometry& odom)
{
  start_pose_ = odom.pose.pose;
}

void Planner::on_target(const geometry_msgs::PoseStamped& pose)
{
  ROS_INFO_STREAM("Get goal " << pose.pose.position.x << " " << pose.pose.position.y);
  ROS_INFO_STREAM("Start is " << start_pose_.position.x << " " << start_pose_.position.y);
  target_pose_ = pose.pose;

  if (!update_static_map() )
  {
    ROS_ERROR_STREAM("Can not receive map");
    return ;
  }

  increase_obstacles(ceil(robot_radius_/map_.info.resolution));
  obstacle_map_publisher_.publish(obstacle_map_);
  cost_map_publisher_.publish(cost_map_);
  // wave_search();

  calculate_path();

  if (!path_msg_.points.empty()) {
    path_msg_.header.stamp = ros::Time::now();
    path_msg_.header.frame_id = pose.header.frame_id;
    path_publisher_.publish(path_msg_);
  } else {
  	ROS_WARN_STREAM("Path not found!");
  }
}

bool Planner::update_static_map()
{
  nav_msgs::GetMap service;
  if (!map_server_client_.call(service))
  {
    ROS_ERROR_STREAM("Failed to receive a map");
    return false;
  }
  map_ = service.response.map;
  ROS_INFO_STREAM("Map received : " << map_.info.width << " " << map_.info.height);
  return true;
}

bool Planner::indices_in_map(int i, int j)
{
  return i >= 0 && j >= 0 && i < map_.info.width && j < map_.info.height;
}

void Planner::increase_obstacles(std::size_t cells)
{
  obstacle_map_.info = map_.info;
  obstacle_map_.header = map_.header;
  obstacle_map_.data.resize(map_.data.size());
  obstacle_map_.data = map_.data;

  std::queue<MapIndex> wave;
  for (int i = 0; i < map_.info.width; ++i)
  {
    for (int j = 0; j < map_.info.height; ++j)
    {
      if (map_value(map_.data, i, j) != kObstacleValue)
      {
        continue;
      }
      // else - obstacle
      // check neighbors
      for(const auto& shift : neighbors)
      {
        int neighbor_i = i + shift.i;
        int neighbor_j = j + shift.j;
        if (!indices_in_map(neighbor_i, neighbor_j))
        {
          continue;
        }
        // if neighbor is not obstacle - add i, j to wave
        if (map_value(map_.data, neighbor_i, neighbor_j) != kObstacleValue)
        {
          wave.push({i, j});
          break;
        }
      }
    }
  }
  ROS_INFO_STREAM("Start wave size = " << wave.size());
  for(std::size_t step = 0; step < cells; ++step)
  {
    std::queue<MapIndex> next_wave;
    while(!wave.empty()) {
      auto indices = wave.front();
      wave.pop();
      for(const auto& shift : neighbors)
      {
        auto neighbor_index = indices;
        neighbor_index.i += shift.i;
        neighbor_index.j += shift.j;
        if (!indices_in_map(neighbor_index.i, neighbor_index.j))
        {
          continue;
        }
        if (map_value(obstacle_map_.data, neighbor_index.i, neighbor_index.j) != kObstacleValue)
        {
          map_value(obstacle_map_.data, neighbor_index.i, neighbor_index.j) = kObstacleValue;
          next_wave.push(neighbor_index);
        }
      }
    } // wave empty
    std::swap(wave, next_wave);
    ROS_INFO_STREAM("Wave size = " << wave.size());
  }
//make cost map 
    cost_map_.info = map_.info;
    cost_map_.header = map_.header;
    cost_map_.data.resize(map_.data.size());
    cost_map_.data = map_.data;

    for (int i = 0; i < cost_map_.info.width; ++i)
    {
      for (int j = 0; j < cost_map_.info.height; ++j){
        if(map_value(obstacle_map_.data, i, j) == kObstacleValue){
          map_value(cost_map_.data, i, j) = 100;}
        else{
          map_value(cost_map_.data, i, j) = 1;}
        }
    }
    ROS_INFO_STREAM("Cost map finished" );
}

class CompareSearchNodes {
public:
  explicit CompareSearchNodes(Planner& planner): planner_(planner) {}
  bool operator () (const MapIndex& left_index, const MapIndex& right_index) const {
  	SearchNode& left = planner_.map_value(planner_.search_map_, left_index.i, left_index.j);
  	SearchNode& right = planner_.map_value(planner_.search_map_, right_index.i, right_index.j);
    if (left.g + left.h == right.g + right.h) {
    	if (left_index.i == right_index.i) {
    		return left_index.j < right_index.j;
    	}
    	return left_index.i < right_index.i;
    }
    return left.g + left.h < right.g + right.h;
  }
private:
  Planner& planner_;
};


// 波搜索算法
void Planner::wave_search(){
    MapIndex start_wave = point_index(start_pose_.position.x, start_pose_.position.y);
    MapIndex target_index = point_index(target_pose_.position.x, target_pose_.position.y);
    
    search_map_.resize(map_.data.size());
    std::fill(search_map_.begin(), search_map_.end(), SearchNode());
    path_msg_.points.clear();
    
    std::queue<MapIndex> queue_wave;
    auto& start_obstacle_value = map_value(obstacle_map_.data, start_wave.i, start_wave.j);
    if (start_obstacle_value == kObstacleValue) {
      ROS_WARN_STREAM("Start is in obstacle!");
      return;
    }
    
    map_value(search_map_, start_wave.i, start_wave.j).g = 0;
    map_value(search_map_, start_wave.i, start_wave.j).state = SearchNode::OPEN;
    queue_wave.push(start_wave);

    ROS_INFO_STREAM("Begin to make search map");  
    bool found = false;
    while (!found && !queue_wave.empty()) {
      std::queue<MapIndex> queue_nextwave;

      while (!queue_wave.empty()) {
        auto current_point = queue_wave.front();
        map_value(search_map_, current_point.i, current_point.j).state = SearchNode::CLOSE;
        queue_wave.pop();

        for (const auto& shift : neighbors) 
        {   
            auto neighbor_index = current_point;
            neighbor_index.i += shift.i;
            neighbor_index.j += shift.j;
            // out of boundary?
            if (!indices_in_map(neighbor_index.i, neighbor_index.j)) 
            {
              continue;
            }
            //is obstacle?
            if(map_value(obstacle_map_.data,neighbor_index.i,neighbor_index.j) == kObstacleValue)
            {
              map_value(search_map_, neighbor_index.i, neighbor_index.j).state = SearchNode::CLOSE;  
              continue;
            }
            //calculate g_neighbor,possible be sqrt(2)
            double g_neighbor;
            if(shift.i*shift.j!=0){
                g_neighbor = map_value(search_map_, current_point.i, current_point.j).g + 1.4; //斜着走
              }
            else{
                g_neighbor = map_value(search_map_, current_point.i, current_point.j).g + 1;
              }
            auto& neighbor = map_value(search_map_, neighbor_index.i, neighbor_index.j);

            if( neighbor_index.i ==  target_index.i && neighbor_index.j == target_index.j ){
              found = true;
              neighbor.g = g_neighbor;
              neighbor.state = SearchNode::CLOSE;
              neighbor.iParent = current_point.i;
              neighbor.jParent = current_point.j;
              break;
            }
              if(neighbor.state == SearchNode::UNDEFINED){
                neighbor.g = g_neighbor;
                neighbor.iParent = current_point.i;
                neighbor.jParent = current_point.j;
                queue_wave.push(neighbor_index);
              }
              else{
                if (g_neighbor < neighbor.g) {
                    neighbor.g = g_neighbor;
                    neighbor.iParent = current_point.i;
                    neighbor.jParent = current_point.j;
              }
                continue;
              }

              neighbor.g = g_neighbor;
              neighbor.state = SearchNode::OPEN;
              neighbor.iParent = current_point.i;
              neighbor.jParent = current_point.j;
              queue_wave.push(neighbor_index);
        }
      }
    }
  ROS_INFO_STREAM("Search map finished" );

  if (found) {
  	int i = target_index.i;
  	int j = target_index.j;
  	geometry_msgs::Point32 p;
    p.x = i * map_.info.resolution + map_.info.origin.position.x;
    p.y = j * map_.info.resolution + map_.info.origin.position.y;
    path_msg_.points.push_back(p);
  	while (i != start_wave.i || j != start_wave.j) {
  		auto& node = map_value(search_map_, i, j);
      i = node.iParent;
      j = node.jParent;
      p.x = i * map_.info.resolution + map_.info.origin.position.x;
      p.y = j * map_.info.resolution + map_.info.origin.position.y;
      path_msg_.points.push_back(p);
  		ROS_INFO_STREAM("i = "<< i <<" j = " << j << " g = " << node.g);
  	}
  }
  ROS_INFO_STREAM("Wave path has been searched " );

}

//  启发式函数
double Planner::heruistic(int i, int j) {
  MapIndex target_index = point_index(target_pose_.position.x, target_pose_.position.y);
  return abs(i-target_index.i)+abs(j-target_index.j);
}


void Planner::calculate_path()
{
  // очищаем карту поиска
  search_map_.resize(map_.data.size());
  std::fill(search_map_.begin(), search_map_.end(), SearchNode());
  path_msg_.points.clear();

  // Здесь необходимо продолжить код для поиска пути
  std::set<MapIndex, CompareSearchNodes> queue(CompareSearchNodes(*this));
  MapIndex start_index = point_index(start_pose_.position.x, start_pose_.position.y);
  MapIndex target_index = point_index(target_pose_.position.x, target_pose_.position.y);

  SearchNode& start = map_value(search_map_, start_index.i, start_index.j);
  start.g = 0;
  start.h = heruistic(start_index.i, start_index.j);
  start.state = SearchNode::OPEN;

  auto& start_obstacle_value = map_value(obstacle_map_.data, start_index.i, start_index.j);
  if (start_obstacle_value == kObstacleValue) {
  	ROS_WARN_STREAM("Start is in obstacle!");
  	return;
  }
  queue.insert(start_index);

  bool found = false;
  while (!found && !queue.empty()) {
  	// insert code here
    ROS_INFO_STREAM("Begin to make search map");  
    while (!queue.empty()) {
        auto node_index_iter = queue.begin();
        auto current_point = *node_index_iter;
        auto& node = map_value(search_map_, current_point.i, current_point.j);
        node.state = SearchNode::CLOSE;
        queue.erase(node_index_iter);

        for (const auto& shift : neighbors) 
        {   
            auto neighbor_index = current_point;
            neighbor_index.i += shift.i;
            neighbor_index.j += shift.j;
            // out of boundary?
            if (!indices_in_map(neighbor_index.i, neighbor_index.j)) 
            {
              continue;
            }
            //is obstacle?
            if(map_value(obstacle_map_.data,neighbor_index.i,neighbor_index.j) == kObstacleValue)
            {
              map_value(search_map_, neighbor_index.i, neighbor_index.j).state = SearchNode::CLOSE;  
              continue;
            }

            //calculate g_neighbor,possible be sqrt(2)
            double g_neighbor;
            if(shift.i*shift.j!=0){
                g_neighbor = map_value(search_map_, current_point.i, current_point.j).g + 1.4; //斜着走
              }
            else{
                g_neighbor = map_value(search_map_, current_point.i, current_point.j).g + 1;
              }
            auto& neighbor = map_value(search_map_, neighbor_index.i, neighbor_index.j);

            if( neighbor_index.i ==  target_index.i && neighbor_index.j == target_index.j ){
              found = true;
              neighbor.g = g_neighbor;
              neighbor.h = heruistic(start_index.i, start_index.j);
              neighbor.state = SearchNode::CLOSE;
              neighbor.iParent = current_point.i;
              neighbor.jParent = current_point.j;
              break;
            }
              if(neighbor.state == SearchNode::UNDEFINED){
                neighbor.g = g_neighbor;
                neighbor.iParent = current_point.i;
                neighbor.jParent = current_point.j;
              }
              else{
                if (g_neighbor < neighbor.g) {
                    neighbor.g = g_neighbor;
                    neighbor.iParent = current_point.i;
                    neighbor.jParent = current_point.j;
              }
                continue;
              }

              neighbor.g = g_neighbor;
              neighbor.h = heruistic(start_index.i, start_index.j);
              neighbor.state = SearchNode::OPEN;
              neighbor.iParent = current_point.i;
              neighbor.jParent = current_point.j;
              queue.insert(neighbor_index);
        }
    }
  ROS_INFO_STREAM("Search map finished" );

  // fill path message with points from path 
  if (found) {
  	int i = target_index.i;
  	int j = target_index.j;
  	geometry_msgs::Point32 p;
    p.x = i * map_.info.resolution + map_.info.origin.position.x;
    p.y = j * map_.info.resolution + map_.info.origin.position.y;
    path_msg_.points.push_back(p);
  	while (i != start_index.i || j != start_index.j) {
  		auto& node = map_value(search_map_, i, j);
      i = node.iParent;
      j = node.jParent;
      p.x = i * map_.info.resolution + map_.info.origin.position.x;
      p.y = j * map_.info.resolution + map_.info.origin.position.y;
      path_msg_.points.push_back(p);
  		// ROS_INFO_STREAM("i = "<< i <<" j = " << j << " g = " << node.g);
  	}
  }

    ROS_INFO_STREAM("A* path has been searched" );
  }

}

} /* namespace simple_planner */
