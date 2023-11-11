
#include "planner.h"

#include <cstddef>
#include <queue>
#include <set>
#include <utility>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Twist.h>

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
  if (!path_msg_.points.empty()) {
  movingonpath(path_msg_,0);
  path_publisher_.publish(path_msg_);
  }

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
    bezier_smooth(path_msg_);
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
  // ROS_INFO_STREAM("Start wave size = " << wave.size());
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
    // ROS_INFO_STREAM("Wave size = " << wave.size());
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
  		// ROS_INFO_STREAM("i = "<< i <<" j = " << j << " g = " << node.g);
  	}
    // size_t num_points = path_msg_.points.size();
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
    ROS_INFO_STREAM("Number of points: "<<path_msg_.points.size());
    ROS_INFO_STREAM("A* path has been searched" );
  }

}

// 计算组合数
int Planner::binomial(int n, int i) {
    int res = 1;
    for (int j = 1; j <= i; ++j) {
        res *= (n - j + 1) / static_cast<double>(j);
    }
    return res;
};

// 计算n次贝塞尔曲线上的点
geometry_msgs::Point32 Planner::bezier_curve(const std::vector<geometry_msgs::Point32>& vector_before, float t) {
    // int n = vector_before.size() - 1;
    int n = vector_before.size() - 1;
    geometry_msgs::Point32 res;
    for (int i = 0; i <= n; ++i) {
        float b = binomial(n, i) * pow(t, i) * pow(1 - t, n - i);
        res.x = res.x + vector_before[i].x * b;
        res.y = res.y + vector_before[i].y * b;
    }
    return res;
}

bool arePointsCollinear(const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2, const geometry_msgs::Point32& p3) {
    return std::fabs((p2.y - p1.y) * (p3.x - p2.x) - (p3.y - p2.y) * (p2.x - p1.x)) < 1e-3;
}

void removeCollinearPoints(std::vector<geometry_msgs::Point32>& points) {
    if (points.size() < 3) {
        return;  // 三点形成一条直线
    }

    std::vector<geometry_msgs::Point32> result;
    result.push_back(points[0]);  // 保留第一个点
    size_t m = 0;
    for (size_t i = 1; i < points.size() - 1; ++i) {
        if (!arePointsCollinear(points[m], points[i], points[i + 1])) {
            result.push_back(points[i]);
            m = i;
        }
    }

    result.push_back(points.back());  // 保留最后一个点
    points = result;
};

void Planner::bezier_smooth(sensor_msgs::PointCloud before_smooth) {
    std::vector<geometry_msgs::Point32> vector_before_all;
    vector_before_all = before_smooth.points;
    ROS_INFO_STREAM("Number of points: " << path_msg_.points.size());
    temp_path_msg_.points.clear();
    size_t group_size = 80; // 每组的点的数量
    float step = 0.005   ;  // 步长
    // 获取 before_smooth 中的点数
    size_t total_points = vector_before_all.size();

    // 循环处理每组点
    for (size_t start_index = 0; start_index < total_points; start_index += group_size) {
        // 构建当前组的 vector_before
        std::vector<geometry_msgs::Point32> vector_before;
        for (size_t i = start_index; i < std::min(start_index + group_size, total_points); ++i) {
            vector_before.push_back(vector_before_all[i]);
            // ROS_INFO_STREAM("i =  " << i);
            }
            // if(start_index!=0)start_index-=10;
            // ROS_INFO_STREAM("Number of points: " << vector_before.size());
            removeCollinearPoints(vector_before);
            // ROS_INFO_STREAM("After deleting: " << vector_before.size());
            // 对当前组进行贝塞尔平滑
            for (float t = 0; t <= 1; t += step) {
              geometry_msgs::Point32 p = bezier_curve(vector_before, t);
              temp_path_msg_.points.push_back(p);
        }
            vector_before.clear();

    }
    std::reverse(temp_path_msg_.points.begin(), temp_path_msg_.points.end());
    path_msg_ = temp_path_msg_;
    // ROS_INFO_STREAM("points1.x=" << path_msg_.points[0].x);
    // ROS_INFO_STREAM("_end.x=" << path_msg_.points[path_msg_.points.size()-1].x);
}

void Planner::movingonpath(const sensor_msgs::PointCloud& path , int i) {
    
  // 获取机器人当前四元数
  double roll,pitch,yaw;
  // tf2::Quaternion tf_quaternion;
  // tf_quaternion.setValue(start_pose_.orientation.x, start_pose_.orientation.y, start_pose_.orientation.z, start_pose_.orientation.w);
  // tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

      ROS_INFO_STREAM("path.points=" << path.points[i].x); 
      double distance = sqrt(pow(start_pose_.position.x-path.points[i].x,2)+pow(start_pose_.position.y-path.points[i].x,2));
      if(distance<0.01){
        path_msg_.points.erase(path_msg_.points.begin());
        return;
      }
      yaw = 2*atan2(start_pose_.orientation.z,start_pose_.orientation.w);
      // ROS_INFO_STREAM("yaw=" << yaw);
      double angle_to_target = atan2(path.points[i].y - start_pose_.position.y, path.points[i].x - start_pose_.position.x);
      double angle_difference = yaw - angle_to_target ;
      // 将夹角差限制在 [-pi, pi] 范围内
      // while (angle_difference > M_PI) {
      //     angle_difference -= 2.0 * M_PI;
      // }
      // while (angle_difference < -M_PI) {
      //     angle_difference += 2.0 * M_PI;
      // }
      // ROS_INFO_STREAM("wwwwww =" << start_pose_.orientation.w);
      ROS_INFO_STREAM("angle_difference=" << angle_difference);
      // ROS_INFO_STREAM("i =" << i);

      // std_msgs::Float32 cmd_v,cmd_steering;
      geometry_msgs::Twist cmd_v_steering;
      if(abs(angle_difference)>0.1){
        cmd_v_steering.linear.x=0;
        cmd_v_steering.angular.z=-0.3*angle_difference;
        v_ste_publisher_.publish(cmd_v_steering);
        ROS_INFO_STREAM("Steering...." );
      }
      else{
        std_msgs::Float32 cmd_v;
        cmd_v.data = 1;
        vel_publisher_.publish(cmd_v);
        cmd_v_steering.angular.z=0;
        v_ste_publisher_.publish(cmd_v_steering);
        ROS_INFO_STREAM("Moving...." );
      }

      // ros::Duration(1.0).sleep();  // 暂停一秒钟，可以根据需要调整
      // 若到达终点，则跳出循环或者发布速度为0的消息停止机器人

      // cmd_v_steering.linear.x=0;
      // cmd_v_steering.angular.z=0;
      // v_ste_publisher_.publish(cmd_v_steering);

}

} /* namespace simple_planner */

