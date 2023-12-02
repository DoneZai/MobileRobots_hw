
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <algorithm>

// global publishers
ros::Publisher throttle_pub;
ros::Publisher err_pub;
ros::Publisher filtered_pub;
// gobal variables
double desired_velocity = 0;
double current_velocity = 0;
ros::Time last_timer_time;

double filtered_current_velocity = 0;

void on_command_velocity(const std_msgs::Float32 &msg)
{
  desired_velocity = msg.data;
  std_msgs::Float32 err;
  // err.data = desired_velocity - current_velocity;
  err.data =
    desired_velocity -
    filtered_current_velocity;  // change the error to error between the desired and the filtered
  // ROS_INFO_STREAM_COND(cmd_velocity > 0.1, "current velocity " << current_velocity << " err = "
  // << err.data);
  err_pub.publish(err);
  std_msgs::Float32 filtered_v;
  filtered_v.data = filtered_current_velocity;
  filtered_pub.publish(filtered_v);
}

void on_odo(const nav_msgs::Odometry &odom)
{
  current_velocity = odom.twist.twist.linear.x;
  // ROS_INFO_STREAM("current velocity " << current_velocity << " err = " << cmd_velocity -
  // current_velocity);
}

double last_error = desired_velocity - current_velocity;
double error_integral;
std::vector<double> velocity_history;
const int window_size = 18;  // size of window of moving average filter

// Define Gaussian function
double gaussian(double x, double mean, double stddev)
{
  return exp(-0.5 * pow((x - mean) / stddev, 2)) / (stddev * sqrt(2 * M_PI));
}

// Define Gaussian filter function,
double gaussian_filter(const std::vector<double> &data, int index, int window_size, double stddev)
{
  double result = 0.0;
  double total_weight = 0.0;

  // Calculate weight and weighted average
  for (int i = -window_size / 2; i <= window_size / 2; ++i) {
    if (index + i >= 0 && index + i < data.size()) {
      double weight = gaussian(i, 0, stddev);
      result += data[index + i] * weight;
      total_weight += weight;
    }
  }

  return result / total_weight;
}

void on_timer(const ros::TimerEvent &event)
{
  double p_factor = 50;
  double d_factor = 0.1;
  double i_factor = 0;

  auto t = ros::Time::now();
  auto dt = (t - last_timer_time).toSec();
  last_timer_time = t;
  std_msgs::Float32 throttle_cmd;

  // place code here to calculate throttle/brake
  velocity_history.push_back(current_velocity);

  // use moving average filter to processes the velocity
  // delete the earliest value if windows is full
  if (velocity_history.size() > window_size) {
    velocity_history.erase(velocity_history.begin());
  }

  // calculate the average velocity in this windows
  for (double v : velocity_history) {
    filtered_current_velocity += v;
  }
  filtered_current_velocity /= velocity_history.size();

  // Set standard deviation
  double stddev = 10;
  double filtered_current_velocity =
    gaussian_filter(velocity_history, velocity_history.size() - 1, window_size, stddev);

  double error = desired_velocity - filtered_current_velocity;
  // double error = desired_velocity - current_velocity;
  double diff_err = error - last_error;
  last_error = error;
  error_integral += error;

  double throttle = p_factor * error + d_factor * diff_err + i_factor * error_integral;

  throttle_cmd.data = throttle;
  throttle_pub.publish(throttle_cmd);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "velocity_controller");
  ros::NodeHandle nh("~");
  throttle_pub = nh.advertise<std_msgs::Float32>("throttle", 1);
  auto odo_sub = nh.subscribe("odom", 1, on_odo);
  auto cmd_sub = nh.subscribe("velocity", 1, on_command_velocity);
  err_pub = nh.advertise<std_msgs::Float32>("velocity_err", 1);
  filtered_pub = nh.advertise<std_msgs::Float32>("filtered_velocity", 1);
  if (nh.param("/use_sim_time", false)) {
    while (ros::ok()) {
      ros::spinOnce();

      last_timer_time = ros::Time::now();
      if (!last_timer_time.isZero()) {
        break;
      }
    }
  }
  auto timer = nh.createTimer(ros::Duration(0.1), on_timer);
  last_timer_time = ros::Time::now();
  ros::spin();
  return 0;
}
