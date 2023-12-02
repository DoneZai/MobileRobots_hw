#include "slam.h"
#include <angles/angles.h>
#include <sstream>
#include <math.h>

void Slam::on_odo(const nav_msgs::Odometry &odom)
{
  v = odom.twist.twist.linear.x;
  w = odom.twist.twist.angular.z;
}

void Slam::add_landmark(const sensor_msgs::LaserScan &scan, std::size_t start, std::size_t finish)
{
  // добавляем только особенные  точки на которые попало более 2 лучей
  if (finish - start < 2) {
    return;
  }
  // ROS_INFO_STREAM("Add landmark between " << start << " and " << finish);
  // TODO Здесь должен быть код определения координаты центра круглого препятствия
  size_t j = start;

  for (size_t i = start; i <= finish; ++i) {
    if (scan.ranges[i] < scan.ranges[j]) {
      j = i;
    }
  }
  double angle = scan.angle_min + j * scan.angle_increment;
  // angle = angles::normalize_angle(angle);
  double dis = scan.ranges[j] + radiusCylinder;
  double x = dis * cos(angle);
  double y = dis * sin(angle);

  // добавляем в вектор особенных точек
  new_landmarks.push_back(Eigen::Vector2d(x, y));
  z_landmarks.push_back(Eigen::Vector2d(dis, angle));
}

void Slam::detect_landmarks(const sensor_msgs::LaserScan &scan)
{
  new_landmarks.clear();
  z_landmarks.clear();
  // TODO Здесь должен быть код для пределения особенные точек скана
  // ищем начальный и конечный индексы лучей, падающих на одно препятствие
  // и вызываем add_landmark
  size_t start, finish;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    if (scan.ranges[i] < scan.range_max) {
      start = i;
      for (size_t j = start; scan.ranges[j] < scan.ranges[i] + 1; ++j) {
        finish = j;
      }
      add_landmark(scan, start, finish);
      i = finish + 1;
    }
  }
}

int Slam::associate_measuriment(const Eigen::Vector2d &landmark_measuriment)
{
  double nearest_distance = 1e10;
  int nearest_index = -1;
  // преобразование от СК карты к СК робота (дальномера)
  Eigen::Isometry2d robot_to_map = Eigen::Translation2d(X.segment(0, 2)) * Eigen::Rotation2Dd(X(2));
  for (std::size_t i = 0; i < landmarks_found_quantity; ++i) {
    double distance =
      (robot_to_map * landmark_measuriment - X.segment(ROBOT_STATE_SIZE + i * 2, 2)).norm();
    if (distance < nearest_distance) {
      nearest_index = i;
      nearest_distance = distance;
    }
  }
  // naive association
  const double kAssocThreshold = 5.0;
  if (nearest_index >= 0 && nearest_distance < kAssocThreshold) {
    return nearest_index;
    // ROS_INFO_STREAM("Find an accociated marker ");
  }
  // ROS_INFO_STREAM("Find a new marker " << nearest_distance);
  return -1;
}

int Slam::add_landmark_to_state(std::size_t i)
{
  ++landmarks_found_quantity;
  // TODO init landmark in state
  // Здесь должен быть код по инициализации части вектора состояния, соответствующей
  // маяку с индексом last_found_landmark_index
  Eigen::Isometry2d robot_to_map = Eigen::Translation2d(X.segment(0, 2)) * Eigen::Rotation2Dd(X(2));
  Eigen::Vector2d landmarks_map = robot_to_map * new_landmarks[i];
  // ROS_INFO_STREAM(
  //   "positions " << landmarks_map(0) << " " << landmarks_map(1) << " "
  //                << z_landmarks[i](0) * cos(z_landmarks[i](1)) + X(0) << " "
  //                << z_landmarks[i](1) * sin(z_landmarks[i](1)) + X(1));
  ROS_INFO_STREAM("landmarks_found_quantity " << landmarks_found_quantity);
  X(2 * landmarks_found_quantity + ROBOT_STATE_SIZE - 2) = landmarks_map(0);
  X(2 * landmarks_found_quantity + ROBOT_STATE_SIZE - 1) = landmarks_map(1);
  double dis = z_landmarks[i](0);
  double angle = z_landmarks[i](1);
  double normAngle = angles::normalize_angle(angle) + X(2);
  normAngle = angles::normalize_angle(normAngle);
  // 对于机器人位姿X的雅克比
  Eigen::MatrixXd Fx;
  Fx = (Eigen::MatrixXd::Zero(2, ROBOT_STATE_SIZE));
  Fx(0, 0) = 1;
  Fx(0, 1) = 0;
  Fx(0, 2) = -dis * sin(normAngle);
  Fx(1, 0) = 0;
  Fx(1, 1) = 1;
  Fx(1, 2) = dis * cos(normAngle);
  // 对于地标位置观测Z的雅克比
  Eigen::MatrixXd Fz;
  Fz = (Eigen::Matrix2d::Zero(2, 2));
  Fz(0, 0) = cos(normAngle);
  Fz(0, 1) = -dis * sin(normAngle);
  Fz(1, 0) = sin(normAngle);
  Fz(1, 1) = dis * cos(normAngle);

  int N = landmarks_found_quantity * 2 + 1;

  P.block(N, N, 2, 2) = Fx * P.topLeftCorner(ROBOT_STATE_SIZE, ROBOT_STATE_SIZE) * Fx.transpose() +
                        Fz * Q * Fz.transpose();

  Eigen::MatrixXd P_temp;
  Eigen::MatrixXd Fx_temp;

  Fx_temp = Eigen::MatrixXd::Zero(2, N);
  Fx_temp.block(0, 0, 2, ROBOT_STATE_SIZE) = Fx;

  P_temp.resize(2 * landmarks_found_quantity + 1, 2 * landmarks_found_quantity + 1);
  P_temp.setZero();
  P_temp = Fx_temp * P.topLeftCorner(N, N);
  P.block(N, 0, 2, N) = P_temp;

  P.block(0, N, N, 2) = P_temp.transpose();

  return landmarks_found_quantity;
}

void Slam::correct(int index, const Eigen::Vector2d &landmark_measuriment, int i)
{
  // TODO
  // Здесь должен быть код для обновления состояния по измерению iого маяка
  // Eigen::Isometry2d robot_to_map = Eigen::Translation2d(X.segment(0, 2)) *
  // Eigen::Rotation2Dd(X(2)); Eigen::Vector2d landmarks_map = robot_to_map * landmark_measuriment;

  double delta_x = X(ROBOT_STATE_SIZE + 2 * index) - X(0);
  double delta_y = X(ROBOT_STATE_SIZE + 2 * index + 1) - X(1);
  double dis = sqrt(delta_x * delta_x + delta_y * delta_y);
  Eigen::MatrixXd Htemp;
  Htemp = (Eigen::MatrixXd::Zero(2, 2 + ROBOT_STATE_SIZE));
  Htemp(0, 0) = -delta_x / dis;
  Htemp(0, 1) = -delta_y / dis;
  Htemp(0, 2) = 0;
  Htemp(0, 3) = delta_x / dis;
  Htemp(0, 4) = delta_y / dis;
  Htemp(1, 0) = delta_y / (dis * dis);
  Htemp(1, 1) = -delta_x / (dis * dis);
  Htemp(1, 2) = -1;
  Htemp(1, 3) = -delta_y / (dis * dis);
  Htemp(1, 4) = delta_x / (dis * dis);
  Eigen::Vector2d Z_hat;
  Z_hat(0) = dis;
  Z_hat(1) = angles::normalize_angle(atan2(delta_y, delta_x) - X(2));
  Eigen::MatrixXd F;
  F = (Eigen::MatrixXd::Zero(ROBOT_STATE_SIZE + 2, 2 * NUMBER_LANDMARKS + ROBOT_STATE_SIZE));
  F.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity(3, 3);
  F.block(3, 2 * index + 3, 2, 2) = Eigen::Matrix2d::Identity(2, 2);
  H.resize(2, 2 * NUMBER_LANDMARKS + ROBOT_STATE_SIZE);
  H.setZero();
  H = Htemp * F;
  K.resize(2 * NUMBER_LANDMARKS + ROBOT_STATE_SIZE, 2);
  K.setZero();
  Eigen::Matrix2d Ftemp;
  Ftemp = (H * P * H.transpose() + Q);
  K = P * H.transpose() * (H * P * H.transpose() + Q).inverse();
  X = X + K * (z_landmarks[i] - Z_hat);
  P = (Eigen::MatrixXd::Identity(
         2 * NUMBER_LANDMARKS + ROBOT_STATE_SIZE, 2 * NUMBER_LANDMARKS + ROBOT_STATE_SIZE) -
       K * H) *
      P;
}

void Slam::on_scan(const sensor_msgs::LaserScan &scan)
{
  detect_landmarks(scan);
  predict((scan.header.stamp - last_time).toSec());
  last_time = scan.header.stamp;
  for (std::size_t i = 0; i < new_landmarks.size(); ++i) {
    const auto landmark_index = associate_measuriment(new_landmarks[i]);
    if (landmark_index >= 0) {
      // ROS_INFO_STREAM("Correcting......  " << landmark_index);
      correct(landmark_index, new_landmarks[i], i);
    } else {
      if (landmarks_found_quantity < NUMBER_LANDMARKS) {
        add_landmark_to_state(i);
      } else {
        ROS_ERROR_STREAM("can not associate new landmark with any existing one");
      }
    }
  }

  publish_results("map", scan.header.stamp);
  publish_transform(scan.header);
}

void fill_pose_msg(
  geometry_msgs::PoseWithCovariance &pose, double x, double y, double fi,
  const Eigen::Matrix2d &cov_matr)
{
  pose.covariance.assign(0);
  pose.covariance[0] = cov_matr(0, 0);
  pose.covariance[1] = cov_matr(0, 1);
  pose.covariance[6] = cov_matr(1, 0);
  pose.covariance[7] = cov_matr(1, 1);
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.w = cos(fi / 2);
  pose.pose.orientation.z = sin(fi / 2);
}

void fill_pose_msg(geometry_msgs::Pose &pose, double x, double y, double fi)
{
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.w = cos(fi / 2);
  pose.orientation.z = sin(fi / 2);
}

void Slam::publish_results(const std::string &frame, const ros::Time &time)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = time;
  // публикуем сообщение с позицией робота
  fill_pose_msg(pose.pose, X(0), X(1), X(2));
  pose_pub.publish(pose);

  // публикуем сообщения с положениями маяков
  for (int i = 0; i < landmarks_found_quantity; ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.header.stamp = time;
    fill_pose_msg(pose.pose, X(ROBOT_STATE_SIZE + i * 2), X(ROBOT_STATE_SIZE + i * 2 + 1), 0);
    landmark_pub[i].publish(pose);
  }
}

void Slam::publish_transform(const std_msgs::Header &scan_header)
{
  // публикуем трансформ от скана до карты,
  // не наоборот, так как дерево tf - однонаправленное
  tf::Transform tf_transform;
  double angle = X(2);
  Eigen::Matrix2d R;
  R(0, 0) = R(1, 1) = cos(angle);
  R(0, 1) = -sin(angle);
  R(1, 0) = sin(angle);
  Eigen::Vector2d t = X.head(2);
  Eigen::Isometry2d transform = Eigen::Translation2d(t) * Eigen::Isometry2d(R);

  Eigen::Isometry2d inverted_transform = transform.inverse();
  tf_transform.setOrigin(
    tf::Vector3(inverted_transform.translation().x(), inverted_transform.translation().y(), 0.0));
  tf::Quaternion inv_q;
  const auto &inv_matrix = inverted_transform.matrix();
  double inv_yaw = atan2(inv_matrix(1, 0), inv_matrix(0, 0));
  inv_q.setRPY(0, 0, inv_yaw);
  tf_transform.setRotation(inv_q);
  br.sendTransform(
    tf::StampedTransform(tf_transform, scan_header.stamp, scan_header.frame_id, map_frame));
}

void Slam::predict(double dt)
{
  // X(t+1) = g(t)
  X(0) += v * cos(X(2)) * dt;
  X(1) += v * sin(X(2)) * dt;
  X(2) += w * dt;
  X(2) = angles::normalize_angle(X(2));

  // вычисляем якобиан
  A = Eigen::Matrix3d::Identity();
  A(0, 0) = 1.0;
  A(0, 1) = 0;
  A(0, 2) = -v * sin(X(2)) * dt;
  A(1, 0) = 0.0;
  A(1, 1) = 1.0;
  A(1, 2) = v * cos(X(2)) * dt;
  A(2, 0) = 0.0;
  A(2, 1) = 0.0;
  A(2, 2) = 1.0;

  // P = A*P*AT + R для блока соответствующего роботу
  P.topLeftCorner(ROBOT_STATE_SIZE, ROBOT_STATE_SIZE) =
    A * P.topLeftCorner(ROBOT_STATE_SIZE, ROBOT_STATE_SIZE) * A.transpose() + R;
  // для остальных блоков
  P.topRightCorner(ROBOT_STATE_SIZE, NUMBER_LANDMARKS * 2) =
    A * P.topRightCorner(ROBOT_STATE_SIZE, NUMBER_LANDMARKS * 2);
  P.bottomLeftCorner(NUMBER_LANDMARKS * 2, ROBOT_STATE_SIZE) =
    P.topRightCorner(ROBOT_STATE_SIZE, NUMBER_LANDMARKS * 2).transpose();
}

void Slam::advertize_landmark_publishers()
{
  std::string landmark("landmark");
  for (int i = 0; i < NUMBER_LANDMARKS; ++i) {
    std::stringstream stream;
    stream << landmark << i;
    landmark_pub[i] = nh.advertise<geometry_msgs::PoseStamped>(stream.str(), 1);
  }
}

Slam::Slam()
: nh("~"),
  odo_sub(nh.subscribe("/odom", 1, &Slam::on_odo, this)),
  scan_sub(nh.subscribe("/scan", 1, &Slam::on_scan, this)),
  pose_pub(nh.advertise<geometry_msgs::PoseStamped>("slam_pose", 1)),
  X(ROBOT_STATE_SIZE + 2 * NUMBER_LANDMARKS),
  A(Eigen::Matrix3d::Identity()),
  P(Eigen::MatrixXd::Zero(X.size(), X.size()))
{
  // начальный вектор состояния заполняем нулями
  X = Eigen::VectorXd::Zero(X.size());
  // записываем огромное значение начальной ковариации для маяков
  P.bottomRightCorner(NUMBER_LANDMARKS * 2, NUMBER_LANDMARKS * 2) =
    HUGE_COVARIANCE * Eigen::MatrixXd::Identity(NUMBER_LANDMARKS * 2, NUMBER_LANDMARKS * 2);

  advertize_landmark_publishers();

  Q = Eigen::Matrix2d::Zero();
  Q(0, 0) = nh.param<double>("range_sigma_sqr", 0.01);
  Q(1, 1) = nh.param<double>("angle_sigma_sqr", 0.001);

  R = Eigen::Matrix3d::Zero();
  R(0, 0) = nh.param<double>("x_sigma_sqr", 0.01);
  R(1, 1) = nh.param<double>("y_sigma_sqr", 0.01);
  R(2, 2) = nh.param<double>("angle_sigma_sqr", 0.001);
}
