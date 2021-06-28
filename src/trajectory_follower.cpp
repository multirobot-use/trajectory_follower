#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <handy_tools/pid_controller.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Eigen>
#include <chrono>

const float YAW_PID_P{0.4};
const float YAW_PID_I{0.02};
const float YAW_PID_D{0.4};

struct State {
  Eigen::Vector3f position{0, 0, 0};
  Eigen::Quaternionf orientation{0, 0, 0, 0};
  Eigen::Vector3f vel{0, 0, 0};
  float time{0.0};
};

struct Trajectory {
  std::vector<State> trajectory;

  void calculateTimes(const float step_size) {
    for (int i = 0; i < trajectory.size(); i++) {
      trajectory[i].time = i * step_size;
    }
  }

  bool calculateTimes() {
    if (trajectory.empty()) {
      return false;
    }
    trajectory[0].time = 0.0;
    for (int i = 0; i < trajectory.size() - 1; i++) {
      trajectory[i].time =
          (trajectory[i + 1].position - trajectory[i].position).norm() /
          trajectory[i].vel.norm();
    }
  }
};

struct Follower {
  const double look_ahead{1.0};
  int pose_on_path{0};
  const float rate{0.01};  // hz
  const float step_size{0.1};

  std::chrono::time_point<std::chrono::high_resolution_clock> time_last_traj;

  float calculateYawDiff(const float _desired_yaw, const float _current_yaw);
  int cal_pose_on_path(const std::vector<State> &trajectory,
                       const Eigen::Vector3f &current_pose);
  int cal_pose_look_ahead(const std::vector<State> &trajectory);
  Eigen::Vector3f calculate_vel(const Eigen::Vector3f &target_pose,
                                const Eigen::Vector3f &current_pose,
                                const float target_time);
};

int main(int _argc, char **_argv) {
  ros::init(_argc, _argv, "trajectory_follower_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  Trajectory last_traj_received;
  State uav_state;
  Follower follower;
  //// ROS publishers and subscribers //////////
  ros::Publisher tracking_pub =
      nh.advertise<nav_msgs::Path>("follower/trajectory_to_follow", 1);

  // trajectory callback
  auto trajectoryCallback =
      [&last_traj_received, &follower,
       tracking_pub](const trajectory_msgs::JointTrajectory::ConstPtr &msg) {
        last_traj_received.trajectory.resize(msg->points.size());
        follower.time_last_traj = std::chrono::high_resolution_clock::now();
        follower.pose_on_path = 0;

        geometry_msgs::PoseStamped pose_stamped;
        nav_msgs::Path path_to_publish;
        path_to_publish.header.frame_id = "map";

        for (int i = 0; i < msg->points.size(); i++) {
          pose_stamped.pose.position.x = msg->points[i].positions[0];
          pose_stamped.pose.position.y = msg->points[i].positions[1];
          pose_stamped.pose.position.z = msg->points[i].positions[2];

          last_traj_received.trajectory[i].position[0] =
              msg->points[i].positions[0];
          last_traj_received.trajectory[i].position[1] =
              msg->points[i].positions[1];
          last_traj_received.trajectory[i].position[2] =
              msg->points[i].positions[2];

          last_traj_received.trajectory[i].orientation.x() =
              msg->points[i].positions[3];
          last_traj_received.trajectory[i].orientation.y() =
              msg->points[i].positions[4];
          last_traj_received.trajectory[i].orientation.z() =
              msg->points[i].positions[5];
          last_traj_received.trajectory[i].orientation.w() =
              msg->points[i].positions[6];

          last_traj_received.trajectory[i].vel[0] =
              msg->points[i].velocities[0];
          last_traj_received.trajectory[i].vel[1] =
              msg->points[i].velocities[1];
          last_traj_received.trajectory[i].vel[2] =
              msg->points[i].velocities[2];

          path_to_publish.poses.push_back(pose_stamped);
        }
        last_traj_received.calculateTimes(follower.step_size);
        tracking_pub.publish(path_to_publish);
      };
  auto sub_traj = pnh.subscribe<trajectory_msgs::JointTrajectory>(
      "trajectory_to_follow", 1, trajectoryCallback);
  // ual state subscriber
  auto ualPoseCallback =
      [&uav_state](const geometry_msgs::PoseStamped::ConstPtr &msg) {
        uav_state.position = Eigen::Vector3f(
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        uav_state.orientation.x() = msg->pose.orientation.x;
        uav_state.orientation.y() = msg->pose.orientation.y;
        uav_state.orientation.z() = msg->pose.orientation.z;
        uav_state.orientation.w() = msg->pose.orientation.w;
      };
  auto pose_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("ual/pose", 1, ualPoseCallback);

  ros::Publisher velocity_ual_pub =
      nh.advertise<geometry_msgs::TwistStamped>("ual/set_velocity", 1);
  /////////////////////////
  // PID controller for yaw
  grvc::utils::PidController yaw_pid("yaw", YAW_PID_P, YAW_PID_I, YAW_PID_D);

  /////// main loop   //////////////
  while (ros::ok) {
    // wait for receiving trajectories
    while (!last_traj_received.trajectory.empty()) {
      follower.pose_on_path = follower.cal_pose_on_path(
          last_traj_received.trajectory, uav_state.position);
      int target_pose_idx =
          follower.cal_pose_look_ahead(last_traj_received.trajectory);
      // if the point to go is out of the trajectory, the trajectory will be
      // finished and cleared
      if (target_pose_idx == last_traj_received.trajectory.size()) {
        last_traj_received.trajectory.clear();
        follower.pose_on_path = 0;
        break;
      }
      Eigen::Vector3f target_pose =
          last_traj_received.trajectory[target_pose_idx].position;

      Eigen::Vector3f velocity_to_command = follower.calculate_vel(
          target_pose, uav_state.position,
          last_traj_received.trajectory[target_pose_idx].time);

      // controlling yaw
      tf2::Quaternion desired_q(
          last_traj_received.trajectory[target_pose_idx].orientation.x(),
          last_traj_received.trajectory[target_pose_idx].orientation.y(),
          last_traj_received.trajectory[target_pose_idx].orientation.z(),
          last_traj_received.trajectory[target_pose_idx].orientation.w());
      tf2::Quaternion current_q(
          uav_state.orientation.x(), uav_state.orientation.y(),
          uav_state.orientation.z(), uav_state.orientation.w());

      float current_yaw = tf2::getYaw(current_q);
      float desired_yaw = tf2::getYaw(desired_q);
      float yaw_diff = follower.calculateYawDiff(desired_yaw, current_yaw);
      float sampling_period = follower.rate;

      // publish topic to ual
      geometry_msgs::TwistStamped vel;
      vel.header.frame_id = "map";
      vel.twist.linear.x = velocity_to_command.x();
      vel.twist.linear.y = velocity_to_command.y();
      vel.twist.linear.z = velocity_to_command.z();
      vel.twist.angular.z = yaw_pid.control_signal(yaw_diff, sampling_period);

      velocity_ual_pub.publish(vel);
      ros::spinOnce();
      ros::Duration(follower.rate).sleep();
    }
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
  return 0;
}

float Follower::calculateYawDiff(const float _desired_yaw,
                                 const float _current_yaw) {
  float yaw_diff = _desired_yaw - _current_yaw;
  while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
  while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
  return yaw_diff;
}

int Follower::cal_pose_on_path(const std::vector<State> &trajectory,
                               const Eigen::Vector3f &current_pose) {
  double min_distance = INFINITY;
  int pose_on_path_id = 0;
  for (int i = pose_on_path; i < trajectory.size(); i++) {
    if ((current_pose - trajectory[i].position).norm() < min_distance) {
      min_distance = (current_pose - trajectory[i].position).norm();
      pose_on_path_id = i;
    }
  }
  return pose_on_path_id;
}

int Follower::cal_pose_look_ahead(const std::vector<State> &trajectory) {
  for (int i = pose_on_path; i < trajectory.size(); i++) {
    if ((trajectory[i].position - trajectory[pose_on_path].position).norm() >
        look_ahead)
      return i;
  }
  return trajectory.size();
}

Eigen::Vector3f Follower::calculate_vel(const Eigen::Vector3f &target_pose,
                                        const Eigen::Vector3f &current_pose,
                                        const float target_time) {
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float> current_time = end - time_last_traj;

  Eigen::Vector3f vel_unitary = (target_pose - current_pose).normalized();
  double vel_module = (target_pose - current_pose).norm() /
                      (target_time - current_time.count());
  return vel_unitary * vel_module;
}