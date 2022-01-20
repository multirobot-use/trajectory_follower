#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <handy_tools/pid_controller.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Eigen>
#include <chrono>

const float YAW_PID_P{0.2};
const float YAW_PID_I{0.02};
const float YAW_PID_D{0.1};

template <typename T>
  inline bool safeGetParam(ros::NodeHandle &_nh, std::string const &_param_name,
                           T &_param_value) {
    if (!_nh.getParam(_param_name, _param_value)) {
      ROS_ERROR("Failed to find parameter: %s",
                _nh.resolveName(_param_name, true).c_str());
      exit(1);
    }
    return true;
  }

struct State {
  Eigen::Vector3f position{0, 0, 0};
  Eigen::Quaternionf orientation{0, 0, 0, 0};
  Eigen::Vector3f vel{0, 0, 0};
  float time{0.0};
  float real_time{0.0};
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
  double look_ahead{1.0};
  int pose_on_path{0};
  const float rate{0.01};  // sec
  float step_size{0.2};
  Trajectory *previous_trajectory = (Trajectory*)malloc(sizeof(Trajectory) * 2);

  //std::chrono::time_point<std::chrono::high_resolution_clock> time_last_traj;
  ros::Time time_last_traj;

  float calculateYawDiff(const float _desired_yaw, const float _current_yaw);
  int cal_pose_on_path(const std::vector<State> &trajectory,
                       const Eigen::Vector3f &current_pose);
  int cal_pose_look_ahead(const std::vector<State> &trajectory);
  Eigen::Vector3f calculate_vel(const Eigen::Vector3f &target_pose,
                                const Eigen::Vector3f &current_pose,
                                const float target_time);
  void calculate_look_ahead(const float &vel);
};

int main(int _argc, char **_argv) {
  ros::init(_argc, _argv, "trajectory_follower_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  Trajectory last_traj_received;
  State uav_state;
  Follower follower;
  safeGetParam(pnh,"look_ahead", follower.look_ahead);
  safeGetParam(pnh,"step_size", follower.step_size);
  //// ROS publishers and subscribers //////////
  ros::Publisher tracking_pub =
      nh.advertise<nav_msgs::Path>("follower/trajectory_to_follow", 1);

  // trajectory callback
  auto trajectoryCallback =
      [&last_traj_received, &follower,
       tracking_pub](const trajectory_msgs::JointTrajectory::ConstPtr &msg) {
        // last_traj_received.trajectory.resize(msg->points.size());
        follower.time_last_traj = ros::Time::now();
        follower.pose_on_path = 0;
        geometry_msgs::PoseStamped pose_stamped;
        nav_msgs::Path path_to_publish;
        path_to_publish.header.frame_id = "map";
        float time_point;
        int start_i = 0;
        int i = 0;
        int p = 0;

        // SECURITY MEASURE: Check times of the new trajectory in order to not execute
        // points of the past
        for (int it = 0; it < msg->points.size(); it++){
          if (follower.time_last_traj.toSec() < msg->points[it].time_from_start.toSec()){
            start_i = it;
            // std::cout << "Start i: " << start_i << " with time: " << msg->points[i].time_from_start.toSec() << std::endl;
            break;
          }
        }

        // std::cout << "Time last traj: " << follower.time_last_traj << std::endl;
        // std::cout << "Time of the first point of the trajectory: " << msg->points[0].time_from_start << std::endl;

        // SECURITY MEASURE: New trajectory received earlier than expected and the
        // points to execute are planned for the future

        // Adding to path_to_publish the points of the previous trajectory
        // (from current time to the first point of the new trajectory received)

        // std::cout << "Size: " << follower.previous_trajectory->trajectory.size() << std::endl;
        // If previous trajectory is not empty
        bool activate_traj_fusion = false;  // When using MAVROS, this upgrade does not work --> put false
        if (activate_traj_fusion){

          if (!follower.previous_trajectory->trajectory.empty() && 
              (abs(follower.previous_trajectory->trajectory[0].position.norm() -
              follower.previous_trajectory->trajectory[follower.previous_trajectory->trajectory.size() - 1].position.norm()) < 0.1)){
            // If the trajectory received is not from the past
            if (start_i == 0){
              // std::cout << "start_i = 0!" << std::endl;
              int i_p = follower.previous_trajectory->trajectory.size() - 1;

              std::cout << "Current time: " << follower.time_last_traj.toSec() << std::endl;
              // std::cout << "First time of previous trajectory: " << follower.previous_trajectory->trajectory[0].real_time << std::endl;
              // std::cout << "Last time of previous trajectory: " << follower.previous_trajectory->trajectory[i_p].real_time << std::endl;

              // Get the closer point in time (staring to the future) of the previous trajectory
              // If the time of that point is greater or equal to the first point of the new trajectory, break
              while ((follower.previous_trajectory->trajectory[i_p].real_time > (follower.time_last_traj.toSec() + follower.step_size)) && (i_p >= 0)){
                i_p = i_p - 1;
              }

              // std::cout << "i_p: " << i_p << std::endl;
              std::cout << "Time for that i_p: " << follower.previous_trajectory->trajectory[i_p].real_time << std::endl;

              std::cout << "Time of the first point of the traj received: " << msg->points[0].time_from_start.toSec() << std::endl;

              if (i_p > 0) {
                for (int k = i_p;
                    (k < follower.previous_trajectory->trajectory.size()) &&
                    ((follower.previous_trajectory->trajectory[k].real_time + follower.step_size) < msg->points[0].time_from_start.toSec());
                    k++, p++){
                  pose_stamped.header.stamp.sec  = int(follower.previous_trajectory->trajectory[k].real_time);
                  pose_stamped.header.stamp.nsec = int( (follower.previous_trajectory->trajectory[k].real_time -
                                                        pose_stamped.header.stamp.sec)*1000000000 );

                  pose_stamped.pose.position.x = follower.previous_trajectory->trajectory[k].position(0);
                  pose_stamped.pose.position.y = follower.previous_trajectory->trajectory[k].position(1);
                  pose_stamped.pose.position.z = follower.previous_trajectory->trajectory[k].position(2);

                  last_traj_received.trajectory[p].position[0] =
                      follower.previous_trajectory->trajectory[k].position(0);
                  last_traj_received.trajectory[p].position[1] =
                      follower.previous_trajectory->trajectory[k].position(1);
                  last_traj_received.trajectory[p].position[2] =
                      follower.previous_trajectory->trajectory[k].position(2);

                  last_traj_received.trajectory[p].orientation.x() =
                      follower.previous_trajectory->trajectory[k].orientation.x();
                  last_traj_received.trajectory[p].orientation.y() =
                      follower.previous_trajectory->trajectory[k].orientation.y();
                  last_traj_received.trajectory[p].orientation.z() =
                      follower.previous_trajectory->trajectory[k].orientation.z();
                  last_traj_received.trajectory[p].orientation.w() =
                      follower.previous_trajectory->trajectory[k].orientation.w();

                  last_traj_received.trajectory[p].vel[0] =
                      follower.previous_trajectory->trajectory[k].vel(0);
                  last_traj_received.trajectory[p].vel[1] =
                      follower.previous_trajectory->trajectory[k].vel(1);
                  last_traj_received.trajectory[p].vel[2] =
                      follower.previous_trajectory->trajectory[k].vel(2);
                  
                  last_traj_received.trajectory[p].real_time =
                      follower.previous_trajectory->trajectory[k].real_time;
                  
                  std::cout << "k: " << k << "   time: " << follower.previous_trajectory->trajectory[k].real_time << std::endl;

                  path_to_publish.poses.push_back(pose_stamped);
                }
                std::cout << "Added " << p << " points from previous trajectory" << std::endl;
              }
            }
          }

        }

        last_traj_received.trajectory.resize(msg->points.size() + p);

        // Adding to path_to_publish the points of the trajectory received
        for (int j = start_i; j < msg->points.size(); i++, j++) {

          // std::cout << "i: " << i << "   j: " << j << std::endl;

          time_point = msg->points[j].time_from_start.toSec();
          pose_stamped.header.stamp.sec  = int(time_point);
          pose_stamped.header.stamp.nsec = int( (time_point - int(time_point))*1000000000 );

          // Check
          // std::cout << "  Total: " << follower.time_last_traj.toSec() << std::endl << std::endl;
          pose_stamped.pose.position.x = msg->points[j].positions[0];
          pose_stamped.pose.position.y = msg->points[j].positions[1];
          pose_stamped.pose.position.z = msg->points[j].positions[2];

          last_traj_received.trajectory[j+p].position[0] =
              msg->points[j].positions[0];
          last_traj_received.trajectory[j+p].position[1] =
              msg->points[j].positions[1];
          last_traj_received.trajectory[j+p].position[2] =
              msg->points[j].positions[2];

          last_traj_received.trajectory[j+p].orientation.x() =
              msg->points[j].positions[3];
          last_traj_received.trajectory[j+p].orientation.y() =
              msg->points[j].positions[4];
          last_traj_received.trajectory[j+p].orientation.z() =
              msg->points[j].positions[5];
          last_traj_received.trajectory[j+p].orientation.w() =
              msg->points[j].positions[6];

          last_traj_received.trajectory[j+p].vel[0] =
              msg->points[j].velocities[0];
          last_traj_received.trajectory[j+p].vel[1] =
              msg->points[j].velocities[1];
          last_traj_received.trajectory[j+p].vel[2] =
              msg->points[j].velocities[2];

          last_traj_received.trajectory[j+p].real_time =
              msg->points[j].time_from_start.toSec();

          path_to_publish.poses.push_back(pose_stamped);
        }

        last_traj_received.calculateTimes(follower.step_size);

        // Refresh the previous trajectory and publish
        follower.previous_trajectory->trajectory.clear();
        std::memcpy(follower.previous_trajectory, &last_traj_received, sizeof(last_traj_received));
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

  // Declare and initialize velocity_to_command
  Eigen::Vector3f velocity_to_command(0.2, 0.2, 0);
  while (ros::ok) {
    
    // wait for receiving trajectories
    while (!last_traj_received.trajectory.empty()) {
      follower.pose_on_path = follower.cal_pose_on_path(
          last_traj_received.trajectory, uav_state.position);
      
      if (velocity_to_command.norm() > 0.1){
        follower.calculate_look_ahead(velocity_to_command.norm());
      }
      else{
        safeGetParam(pnh,"look_ahead", follower.look_ahead);
      }
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

      // Use ROS::Time::now() instead of std::chrono
      velocity_to_command = follower.calculate_vel(
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
    ros::Duration(0.1).sleep();
    ros::spinOnce();
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
  ros::Time end = ros::Time::now();
  ros::Duration current_time_ros = end - time_last_traj;
  float current_time = current_time_ros.toSec();

  Eigen::Vector3f vel_unitary = (target_pose - current_pose).normalized();
  double vel_module = (target_pose - current_pose).norm() /

                      (target_time - current_time);

  // std::cout << "Vel: " << vel_module << std::endl;
  // std::cout << "Diff time: " << (target_time - current_time) << std::endl;
  if(vel_module<0) {
	std::cerr<<"vel negative\n";
	vel_module = 0.0;	}
  return vel_unitary * vel_module;
}

void Follower::calculate_look_ahead(const float &vel){
  look_ahead = vel*3*step_size;

  // std::cout << "Look ahead: " << look_ahead << std::endl;
}
