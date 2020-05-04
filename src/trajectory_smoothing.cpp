#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>
#include <cstdio>
#include <Eigen/Core>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <trajectory_msgs/JointTrajectory.h>
using namespace std;
using namespace Eigen;

moveit::core::RobotModelConstPtr model = moveit::core::loadTestingRobotModel("dual_panda");
robot_trajectory::RobotTrajectory left_trajectory(model, "panda_left");
robot_trajectory::RobotTrajectory right_trajectory(model, "panda_right");
const robot_model::JointModelGroup* left_group = left_trajectory.getGroup();
const robot_model::JointModelGroup* right_group = right_trajectory.getGroup();
const std::vector<int>& left_idx = left_group->getVariableIndexList();
const std::vector<int>& right_idx = right_group->getVariableIndexList();
    
moveit::core::RobotState left_state(left_trajectory.getRobotModel());
moveit::core::RobotState right_state(right_trajectory.getRobotModel());

const std::string DUAL_JOINT_NAME[14] =
{
    "panda_left_joint1", "panda_left_joint2", "panda_left_joint3",
    "panda_left_joint4", "panda_left_joint5", "panda_left_joint6",
    "panda_left_joint7",
    "panda_right_joint1", "panda_right_joint2", "panda_right_joint3",
    "panda_right_joint4", "panda_right_joint5", "panda_right_joint6",
    "panda_right_joint7"
};

int initTrajectory(robot_trajectory::RobotTrajectory& left_trajectory, robot_trajectory::RobotTrajectory& right_trajectory)
{
    if (!left_group || !right_group)
    {
        ROS_ERROR_NAMED("trajectory_processing", "Need to set the group");
        return -1;
    }
    left_trajectory.clear();   
    right_trajectory.clear();

    ifstream path_file("/home/jiyeong/catkin_ws/src/3_dual_constraint_planning/test_vrep/path.txt");
    while (path_file)
    {
        bool eof = false;
        for (int j = 0; j < 14; j++)
        {
            double data;
            if (!(path_file >> data))
            {
                eof = true;
                break;
            }
            if (j < 7)
              left_state.setVariablePosition(left_idx[j], data);
            else
              right_state.setVariablePosition(right_idx[j-7], data);
        }
        left_trajectory.addSuffixWayPoint(left_state, 0.0);
        right_trajectory.addSuffixWayPoint(right_state, 0.0);
      
        if (eof)
            break;
    }
    return 0;
}

void printTrajectory(robot_trajectory::RobotTrajectory& trajectory)
{
  const robot_model::JointModelGroup* group = trajectory.getGroup();
  const std::vector<int>& idx = group->getVariableIndexList();
  unsigned int count = trajectory.getWayPointCount();

  std::cout << "trajectory length is " << trajectory.getWayPointDurationFromStart(count - 1) << " seconds."
            << std::endl;
  std::cout << "  Trajectory Points" << std::endl;
  for (unsigned i = 0; i < count; i++)
  {
    robot_state::RobotStatePtr point = trajectory.getWayPointPtr(i);
    printf("  waypoint %2d time %6.2f pos %6.2f vel %6.2f acc %6.2f ", i, trajectory.getWayPointDurationFromStart(i),
           point->getVariablePosition(idx[0]), point->getVariableVelocity(idx[0]),
           point->getVariableAcceleration(idx[0]));
    if (i > 0)
    {
      robot_state::RobotStatePtr prev = trajectory.getWayPointPtr(i - 1);
      printf("jrk %6.2f",
             (point->getVariableAcceleration(idx[0]) - prev->getVariableAcceleration(idx[0])) /
                 (trajectory.getWayPointDurationFromStart(i) - trajectory.getWayPointDurationFromStart(i - 1)));
    }
    printf("\n");
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "constraint_planner");
    ros::NodeHandle nh;

    trajectory_processing::IterativeSplineParameterization time_parameterization(true);
    initTrajectory(left_trajectory, right_trajectory);
    time_parameterization.computeTimeStamps(left_trajectory, 0.9, 0.3);
    time_parameterization.computeTimeStamps(right_trajectory, 0.9, 0.3);
    ROS_INFO("Compute trajectory parameterization");
    unsigned int count = left_trajectory.getWayPointCount();
  
    trajectory_msgs::JointTrajectory dual_traj;
    dual_traj.joint_names.resize(14);
    for (size_t i = 0; i < 14; i++)
    {
        dual_traj.joint_names[i] = DUAL_JOINT_NAME[i];
    }
    for (unsigned int i = 0; i < count; i++)
    {
        robot_state::RobotStatePtr Lpoint = left_trajectory.getWayPointPtr(i);    
        robot_state::RobotStatePtr Rpoint = right_trajectory.getWayPointPtr(i);    

        trajectory_msgs::JointTrajectoryPoint traj_points;
        // traj_points.positions.resize(14);
        // traj_points.velocities.resize(14);
        // traj_points.accelerations.resize(14);
        for (int j = 0; j < 7; j++)
        {
            traj_points.positions.push_back(Lpoint->getVariablePosition(left_idx[j]));
            traj_points.velocities.push_back(Lpoint->getVariableVelocity(left_idx[j]));
            traj_points.accelerations.push_back(Lpoint->getVariableAcceleration(left_idx[j]));
        }
        for (int j = 0; j < 7; j++)
        {
            traj_points.positions.push_back(Rpoint->getVariablePosition(right_idx[j]));
            traj_points.velocities.push_back(Rpoint->getVariableVelocity(right_idx[j]));
            traj_points.accelerations.push_back(Rpoint->getVariableAcceleration(right_idx[j]));
        }
        traj_points.time_from_start = ros::Duration(left_trajectory.getWayPointDurationFromStart(i));
        dual_traj.points.push_back(traj_points);
    }

    ros::Publisher dual_traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/panda_dual/dual_trajectory", 1);
    
    while (ros::ok())
    {
        dual_traj_pub.publish(dual_traj);   
    }
    
    return 0; 
}
