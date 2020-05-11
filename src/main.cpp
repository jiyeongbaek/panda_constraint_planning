

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <memory>
#include <cmath>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>


#include <ompl/util/Time.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/PathGeometric.h>

#include <constraint_planner/constraints/ConstrainedPlanningCommon.h>
#include <constraint_planner/kinematics/KinematicChain.h>
#include <ctime>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ExecuteTrajectoryGoal.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/planning_context_manager.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <boost/algorithm/string.hpp>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
unsigned int links = 14;

bool plannedPath(Eigen::VectorXd start, Eigen::VectorXd goal, std::string file_name)
{
    auto ss = std::make_shared<KinematicChainSpace>(links);
    std::vector<enum PLANNER_TYPE> planners = {RRT, PRM, newRRT, newPRM, RRTConnect}; //RRTConnect

    std::cout << "init state   : " << start.transpose() << std::endl;
    std::cout << "target state : " << goal.transpose() << std::endl;

    auto constraint = std::make_shared<KinematicChainConstraint>(links, start);

    ConstrainedProblem cp(ss, constraint); // define a simple problem to solve this constrained space
    cp.setConstrainedOptions();
    cp.setStartAndGoalStates(start, goal);
    
    // cp.ss->setStateValidityChecker(std::make_shared<ConstrainedKinematicChainValidityChecker>(cp.csi));
    cp.ss->setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(cp.csi));

    cp.setPlanner(planners[3]);
    ob::PlannerStatus stat = cp.solveOnce(true, file_name);
    return stat;    // return (planningBench(cp, planners));    
}


void execute_path(std::string path_name, moveit::planning_interface::MoveGroupInterface& move_group, double total_time = 1.5)
{
    moveit_msgs::RobotTrajectory robot_trajectory;
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = move_group.getJointNames();
    ifstream path_file(path_name);
    while (path_file)
    {
        trajectory_msgs::JointTrajectoryPoint traj_point;
        bool eof = false;
        for (int j = 0; j < 14; j++)
        {
            double data;
            if (!(path_file >> data))
            {
                // cout << "wrong file: " << j << endl;
                eof = true;
                break;
            }
            traj_point.positions.push_back(data);
        }
        if (eof)
            break;
        // traj_point.time_from_start = ros::Duration();
        joint_trajectory.points.push_back(traj_point);
    }

    int n_traj = joint_trajectory.points.size();
    for (int i = 0; i < n_traj; i++)
        joint_trajectory.points[i].time_from_start = ros::Duration(total_time / n_traj * i);


    robot_trajectory.joint_trajectory = joint_trajectory;
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = robot_trajectory;
    plan.planning_time_ = total_time;
    move_group.execute(plan);;
}

int main(int argc, char **argv)
{
    std::string name_ = "dual_arm_constraint_planning";
    ros::init(argc, argv, name_);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    ros::WallDuration(1.0).sleep();    
    
    /* PLANNING AND RETURN PLANNED PATH */
    Eigen::VectorXd start(links), goal(links);
    Matrix<double, 2, 14> goal_lists;

    goal_lists <<-1.6671076987319884, -1.4984785565179355, 1.5404192524883924, -2.388776541995507, -2.897203095390305, 3.280959665933266, -1.0941728565641882, -0.10245023002256344, 0.26594811616937525, 0.41290315245014864, -1.390080227072562, 0.06799447874492352, 1.5907921066040371, 2.0916256998720577,
                -0.42784663202016965, 0.46688110660443566, -0.050990016440297804, -0.8969788666395989, 0.010386418882852508, 1.3656065313896382, 1.8328089096024114, -0.12732719919206942, 0.6544413855989148, -0.04499967102510616, -1.7690418525615275, 1.4836964245964588, 1.3373796725734979, 1.4676640046123115;

    for (int i = 0; i < goal_lists.rows() - 1; i++)
    {   
        start = goal_lists.row(i);
        goal = goal_lists.row(i + 1);
        if (!plannedPath(start, goal, to_string(i)))
            break;
    }

    moveit::planning_interface::MoveGroupInterface move_group("panda_arms");    
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 5, true);
    while (display_publisher.getNumSubscribers() == 0 && ros::ok())
        ros::spinOnce();
    for (int i = 0; i < goal_lists.rows() - 1; i++)
        execute_path("/home/jiyeong/catkin_ws/" + to_string(i) + "_path.txt", move_group);        
}
