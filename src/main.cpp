
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <memory>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

// #include <ompl/base/Constraint.h>
#include <constraint_planner/Constraint.h>

#include <ompl/util/Time.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>

#include <constraint_planner/ConstrainedPlanningCommon.h>
#include <constraint_planner/KinematicChain.h>
#include <constraint_planner/panda_model_updater.h>

#include <constraint_planner/jacobian.h>

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


#include <constraint_planner/ConstraintFunction.h>

using namespace std;

class ConstrainedKinematicChainValidityChecker : public KinematicChainValidityChecker
{
public:
    ConstrainedKinematicChainValidityChecker(const ob::ConstrainedSpaceInformationPtr &si)
        : KinematicChainValidityChecker(si)
    {
    }
    bool isValid(const ob::State *state) const override
    {
        auto &&space = si_->getStateSpace()->as<ob::ConstrainedStateSpace>()->getSpace()->as<KinematicChainSpace>();
        auto &&s = state->as<ob::ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>();
        // if (isValidImpl(space, s))
        //     OMPL_INFORM("FOUND VALID STATE");
        return isValidImpl(space, s);
    }
};


// bool planning(ConstrainedProblem &cp, ompl::geometric::PathGeometric &path, std::string file_name)
bool planning(ConstrainedProblem &cp, std::string file_name)
{
    ob::PlannerStatus stat = cp.solveOnce(true, file_name);
    return stat;
}

bool planningBench(ConstrainedProblem &cp, std::vector<enum PLANNER_TYPE> &planners)
{
    cp.setupBenchmark(planners, "kinematic");
    cp.bench->addExperimentParameter("links", "INTEGER", std::to_string(cp.constraint->getAmbientDimension()));

    cp.runBenchmark();
    return 0;
}

bool plannedPath(Eigen::VectorXd start, Eigen::VectorXd goal, std::string file_name)
{
    auto ss = std::make_shared<KinematicChainSpace>(links);
    enum SPACE_TYPE space = PJ; //"PJ", "AT", "TB"
    std::vector<enum PLANNER_TYPE> planners = {RRT, PRM, No_RRT, RRTConnect, KPIECE}; //RRTConnect

    std::cout << "init state   : " << start.transpose() << std::endl;
    std::cout << "target state : " << goal.transpose() << std::endl;

    auto constraint = std::make_shared<KinematicChainConstraint2>(links, start);

    ConstrainedProblem cp(space, ss, constraint); // define a simple problem to solve this constrained space
    cp.setConstrainedOptions();
    cp.setAtlasOptions();
    cp.setStartAndGoalStates(start, goal);
    
    
    cp.ss->setStateValidityChecker(std::make_shared<ConstrainedKinematicChainValidityChecker>(cp.csi));
    

    cp.setPlanner(planners[0]);
    return (planning(cp, file_name));
    // return (planningBench(cp, planners));
     
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
    {
        joint_trajectory.points[i].time_from_start = ros::Duration(total_time / n_traj * i);
    }

    robot_trajectory.joint_trajectory = joint_trajectory;
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = robot_trajectory;
    plan.planning_time_ = total_time;
    move_group.execute(plan);
    cout << "EXECUTE" << endl;
}

int main(int argc, char **argv)
{
    std::string name_ = "dual_arm_constraint_planning";
    ros::init(argc, argv, name_);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    ros::WallDuration(1.0).sleep();
    const std::string PLANNING_GROUP = "panda_arms";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    robot_state::RobotStatePtr robot_state = move_group.getCurrentState(); // plan_manager->robot_state_;
    robot_model::RobotModelConstPtr robot_model = move_group.getRobotModel();

    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 5, true);
    while (display_publisher.getNumSubscribers() == 0 && ros::ok())
    {
        ros::spinOnce();
    }
    
    /* PLANNING AND RETURN PLANNED PATH */
    Eigen::VectorXd start(links), goal(links);

    start.setZero();
    goal.setZero();
    Matrix<double, 10, 14> goal_lists;

    goal_lists << 1.14669, -1.38017, -1.45742, -3.02049, -2.82783, 1.09462, 2.5943, 0.136339, 0.0771, 0.132456, -1.61833, 0.163069, 1.64452, 2.04722, 
 // distance : 1.79682
// 1.14621, -1.59073, -1.15525, -2.94124, -2.58406, 1.11348, 2.33476, 0.0441954, 0.128898, 0.130295, -1.41736, 0.252909, 1.44964, 1.99681, 
 // distance : 0.26136
1.18968, -1.59237, -1.11863, -2.91066, -2.52977, 1.08409, 2.33773, 0.0334045, 0.136417, 0.124587, -1.41625, 0.281727, 1.45013, 1.98884, 
 // distance : 1.16696
1.39528, -1.60228, -0.983353, -2.73914, -2.33624, 0.94933, 2.34156, -0.0192996, 0.173522, 0.0987747, -1.42393, 0.425628, 1.45452, 1.94789, 
 // distance : 1.00669
1.57046, -1.60047, -0.893183, -2.54753, -2.22043, 0.84452, 2.34377, -0.0695203, 0.210368, 0.0796314, -1.45146, 0.565459, 1.4618, 1.90367, 
 // distance : 0.908523
1.72441, -1.59743, -0.824236, -2.34609, -2.16067, 0.759584, 2.33849, -0.12392, 0.248795, 0.0769186, -1.49489, 0.699122, 1.47191, 1.85524, 
 // distance : 0.834975
1.84508, -1.57475, -0.771146, -2.13532, -2.11418, 0.710033, 2.33904, -0.165651, 0.292093, 0.0720454, -1.54657, 0.828797, 1.4782, 1.80159, 
 // distance : 0.778868
1.93982, -1.53853, -0.733273, -1.91646, -2.07348, 0.688523, 2.33985, -0.201064, 0.342286, 0.0728541, -1.60237, 0.953716, 1.48057, 1.74296, 
 // distance : 0.754351
2.02247, -1.49706, -0.705544, -1.68878, -2.04497, 0.681718, 2.33247, -0.224104, 0.401821, 0.0720446, -1.65717, 1.07746, 1.47468, 1.67865, 
 // distance : 0.762463
// 2.10085, -1.45043, -0.684826, -1.44756, -2.03038, 0.686014, 2.31258, -0.236162, 0.472911, 0.0717868, -1.70595, 1.19792, 1.45845, 1.61077, 
 // distance : 0.82212
2.18154, -1.39057, -0.673757, -1.17815, -2.02867, 0.703286, 2.27391, -0.239152, 0.55708, 0.0749066, -1.745, 1.31533, 1.43125, 1.54129, 
 // distance : 1.07191
2.28836, -1.292, -0.684402, -0.825018, -2.05174, 0.744845, 2.19023, -0.238986, 0.655853, 0.0894607, -1.77037, 1.42673, 1.39508, 1.47327;

        // execute_path("/home/jiyeong/catkin_ws/hihihi.txt", move_group);

    ompl::time::point start_time = ompl::time::now();
    for (int i = 0; i < goal_lists.rows() - 1; i++)
    {   
        start = goal_lists.row(i);
        goal = goal_lists.row(i + 1);
        plannedPath(start, goal, to_string(i));
    }
    double planTime_ = ompl::time::seconds(ompl::time::now() - start_time);
    OMPL_INFORM("Solution found in %f seconds", planTime_);

    for (int i = 0; i < goal_lists.rows() - 1; i++)
        execute_path("/home/jiyeong/catkin_ws/" + to_string(i) + "_path.txt", move_group);
        
}
