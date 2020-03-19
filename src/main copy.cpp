
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <memory>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/base/Constraint.h>
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

using namespace std;
unsigned int links = 14;
Eigen::Affine3d left_base = Eigen::Affine3d::Identity();
Eigen::Affine3d right_base = Eigen::Affine3d::Identity();

int iter = 0;
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
        if (isValidImpl(space, s))
            OMPL_INFORM("FOUND VALID STATE");
        return isValidImpl(space, s);
    }
};

class KinematicChainConstraint : public ob::Constraint
{
public:
    /* ob::Constraint(a, b) : a is dimension of the ambient space, b is constraint ouputs*/
    KinematicChainConstraint(unsigned int links, Eigen::VectorXd start) : ob::Constraint(links, 1)
    {
        for (int i = 0; i < 7; i++)
        {
            left_qinit[i] = start[i];
            right_qinit[i] = start[i + 7];
        }
        left_base.translation() = Eigen::Vector3d(0.0, 0.2, 0.0);
        right_base.translation() = Eigen::Vector3d(0.0, -0.2, 0.0);

        left_arm = std::make_shared<FrankaModelUpdater>(left_q);
        right_arm = std::make_shared<FrankaModelUpdater>(right_q);

        left_init = left_base * left_arm->getTransform(left_qinit);
        right_init = right_base * right_arm->getTransform(right_qinit);
        init = left_init.inverse() * right_init;
        init_tr = init.linear().transpose();
    }

    /*actual constraint function, state "x" from the ambient space */
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {

        Eigen::VectorXd &&temp = x.segment(0, 7);
        Eigen::VectorXd &&temp2 = x.segment(7, 7);

        Eigen::Affine3d lt = left_base * left_arm->getTransform(temp);
        Eigen::Affine3d rt = right_base * right_arm->getTransform(temp2);

        Eigen::Affine3d result = lt.inverse() * rt;
        
        double r;
        Eigen::Quaterniond cur_q(result.linear());
        Eigen::Quaterniond ori_q(init.linear());
        r = cur_q.angularDistance(ori_q);
        
        double d;
        d = (result.translation() - init.translation()).norm();
        
        // double d_rot;
        // d_rot = (  init_tr * result.linear() ).log().norm();
        
        out[0] = d + r;
    }

    /* this is very computationally intensive, and providing an analytic derivative is preferred. We provide a simple scrip */
    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        double h = 1e-3;
        double d, d_rot;
        double r;
        Eigen::Quaterniond ori_q(init.linear());
        for (int i = 0; i < 14; i++)
        {
            Eigen::VectorXd &&q_temp = x;
            q_temp(i) = x(i) + h;
            Eigen::Affine3d lt_1 = left_base * left_arm->getTransform(q_temp.segment(0, 7));
            Eigen::Affine3d rt_1 = right_base * right_arm->getTransform(q_temp.segment(7, 7));
            Eigen::Affine3d result_1 = lt_1.inverse() * rt_1;
            
            // d_rot = ( init_tr * result_1.linear() ).log().norm();

            Eigen::Quaterniond cur1_q(result_1.linear());
            r = cur1_q.angularDistance(ori_q);
            d = (result_1.translation() - init.translation()).norm();
            double g1 = d + r;

            q_temp(i) = x(i) - h;
            Eigen::Affine3d lt_2 = left_base * left_arm->getTransform(q_temp.segment(0, 7));
            Eigen::Affine3d rt_2 = right_base * right_arm->getTransform(q_temp.segment(7, 7));
            Eigen::Affine3d result_2 = lt_2.inverse() * rt_2;
            Eigen::Quaterniond cur2_q(result_2.linear());
            r = cur2_q.angularDistance(ori_q);
            d = (result_2.translation() - init.translation()).norm();
            // d_rot = ( init_tr * result_2.linear() ).log().norm();
            
            double g2 = d + r;
            out(0, i) = (g1 - g2) / (2 * h);
        }
    }

private:
    Eigen::Matrix<double, 7, 1> left_q, right_q;
    Eigen::Matrix<double, 7, 1> left_qinit, right_qinit;
    Eigen::Affine3d left_init, right_init, init, init_tr;

    std::shared_ptr<FrankaModelUpdater> init_left_arm, left_arm;
    std::shared_ptr<FrankaModelUpdater> right_arm, init_right_arm;
    std::shared_ptr<rot_jaco> rot_jaco_;
    Eigen::Matrix<double, 4, 4> init_;
};

// bool planning(ConstrainedProblem &cp, ompl::geometric::PathGeometric &path, std::string file_name)
bool planning(ConstrainedProblem &cp, std::string file_name)
{
    clock_t start_time = clock();
    ob::PlannerStatus stat = cp.solveOnce(true, file_name);
    clock_t end_time = clock();

    OMPL_INFORM("PLANNING TIME : %d s", (end_time - start_time) / CLOCKS_PER_SEC);
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
    std::vector<enum PLANNER_TYPE> planners = {RRT, PRM}; //RRTConnect

    std::cout << "init state   : " << start.transpose() << std::endl;
    std::cout << "target state : " << goal.transpose() << std::endl;

    auto constraint = std::make_shared<KinematicChainConstraint>(links, start);

    ConstrainedProblem cp(space, ss, constraint); // define a simple problem to solve this constrained space
    cp.setConstrainedOptions();
    cp.setAtlasOptions();
    cp.setStartAndGoalStates(start, goal);
    
    
    // Get the maximum value a call to distance() can return (or an upper bound). For unbounded state spaces, this function can return infinity. More
    // cout << cp.css->getMaximumExtent() << endl; // 18.4372
    
    cp.ss->setStateValidityChecker(std::make_shared<ConstrainedKinematicChainValidityChecker>(cp.csi));
    

    cp.setPlanner(planners[1]);
    return (planning(cp, file_name));
    // return (planningBench(cp, planners));
     
}

void execute_path(std::string path_name, moveit::planning_interface::MoveGroupInterface& move_group, double total_time = 4)
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
    Matrix<double, 12, 14> goal_lists;

//     goal_lists <<1.60441, 1.51589, -1.45361, -2.56111, 2.84069, 3.23779, -0.408184, 0.142083, 0.0770971, 0.126544, -1.61819, 0.162839, 1.6443, 2.04722, 
// // Info:    distance : 1.849210
// 1.70253, 1.68999, -1.51646, -2.80147, 2.7825, 2.95227, -0.407189, 0.184554, 0.166405, 0.162883, -1.29117, 0.039396, 1.34764, 2.03339, 
// // Info:    distance : 1.988827
// 1.71381, 1.22035, -1.78547, -2.63535, 2.40889, 3.21138, -0.417845, 0.0939303, 0.204398, 0.126659, -1.28535, 0.23356, 1.36722, 1.98838, 
// // Info:    distance : 2.473171
// 1.79965, 0.648494, -1.96255, -2.28855, 2.40195, 3.6491, -0.733818, -0.00722323, 0.244964, 0.0963148, -1.33145, 0.456569, 1.3986, 1.92996, 
// // Info:    distance : 6.294319
// 1.636, 1.14716, -2.11518, -2.45973, 0.666354, 2.5112, 1.15601, -0.102159, 0.288934, 0.0876561, -1.4165, 0.665698, 1.4317, 1.85994, 
// // Info:    distance : 1.598366
// 1.40062, 1.04375, -2.10725, -2.13584, 0.666705, 2.15419, 1.18663, -0.176245, 0.345017, 0.0864277, -1.51924, 0.863659, 1.45673, 1.77731, 
// // Info:    distance : 1.831908
// 1.0835, 0.956393, -2.01698, -1.78039, 0.731046, 1.80282, 1.22334, -0.228884, 0.421713, 0.0919038, -1.6214, 1.05405, 1.46391, 1.68256, 
// // Info:    distance : 2.071808
// 0.713381, 0.830654, -1.80015, -1.38706, 0.760083, 1.50548, 1.33732, -0.242044, 0.523148, 0.0829702, -1.70956, 1.24387, 1.44451, 1.57803, 
// // Info:    distance : 2.672184
// 0.15331, 0.805997, -1.42052, -0.842003, 0.868714, 1.18596, 1.52904, -0.246375, 0.655975, 0.0983988, -1.77093, 1.42343, 1.39962, 1.47355;

    goal_lists << 1.14669, -1.38017, -1.45742, -3.02049, -2.82783, 1.09462, 2.5943, 0.136339, 0.0771, 0.132456, -1.61833, 0.163069, 1.64452, 2.04722, 
 // distance : 1.79682
1.14621, -1.59073, -1.15525, -2.94124, -2.58406, 1.11348, 2.33476, 0.0441954, 0.128898, 0.130295, -1.41736, 0.252909, 1.44964, 1.99681, 
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
2.10085, -1.45043, -0.684826, -1.44756, -2.03038, 0.686014, 2.31258, -0.236162, 0.472911, 0.0717868, -1.70595, 1.19792, 1.45845, 1.61077, 
 // distance : 0.82212
2.18154, -1.39057, -0.673757, -1.17815, -2.02867, 0.703286, 2.27391, -0.239152, 0.55708, 0.0749066, -1.745, 1.31533, 1.43125, 1.54129, 
 // distance : 1.07191
2.28836, -1.292, -0.684402, -0.825018, -2.05174, 0.744845, 2.19023, -0.238986, 0.655853, 0.0894607, -1.77037, 1.42673, 1.39508, 1.47327;

    for (int i = 0; i < goal_lists.rows() - 1; i++)
    {   
        start = goal_lists.row(i);
        goal = goal_lists.row(i + 1);
        plannedPath(start, goal, to_string(i));
        execute_path("/home/jiyeong/catkin_ws/" + to_string(i) + "_path.txt", move_group);
    }
        
    // execute_path("/home/jiyeong/catkin_ws/result_path.txt", move_group);
}
