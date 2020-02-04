
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

#include "ConstrainedPlanningCommon.h"
#include "KinematicChain.h"
#include "panda_model_updater.h"
#include "jacobian.h"
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
        std::cout << isValidImpl(space, s) << std::endl;
        return isValidImpl(space, s);
    }
};

class KinematicChainConstraint : public ob::Constraint
{
public:
    /* ob::Constraint(a, b) : a is dimension of the ambient space, b is constraint ouputs*/
    KinematicChainConstraint(unsigned int links, Eigen::VectorXd start) : ob::Constraint(links, 1, 1)
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

bool planning(ConstrainedProblem &cp, ompl::geometric::PathGeometric &path, std::string file_name)
{
    clock_t start_time = clock();
    ob::PlannerStatus stat = cp.solveOnce(true);
    clock_t end_time = clock();

    // if (stat)
    // {
    //     path = cp.ss->getSolutionPath();
    //     path.interpolate();
    // }
    OMPL_INFORM("PLANNING TIME : %d s", (end_time - start_time) / CLOCKS_PER_SEC);
    return stat;
}

bool plannedPath(Eigen::VectorXd start, Eigen::VectorXd goal, std::string file_name)
{
    auto ss = std::make_shared<KinematicChainSpace>(links);
    enum SPACE_TYPE space = PJ; //"PJ", "AT", "TB"
    std::vector<enum PLANNER_TYPE> planners = {RRT}; //RRTConnect

    std::cout << "init state   : " << start.transpose() << std::endl;
    std::cout << "target state : " << goal.transpose() << std::endl;

    auto constraint = std::make_shared<KinematicChainConstraint>(links, start);

    ConstrainedProblem cp(space, ss, constraint); // define a simple problem to solve this constrained space
    cp.setConstrainedOptions();
    cp.setAtlasOptions();
    cp.setStartAndGoalStates(start, goal);
    cp.setPlanner(planners[0]);
    
    // Get the maximum value a call to distance() can return (or an upper bound). For unbounded state spaces, this function can return infinity. More
    // cout << cp.css->getMaximumExtent() << endl; // 18.4372
    cp.ss->setStateValidityChecker(std::make_shared<ConstrainedKinematicChainValidityChecker>(cp.csi));
    ompl::geometric::PathGeometric path(cp.csi);
    return (planning(cp, path, file_name));
     
}

void execute_path(std::string path_name, moveit::planning_interface::MoveGroupInterface& move_group, double total_time = 7)
{
    moveit_msgs::RobotTrajectory robot_trajectory;
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = move_group.getJointNames();
    cout << "file name : " << path_name << endl;
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
                cout << "wrong file: " << j << endl;
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
    cout << "PUBLISHED" << endl;
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

    // start.setZero();
    // goal.setZero();
    // start << -1.278022704113855, 0.7599595568823166, 0.8504105865120947, -2.2532133470941753, -1.1298611922879744, 2.521625636015966, 2.2355349196620695,  0.025852137525551332, -0.012156381609418291, 0.28454053715882205, -1.996989239487609, 0.1902084893722976, 1.937095545026834, 2.0326259015811394;
    // goal << 1.7774068066631177, -1.6071385556188158, -1.4847508757964791, -1.889774699489722, -0.9339935055363789, 1.6617390338964741, 2.582642192418717,  0.8231283220077812, 0.7854915462891668, -1.3605836081236864, -1.3645664916893638, 1.4984756557502872, 1.03801815440285, 1.8101569484820168;
    // plannedPath(start, goal, "1");
    // execute_path("/home/jiyeong/catkin_ws/1_path.txt", move_group);
    
    // start.setZero();
    // goal.setZero();
    // start << 1.7774068066631177, -1.6071385556188158, -1.4847508757964791, -1.889774699489722, -0.9339935055363789, 1.6617390338964741, 2.582642192418717,  0.8231283220077812, 0.7854915462891668, -1.3605836081236864, -1.3645664916893638, 1.4984756557502872, 1.03801815440285, 1.8101569484820168;
    // goal << 1.7964825792215433, -1.4608329426331506, -1.5601839208321042, -1.9023645158590405, -0.5018620082517232, 2.029607475617737, 2.6516940006355156, 1.9134626801325931, -0.8272316695119547, -1.6818612100242758, -1.8767001731565196, 0.46704141331047666, 2.10534748329074, 2.2395795202166964;
    // plannedPath(start, goal, "2");
    // execute_path("/home/jiyeong/catkin_ws/2.txt", move_group);
    // ros::WallDuration(10.0).sleep();

    // start.setZero();
    // goal.setZero();
    // start << 1.7964825792215433, -1.4608329426331506, -1.5601839208321042, -1.9023645158590405, -0.5018620082517232, 2.029607475617737, 2.6516940006355156, 1.9134626801325931, -0.8272316695119547, -1.6818612100242758, -1.8767001731565196, 0.46704141331047666, 2.10534748329074, 2.2395795202166964;
    // goal <<  1.699059698012825, -1.7031600686437707, -1.5601096577679954, -1.742651411228564, -0.26001397984819996, 1.859883480955693, 2.4405694939121227, 0.252032751296342, 0.8481168795391635, -0.7850492071727834, -1.7340252218533394, 1.7612539500692608, 0.8689462263242886, 1.5201191679260264;
    // plannedPath(start, goal, "3");


    // 0.6cm 간격일때
    start.setZero();
    goal.setZero();
    start << 1.724232164642815, -1.6869073124631602, -1.4320308750075856, -1.6922132580732796, -0.9959997578244485, 1.4547076011710542, 2.410820584020732, 2.345953104979997, -0.9109013835952869, -1.814988937554947, -1.4927633631005977, -7.949861612059433e-05, 1.8887562745648154, 2.400458215415575;
    goal << -1.39258467601273, 1.1450542368090963, 1.3567036879943393, -1.7602722008928466, -0.10617590217056297, 1.912094626772453, 2.2625711904628996, -1.1050391746038768, 1.1816216109655286, 1.5050052121830346, -1.9737531616591852, 0.054494485403056375, 2.3870565089748568, 2.4392020251436484;

    plannedPath(start, goal, "2");
    execute_path("/home/jiyeong/catkin_ws/projection_path.txt", move_group);
  

}
